#! /usr/bin/python
"""
The main class of the cooperative control simulation.
The script starts the GUI from which commands can be sent to FlightGear,
and starts the control thread.
The control thread here computes a rendezvous trajectory dor the UAV and UGV using the MPC
found in control/MPC.
"""
from __future__ import print_function
import sys
import os
import time
import math
import threading
import csv
import numpy as np
import matplotlib.pyplot as plt
from fgpython import SimulationGUI, FGTelnetConnection
from control import Positioner, ControllerOSQP, ControllerPID,  Kalman
from control import ControllerOSQPRobust, ControllerOSQPRobustVariableHorizon
from control import get_horizontal_dynamics, get_vertical_dynamics
from control import get_y_ugv, get_y_uav, get_h_uav
from control import Q_vrt, R_vrt, Fset_vrt, Yset_vrt, W_vrt
from control import Q_hrz, R_hrz, Fset, Yset, W

from datetime import datetime
from fgsock import UDPConnect, vector_of_double, InputProtocol, UAVProtocol


# REACHABLE SET
# ----------------------------------------------------------
with open("control/A.csv", "r") as f:
    readerA = csv.reader(f, delimiter=",")
    A = np.array(list(readerA)).astype("float")
with open("control/B.csv", "r") as f:
    readerB = csv.reader(f, delimiter=",")
    b = np.array(list(readerB)).astype("float")


# MAIN PROGRAM PARAMETERS
# ----------------------------------------------------------
# ORIGIN = (42.186702238384, -71.00457277413)
ORIGIN_FG = (42.37824878120545, -71.00457885362507)
HEADING = 199.67
POSITIONER = Positioner(ORIGIN_FG, HEADING)
UAV_START = POSITIONER.get_global_position(-70, 8)
UGV_START = POSITIONER.get_global_position(10, 0)
FEET2M = 0.3048

# The path to the FlightGear folder
PATH = os.path.dirname(os.path.abspath(__file__))
SCRIPT = os.path.join(PATH, 'flightgear/run_flightgear.sh')

# Telnet connection for commands
ugv_ = FGTelnetConnection('localhost', 6600)
uav_ = FGTelnetConnection('localhost', 6500)


class Vehicle(object):
    id_counter = 0
    path = SCRIPT

    def __init__(self, name, type_):
        self.name = name
        self.id = type_ + str(self.id_counter)
        self.type = type_
        self.running = False


uav = Vehicle("Aerial Vehicle", "uav")
ugv = Vehicle("Ground Vehicle", "ugv")

ugv.control_variables = {'Velocity': {'range': (0, 35), 'value': 23},
                         'Heading': {'range': (-180, 180), 'value': 0},
                         'Acceleration': {'range': (-5, 5), 'value': 0}}

uav.control_variables = {'Altitude': {'range': (0, 100), 'value': 18},
                         'Velocity': {'range': (15, 35), 'value': 23},
                         'Heading': {'range': (-180, 180), 'value': 0},
                         'Acceleration': {'range': (-5, 5), 'value': 0},
                         'Gamma': {'range': (-15, 15), 'value': 0}}


class MainSimulation(object):
    UAV_FILE = './logs/Rascal.csv'
    UGV_FILE = './logs/ground-vehilce.csv'
    ap_mode = 'HOLD'
    uav_state = np.zeros((8, 1))
    ugv_state = np.zeros((5, 1))
    uav_input = InputProtocol
    ugv_input = InputProtocol
    uav_time = 0
    ugv_time = 0
    plot_it = 0
    final_stage = False
    h_s = 2
    d_s = 2.3
    d_l = 0.5
    itr = 0
    count = 0

    def __init__(self):
        self.ds = 0.1
        self.Nmax = 300
        self.update_rate = 100
        self.N_data = 1000
        self.v_ref = 20  # m/s

        self.PID = ControllerPID(self.v_ref, 0.1, 0.1, 0.05, 0.1)

        self.uav_state = np.zeros((8, 1))
        self.ugv_state = np.zeros((5, 1))
        self.last_input = np.zeros((5, 1))
        self.ugv_data = np.zeros((self.N_data, 7))*np.NaN
        self.uav_data = np.zeros((self.N_data, 11))*np.NaN
        self.time = np.zeros((self.N_data))*np.NaN
        self.solve_time = []
        self.solve_horizon = []

        A_c_vrt, B_c_vrt, C_vrt, D_vrt = get_vertical_dynamics()
        A_c, B_c, C_hrz, D_hrz, H_hrz, G_hrz = get_horizontal_dynamics(
            self.v_ref)

        self.hparam_uav = get_h_uav(self.ds)
        self.yparam_uav = get_y_uav(self.ds)
        self.yparam_ugv = get_y_ugv(self.ds)

        print(self.yparam_uav)
        print(self.yparam_ugv)
        self.kalman_hrz = Kalman(A_c, B_c)
        self.kalman_vrt = Kalman(A_c_vrt, B_c_vrt)

        nx = A_c.shape[0]
        nu = B_c.shape[1]
        self.nvar = nx + nu

        ny = C_hrz.shape[0]
        np.set_printoptions(precision=3, linewidth=210, threshold=2000,
                            edgeitems=15, suppress=True)

        Phi = np.eye(nx)*self.ds + \
            (np.linalg.matrix_power(A_c, 1)*self.ds**2)/2 + \
            (np.linalg.matrix_power(A_c, 2)*self.ds**3)/6 + \
            (np.linalg.matrix_power(A_c, 3)*self.ds**4)/24
        A_hrz = np.eye(nx) + np.matmul(A_c, Phi)
        B_hrz = np.matmul(Phi, B_c)

        Phi = np.eye(A_c_vrt.shape[0])*self.ds + \
            (np.linalg.matrix_power(A_c_vrt, 1)*self.ds**2)/2 + \
            (np.linalg.matrix_power(A_c_vrt, 2)*self.ds**3)/6 + \
            (np.linalg.matrix_power(A_c_vrt, 3)*self.ds**4)/24
        A_vrt = np.eye(A_c_vrt.shape[0]) + np.matmul(A_c_vrt, Phi)
        B_vrt = np.matmul(Phi, B_c_vrt)

        self.mpc_hrz = ControllerOSQPRobustVariableHorizon(
            A_hrz, B_hrz, C_hrz, D_hrz, self.Nmax, self.ds, 4)

        self.mpc_vrt = ControllerOSQPRobust(
            A_vrt, B_vrt, C_vrt, D_vrt, self.Nmax, self.ds, 2)

        Y, Q = self.mpc_hrz.reachability_matrices(W, Fset, Yset, 25)
        self.mpc_hrz.set_cost_matrix(Q_hrz, R_hrz, Q_hrz*10, H_hrz, G_hrz)
        self.mpc_hrz.set_inequality_constraints(Y)
        self.mpc_hrz.set_terminal_constraint(Q)
        self.mpc_hrz.setup_problems()

        Y, Q = self.mpc_vrt.reachability_matrices(
            W_vrt, Fset_vrt, Yset_vrt, 20)
        self.mpc_vrt.set_cost_matrix(Q_vrt, R_vrt, Q_vrt)
        self.mpc_vrt.set_inequality_constraints(Y)
        self.mpc_vrt.set_terminal_constraint(Q)
        self.mpc_vrt.setup_problems()

        self.init_time = time.time()

    def start_control_thread(self):
        """ Start thread for sending current control commands"""
        # uav_ctrl.start()
        # ugv_ctrl.start()

        self.control = True
        self.next_call = time.time()

        self.control_thread()

    def stop_control_thread(self):
        """ Stop thread for sending current control commands"""
        self.control = False

    def control_thread(self):
        """ Thread for continousy sending control commands to JSB. """
        if not self.control:
            return

        self.next_call = max(self.next_call + 1/self.update_rate, time.time())

        if self.ap_mode != 'HOLD':
            self.update_state()
            if self.count == 10:
                self.compute_control()
                self.count = 0
            else:
                self.count += 1

        self.send_command('both')

        threading.Timer(self.next_call - time.time(),
                        self.control_thread).start()

    def compute_control(self):
        """ Calculates the desired inputs when system is
            in landing mode """
        # Current state
        state = ([self.uav_state[i, 0] for i in [0, 1, 3, 4, 6, 7]]
                 + [self.ugv_state[i, 0] for i in [0, 1, 2, 3, 4]])

        # Current input
        input_ = [uav_ctrl.setpoint.a_ref, math.radians(uav_ctrl.setpoint.psi_ref),
                  ugv_ctrl.setpoint.a_ref, math.radians(ugv_ctrl.setpoint.psi_ref)]

        if self.final_stage:
            self.compute_mpc(state, input_)
        else:
            self.compute_pid(state)

    def compute_pid(self, state):
        """ Computes control inputs that will drive the system to the final stage.
            Inputs: state_t The state at time t """
        deltax = self.uav_state[0, 0] - self.ugv_state[0, 0]
        deltay = self.uav_state[1, 0] - self.ugv_state[1, 0]
        deltau = (self.uav_state[3, 0]*math.cos(self.uav_state[6, 0])
                  - self.ugv_state[2, 0]*math.cos(self.ugv_state[4, 0]))
        deltav = (self.uav_state[3, 0]*math.sin(self.uav_state[6, 0])
                  - self.ugv_state[2, 0]*math.sin(self.ugv_state[4, 0]))

        v_uav, v_ugv = self.PID.get_control(deltax, deltay, deltav)

        uav_ctrl.setpoint.v_ref = max(min(28.0, v_uav), 18.0)     # [m/s]
        ugv_ctrl.setpoint.v_ref = max(min(30.0, v_ugv), 0)     # [m/s]

        if abs(deltax) < 35:
            uav_.landing_mode()
            ugv_.landing_mode()
            self.final_stage = True
            self.mpc_hrz.initial_solve(state)

    def compute_mpc(self, state_t, input_t):
        """ Computes the optimal path for the final stage of the landing
            The method is MPC with two spearate controllers.
            Inputs: state_t The state at time t
                    input_t The input at time t """
        # try:
        # Save the past 4 UAV heading inputs
        start_time = time.time()

        if False:
            res = self.mpc_hrz.solve(state_t)

            u0 = res.x[0]  # Thrust UAV
            u1 = res.x[1]  # Steering UAV
            u2 = res.x[2]  # Thrust UGV
            u3 = res.x[3]  # Steering UGV

            x1 = res.x[np.arange(4, self.nvar*self.Nmax, self.nvar)]
            y1 = res.x[np.arange(5, self.nvar*self.Nmax, self.nvar)]
            v1 = res.x[np.arange(6, self.nvar*self.Nmax, self.nvar)]
            x2 = res.x[np.arange(10, self.nvar*self.Nmax, self.nvar)]
            y2 = res.x[np.arange(11, self.nvar*self.Nmax, self.nvar)]
            v2 = res.x[np.arange(12, self.nvar*self.Nmax, self.nvar)]

            distance = np.array([math.sqrt((x1[i]-x2[i])**2 + (y1[i]-y2[i])**2)
                                 for i in range(len(x1))])

            scaling = np.ones((self.mpc_vrt.ni, 1))
            scaling[np.arange(1, self.mpc_vrt.ni, self.mpc_vrt.ny), 0] = np.maximum(
                np.minimum((self.h_s * distance - self.h_s*self.d_l)/(self.d_s-self.d_l), self.h_s), -0.01)

            lower_bound = self.mpc_vrt.ineq_l.copy()
            lower_bound[self.mpc_vrt.ne:self.mpc_vrt.ni +
                        self.mpc_vrt.ne] *= scaling

            print("1 deltax = ", x1[0:20] - x2[0:20])

        else:
            status, path, N = self.mpc_hrz.solve(state_t)
            #N = self.Nmax

            x1 = path[np.arange(4, N*self.nvar, self.nvar)]
            x2 = path[np.arange(10, N*self.nvar, self.nvar)]
            y1 = path[np.arange(5, N*self.nvar, self.nvar)]
            y2 = path[np.arange(11, N*self.nvar, self.nvar)]
            u0 = path[0]
            u1 = path[1]
            u2 = path[2]
            u3 = path[3]

            dist = np.array([math.sqrt((x1[i]-x2[i])**2 + (y1[i]-y2[i])**2)
                             for i in range(len(x1))])

            scale = np.ones((self.mpc_vrt.ni, 1))
            scale[np.arange(1, N*self.mpc_vrt.ny,
                            self.mpc_vrt.ny), 0] = np.maximum(
                                np.minimum((self.h_s * dist -
                                            self.h_s*self.d_l)/(self.d_s-self.d_l), self.h_s),
                                -0.01)

            scale[np.arange(N*self.mpc_vrt.ny+1, self.mpc_vrt.ni, self.mpc_vrt.ny),
                  0] = np.ones(((self.Nmax-N)))*-0.01

            lower_bound = self.mpc_vrt.ineq_l.copy()
            lower_bound[self.mpc_vrt.ne: self.mpc_vrt.ni +
                        self.mpc_vrt.ne] *= scale

            print("SOLVER = %i, N = %i" % (status, N))
            print("1 deltax = ", x1[0:20] - x2[0:20])

        uav_ctrl.setpoint.a_ref = u0  # [m/s2]
        ugv_ctrl.setpoint.a_ref = u2  # [m/s2]
        uav_ctrl.setpoint.psi_ref = u1  # [rad/s]
        ugv_ctrl.setpoint.psi_ref = u3  # [rad/s]

        try:
            path_vrt, status = self.mpc_vrt.solve(
                np.array([[self.uav_state[2, 0] - 1.3, self.uav_state[5, 0]]]).T, lower_bound)
            u0_vrt = path_vrt[0]
            h0 = path_vrt[np.arange(1, 3*self.Nmax, 3)]
            g0 = path_vrt[np.arange(2, 3*self.Nmax, 3)]
            uav_ctrl.setpoint.gamma_ref = u0_vrt

        except:
            print("Could not solve vertical...: ", status)
            print("state = ", np.array(
                [[self.uav_state[2, 0] - 1.3, self.uav_state[5, 0]]]))
            u0_vrt = 0.0
            # print(distance)
            # print("SCALING = ", scaling[1:3:])

        self.last_input = np.array([[u0, u1, u2, u3, u0_vrt]]).T
        self.solve_time.append(time.time()-start_time)
        self.solve_horizon.append(N)
        # self.mpc_vrt.update_equality_constraints(0, 1, v1*self.hparam_uav)

        # self.mpc_hrz.update_equality_constraints(1, 4, v1*self.yparam_uav[0])
        # self.mpc_hrz.update_equality_constraints(1, 5, v1*self.yparam_uav[1])

        # self.mpc_hrz.update_equality_constraints(7, 10, v2*self.yparam_ugv)

    def send_command(self, vehicle):
        """ Send command to control system of vehicle. """
        if vehicle == 'both':
            uav_ctrl.send_cmd()
            ugv_ctrl.send_cmd()
        elif vehicle == 'uav':
            uav_ctrl.send_cmd()
        elif vehicle == 'ugv':
            ugv_ctrl.send_cmd()
        else:
            raise ValueError(
                "Acceptable arguments are 'uav', 'ugv' or 'both'. ")

    def update_state(self):
        """ Send command to control system of vehicle. """
        time_now = (datetime.now() - datetime.now().replace(hour=0,
                                                            minute=0, second=0, microsecond=0)).total_seconds() - 3600
        uav_state = uav_ctrl.return_uav()
        ugv_state = ugv_ctrl.return_ugv()

        hrz_state = np.array([[uav_state.x, uav_state.y, uav_state.v,
                               uav_state.a, uav_state.psi -
                               math.radians(HEADING), uav_state.phi,
                               ugv_state.x, ugv_state.y, ugv_state.v,
                               ugv_state.a, ugv_state.psi-math.radians(HEADING)]]).T

        vrt_state = np.array([[uav_state.h, uav_state.gamma]]).T

        if self.itr == 0:
            self.kalman_hrz.x = hrz_state
            self.kalman_vrt.x = vrt_state

        new_uav = not (uav_state.t == self.uav_time)
        new_ugv = not (ugv_state.t == self.ugv_time)
        self.uav_time = uav_state.t
        self.ugv_time = ugv_state.t

        # Predict state
        self.kalman_hrz.predict(self.last_input[0:4, :], 1/self.update_rate)
        self.kalman_vrt.predict(
            self.last_input[4, None, :], 1/self.update_rate)

        # Correct
        if new_uav:
            # Aerial vehicle
            deltat = time_now - uav_state.t
            hrz_state[0:6, 0] = self.kalman_hrz.prop_uav(hrz_state[0:6, :],
                                                         self.last_input[0:2, :], deltat)
            self.kalman_hrz.update_uav(hrz_state[0:6, :])
            self.kalman_vrt.update_alt(vrt_state)

        if new_ugv:
            # Ground vehicle
            deltat = time_now - ugv_state.t  #
            hrz_state[6:11, 0] = self.kalman_hrz.prop_ugv(hrz_state[6:11, :],
                                                          self.last_input[2:4, :], deltat)
            self.kalman_hrz.update_ugv(hrz_state[6:11, :])

        self.uav_state[[2, 5], 0] = self.kalman_vrt.x[:, 0]
        self.uav_state[[0, 1, 3, 4, 6, 7], 0] = self.kalman_hrz.x[0: 6, 0]
        self.ugv_state[0: 5, 0] = self.kalman_hrz.x[6:, 0]

        self.uav_data[self.itr % self.N_data, :] = [*self.uav_state[0:8, 0],
                                                    *self.last_input[[0, 1, 4], :]]
        self.ugv_data[self.itr % self.N_data, :] = [*self.ugv_state[0:5, 0],
                                                    *self.last_input[[2, 3], :]]

        self.time[self.itr % self.N_data] = time_now
        self.itr += 1

        if uav_state.h < 1.6:
            print("SUCCESSFUL LANDING")
            uav_.pause()
            ugv_.pause()
            self.stop_control_thread()


# -------------------- GUI CLASS -------------------------


class MyGui(SimulationGUI):
    """ GUI for sending commands to FlightGear. """
    ap_mode = 'HOLD'

    def __init__(self, simulation, vehicles):
        SimulationGUI.__init__(self, vehicles)
        # Initial GUI setup
        for group in self.groups.values():
            for button in group.buttons():
                if button.isChecked():
                    button.click()

        for vehicle in self.vehicles:
            for prop in vehicle.control_variables.values():
                slider = prop['slider']
                if str(slider.objectName()).lower() == "acceleration":
                    slider.setValue(uav_ctrl.setpoint.a_ref)
                elif str(slider.objectName()).lower() == "velocity":
                    slider.setValue(uav_ctrl.setpoint.v_ref)
                elif str(slider.objectName()).lower() == "altitude":
                    slider.setValue(uav_ctrl.setpoint.h_ref)
                elif str(slider.objectName()).lower() == "heading":
                    slider.setValue(uav_ctrl.setpoint.psi_ref)
                elif str(slider.objectName()).lower() == "gamma":
                    slider.setValue(uav_ctrl.setpoint.gamma_ref)

                else:
                    slider.setValue(
                        uav_ctrl.setpoint[str(slider.objectName()).lower()])

        self.sim = simulation
        self.sim.send_command('both')
        # Start the control thread
        self.sim.start_control_thread()

    def stop_control(self):
        self.sim.stop_control_thread()


# :::::::::::::::::: UPDATE DATA :::::::::::::::::::::::::::


ugv_ctrl = UDPConnect(5526, 5525, 'InputProtocol',
                      'UGVProtocol', "127.0.0.1")
uav_ctrl = UDPConnect(5515, 5514, 'InputProtocol',
                      'UAVProtocol', "127.0.0.1")
uav_ctrl.heading = math.radians(HEADING)
ugv_ctrl.heading = math.radians(HEADING)
uav_ctrl.setpoint.h_ref = 20.0
uav_ctrl.setpoint.v_ref = 20.0


# Vehicle class
uav.command = uav_
ugv.command = ugv_
uav.control = uav_ctrl
ugv.control = ugv_ctrl


uav.mp_output_port = 5002
uav.mp_input_port = 5000
ugv.mp_output_port = 5000
ugv.mp_input_port = 5002

uav.arguments = {'aircraft': 'Rascal110-JSBSim',
                 # 'lat': UAV_START[0] , 'lon':UAV_START[1],
                 'prop:/position/ref-origin-lat': ORIGIN_FG[0],
                 'prop:/position/ref-origin-lon': ORIGIN_FG[1],
                 'lat': UAV_START[0], 'lon': UAV_START[1],
                 'altitude': 20,
                 'uBody': 25,
                 'heading': 199,
                 'glideslope': 0,
                 'roll': 0,
                 'prop:/position/ref-heading': HEADING,
                 }

ugv.arguments = {'aircraft': 'ground-vehicle',
                 # 'airport': 'KBOS',
                 'prop:/position/ref-origin-lat': ORIGIN_FG[0],
                 'prop:/position/ref-origin-lon': ORIGIN_FG[1],
                 # 'runway': '22R',
                 'lat': UGV_START[0], 'lon': UGV_START[1],
                 'heading': 199,
                 # 'altitude': 0,
                 }
# -------------------- MAIN CLASS -------------------------

sim = MainSimulation()


from PyQt5.QtGui import QApplication

app = QApplication([])   # THIS MAKES SOMETHING GO WRONG

gui = MyGui(sim, [uav, ugv])

gui.show()
app.exec_()


if sim.itr > sim.N_data:
    print("More data than maximum")
    sim.uav_data = np.roll(
        sim.uav_data, -(sim.itr % sim.N_data), axis=0)
    sim.ugv_data = np.roll(
        sim.ugv_data, -(sim.itr % sim.N_data), axis=0)
    sim.time = np.roll(
        sim.time, -(sim.itr % sim.N_data), axis=0)
    t1 = sim.time[-1]
else:
    print("less data than maximum")
    t1 = sim.time[sim.itr-1]


if sim.itr > 0:
    plt.figure("STATES", (16, 10))
    t0 = sim.time[0]
    plt.suptitle("Simulation states")
    plt.subplot(3, 2, 1)
    plt.title("Relative position")
    plt.plot(sim.time-t0,
             sim.uav_data[:, 0]-sim.ugv_data[:, 0], label='deltax')
    plt.plot(sim.time-t0,
             sim.uav_data[:, 1]-sim.ugv_data[:, 1], label='deltay')
    plt.plot([0, t1-t0], [sim.d_l, sim.d_l], color='r')
    plt.plot([0, t1-t0], [-sim.d_l, -sim.d_l], color='r')
    plt.xlabel("Time [s]")
    plt.ylabel("Distance [m]")
    plt.ylim([-5, 30])
    plt.legend()

    plt.subplot(3, 2, 2)
    plt.title("Altitude")
    plt.xlabel("Time [s]")
    plt.ylabel("Altitude [m]")
    plt.ylim([0, 18])
    plt.plot(sim.time-t0, sim.uav_data[:, 2], label='UAV')

    plt.subplot(3, 2, 3)
    plt.title("Velocity")
    plt.ylabel("Velocity")
    plt.xlabel("Time [s]")
    plt.plot(sim.time-t0, sim.uav_data[:, 3], label='UAV')
    plt.plot(sim.time-t0, sim.ugv_data[:, 2], label='UGV')
    plt.legend()

    plt.subplot(3, 2, 4)
    plt.title("Acceleration")
    plt.ylabel("Acceleration")
    plt.xlabel("Time [s]")
    plt.plot(sim.time-t0, sim.uav_data[:, 4], label='UAV')
    plt.plot(sim.time-t0, sim.ugv_data[:, 3], label='UGV')
    plt.legend()

    plt.subplot(3, 2, 5)
    plt.xlabel("Time [s]")
    plt.ylabel("Degrees")
    plt.title("Heading")
    plt.plot(sim.time-t0, np.degrees(sim.uav_data[:, 6]), label='UAV')
    plt.plot(sim.time-t0, np.degrees(sim.uav_data[:, 7]), label='Roll')
    plt.plot(sim.time-t0, np.degrees(sim.ugv_data[:, 4]), label='UGV')
    plt.ylim([-25, 25])
    plt.legend()

    plt.subplot(3, 2, 6)
    plt.xlabel("Time [s]")
    plt.ylabel("Degrees")
    plt.title("Gamma")
    plt.plot(sim.time-t0, np.degrees(sim.uav_data[:, 5]), label='UAV')
    plt.ylim([-20, 20])
    plt.legend()
    plt.subplots_adjust(left=None, bottom=None, right=None,
                        top=None, wspace=None, hspace=0.4)

    plt.figure("INPUTS", (16, 10))

    plt.subplot(1, 3, 1)
    plt.xlabel("Time [s]")
    plt.ylabel("Input [m/s^2]")
    plt.title("Thrust inputs")
    plt.step(sim.time-t0, sim.uav_data[:, 8], label='UAV')
    plt.step(sim.time-t0, sim.ugv_data[:, 5], label='UGV')
    plt.ylim([-5, 5])
    plt.legend()
    plt.subplot(1, 3, 2)
    plt.xlabel("Time [s]")
    plt.ylabel("Angles [deg]")
    plt.title("Heading inputs")
    plt.step(sim.time-t0, np.degrees(sim.uav_data[:, 9]), label='UAV')
    plt.step(sim.time-t0, np.degrees(sim.ugv_data[:, 6]), label='UGV')
    plt.ylim([-35, 35])
    plt.legend()

    plt.subplot(1, 3, 3)
    plt.xlabel("Time [s]")
    plt.ylabel("Angles [deg]")
    plt.title("Gamma input")
    plt.step(sim.time-t0, np.degrees(sim.uav_data[:, 10]), label='Input')
    plt.plot(sim.time-t0, np.degrees(sim.uav_data[:, 5]), label='Gamma')
    plt.ylim([-45, 35])
    plt.legend()
    plt.suptitle("INPUTS")
    plt.subplots_adjust(left=None, bottom=None, right=None,
                        top=None, wspace=None, hspace=0.4)

    fig, ax1 = plt.subplots()
    ax1.plot(sim.solve_time, color='blue')
    ax1.set_ylabel('Solver time', color='blue')
    ax1.tick_params(axis='y', labelcolor='blue')
    ax2 = ax1.twinx()  # instantiate a second axes that shares the same x-axis
    ax2.plot(sim.solve_horizon, color='green')
    ax2.set_ylabel('Horizon', color='green')
    ax2.tick_params(axis='y', labelcolor='green')

    plt.title("MPC solve time")
    fig.tight_layout()  # otherwise the right y-label is slightly clipped
    plt.show()

now = datetime.now()  # current date and time
date_time = now.strftime("%y%m%d_%H-%M-%S")
print("date and time:", date_time)

np.savetxt("uav" + date_time + ".csv", sim.uav_data, delimiter=",")
np.savetxt("ugv" + date_time + ".csv", sim.ugv_data, delimiter=",")
np.savetxt("time" + date_time + ".csv", sim.time, delimiter=",")

solver = np.array([sim.solve_horizon, sim.solve_time]).T
np.savetxt("solver" + date_time + ".csv", solver, delimiter=",")
