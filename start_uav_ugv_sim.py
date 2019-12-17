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
import tempfile
from PyQt5 import QtGui, QtCore
from fgpython import SimulationGUI, FGTelnetConnection, FGSocketConnection
from control import MPC, Positioner


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
UAV_START = POSITIONER.get_global_position(-70, 5)
UGV_START = POSITIONER.get_global_position(10, 0)
FEET2M = 0.3048

# The path to the FlightGear folder
PATH = os.path.dirname(os.path.abspath(__file__))
SCRIPT = os.path.join(PATH, 'flightgear/run_flightgear.sh')

# Telnet connection for commands
ugv_ = FGTelnetConnection('localhost', 6600)
uav_ = FGTelnetConnection('localhost', 6500)

# Socket connection for control
FGSocketConnection.heading = HEADING
ugv_ctrl = FGSocketConnection(5526, 5525, 'InputProtocol', 'UGVProtocol')
uav_ctrl = FGSocketConnection(5515, 5514, 'InputProtocol', 'UAVProtocol')
uav_ctrl.setpoint['altitude'] = 18


# Vehicle class
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


class MainSimulation(object):
    UAV_FILE = './logs/Rascal.csv'
    UGV_FILE = './logs/ground-vehilce.csv'
    ap_mode = 'HOLD'
    uav_state = []
    ugv_state = []
    plot_it = 0
    past_inputs = [[0, 0, 0, 0], [0, 0, 0, 0],
                   [0, 0, 0, 0], [0, 0, 0, 0], [0, 0, 0, 0]]
    final_stage = False
    d_safe = 1.8
    deltax = []
    deltay = []
    deltah = []
    velocity_uav = []
    velocity_ugv = []
    acceleration_uav = []
    acceleration_ugv = []
    acceleration_uav_des = []
    acceleration_ugv_des = []
    heading_uav = []
    heading_ugv = []
    heading_uav_des = []
    heading_ugv_des = []
    uav_path = []
    ugv_path = []
    ta = []
    tg = []
    start_final = None

    def __init__(self):
        self.ds = 0.18
        self.mpc, self.mpc_alt = self.initalize_mpc()
        self.init_time = time.time()

    def initalize_mpc(self):
        """ Initialize the two MPC with the correct values """

        T = 38
        mpc = MPC.Controller(MPC.state_constraints_lat,
                             MPC.input_constraints_lat,
                             MPC.A_lat, MPC.B_lat, T, self.ds)

        mpc_alt = MPC.Controller(MPC.state_constraints_lon,
                                 MPC.input_constraints_lon,
                                 MPC.A_lon, MPC.B_lon, T, self.ds)
        MPC.add_align_constraints(mpc)
        MPC.add_alt_constraints(mpc_alt)

        return mpc, mpc_alt

    def start_control_thread(self):
        """ Start thread for sending current control commands"""
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
        self.next_call = max(self.next_call + self.ds, time.time())
        if self.ap_mode != 'HOLD':
            self.get_state()
            self.calculate_control()
        self.send_command('both')

        threading.Timer(self.next_call - time.time(),
                        self.control_thread).start()

    def calculate_control(self):
        """ Calculates the desired inputs when system is
            in landing mode """
        # Current state
        state = ([self.uav_state[i] for i in [0, 1, 3, 4, 6]]
                 + [self.ugv_state[i] for i in [0, 1, 2, 3, 4]])

        # Current input
        input_ = [uav_ctrl.setpoint['acceleration'], math.radians(uav_ctrl.setpoint['heading']),
                  ugv_ctrl.setpoint['acceleration'], math.radians(ugv_ctrl.setpoint['heading'])]

        # Save the old input values
        # Vector: [x(t-4), x(t-3), x(t-2),..., x(t-1), x(t)]
        self.past_inputs[0:-1] = self.past_inputs[1:]
        self.past_inputs[-1] = [0,
                                math.radians(uav_ctrl.setpoint['heading']), 0, 0]

        if self.final_stage:
            self.optimize_final_path(state, input_)
        else:
            self.compute_pid()

    def compute_pid(self):
        """ Computes control inputs that will drive the system to the final stage.
            Inputs: state_t The state at time t """
        deltax = self.uav_state[0] - self.ugv_state[0]
        deltay = self.uav_state[1] - self.ugv_state[1]
        deltau = (self.uav_state[3]*math.cos(self.uav_state[6])
                  - self.ugv_state[2]*math.cos(self.ugv_state[4]))
        deltav = (self.uav_state[3]*math.sin(self.uav_state[6])
                  - self.ugv_state[2]*math.sin(self.ugv_state[4]))

        # HEADING
        # heading_uav = (0 - 0.02*deltay - 0.0001*deltav)*180/3.14      # [deg]
        # heading_ugv = (0 + 0.005*deltay - 0.00001*deltav)*180/3.14    # [deg]
        # uav_ctrl.setpoint['heading'] = max(min(5, heading_uav), -5)
        # ugv_ctrl.setpoint['heading'] = max(min(5, heading_ugv), -5)

        # VELOCITY
        v_uav = 20 - 0.1*deltax - 0.1*deltau  # m/s
        v_ugv = 20 + 0.05*deltax + 0.1*deltau  # m/s
        uav_ctrl.setpoint['velocity'] = max(min(28, v_uav), 18)     # [m/s]
        ugv_ctrl.setpoint['velocity'] = max(min(30, v_ugv), 0)     # [m/s]
        x = np.array([deltax, deltay,
                      self.uav_state[3], self.uav_state[4], self.uav_state[6],
                      self.ugv_state[2], self.ugv_state[3], self.ugv_state[4]])[np.newaxis].T
        ans = not False in np.less_equal(np.dot(A, x), b)
        if (ans):
            self.start_final = [[len(self.deltax), self.deltax[-1]],
                                [len(self.deltay), self.deltay[-1]]]
            uav_.landing_mode()
            ugv_.landing_mode()
            self.final_stage = True

    def optimize_final_path(self, state_t, input_t):
        """ Computes the optimal path for the final stage of the landing
            The method is MPC with two spearate controllers.
            Inputs: state_t The state at time t
                    input_t The input at time t """
        try:
            # Save the past 4 UAV heading inputs
            self.mpc.u_delay.value = self.past_inputs[:-1]
            start_time = time.time()
            self.mpc.solve(state_t, input_t)

            # Retrieve the computed inputs
            inputs = self.mpc.u[:, 1].value

            uav_ctrl.setpoint['acceleration'] = inputs[0]     # [m/s2]
            ugv_ctrl.setpoint['acceleration'] = inputs[2]     # [m/s2]
            uav_ctrl.setpoint['heading'] = math.degrees(
                inputs[1])     # [rad/s]
            ugv_ctrl.setpoint['heading'] = math.degrees(
                inputs[3])     # [rad/s]

            x1 = self.mpc.x[0, :].value  # uav x
            x2 = self.mpc.x[1, :].value  # uav y
            x6 = self.mpc.x[5, :].value  # ugv x
            x7 = self.mpc.x[6, :].value  # ugv y

            if self.uav_path == []:
                va = self.mpc.x[2, :].value  # uav v
                aa = self.mpc.x[3, :].value  # uav a
                pa = self.mpc.x[4, :].value  # uav psi
                vg = self.mpc.x[7, :].value  # ugv v
                ag = self.mpc.x[8, :].value  # ugv a
                pg = self.mpc.x[9, :].value  # ugv psi

                pap = self.mpc.u[1, :].value  # ugv psi
                pgp = self.mpc.u[3, :].value  # ugv psi

                self.uav_path = [va, aa, x1, x2, pa, pap]
                self.ugv_path = [vg, ag, x6, x7, pg, pgp]

            distance = [math.sqrt((x1[i]-x6[i])**2 + (x2[i]-x7[i])**2)
                        for i in range(self.mpc.T)]

            b1 = [int(distance[i] < self.d_safe) for i in range(self.mpc.T)]
            b2 = [int(not distance[i] < self.d_safe)
                  for i in range(self.mpc.T)]

            self.mpc_alt.b1.value = b1
            self.mpc_alt.b2.value = b2
            self.mpc_alt.distance.value = distance

            print("time: ", time.time()-start_time)
        except:
            print("ERROR OCCURED: NO SOLUTION FOUND (LAT)")
            uav_.pause()
            ugv_.pause()
            self.stop_control_thread()

        try:
            result = self.mpc_alt.solve([self.uav_state[2] - 1.3,
                                         self.uav_state[5]],
                                        [math.radians(uav_ctrl.setpoint['gamma'])])
            u = self.mpc_alt.u[0, 1].value
            uav_ctrl.setpoint['gamma'] = math.degrees(u)
        except:
            print("ERROR OCCURED: NO SOLUTION FOUND (LON)")

            uav_ctrl.setpoint['gamma'] = 0.0
            # uav_ctrl.toggle_pause(1)
            # ugv_ctrl.toggle_pause(1)
            # self.stop_control_thread()

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

    def get_state(self):
        """ Send command to control system of vehicle. """
        # UAV = [0=x, 1=y, 2=h-agl-ft, 3=vt-fps, 4=udot-ft_sec2, 5=gamma-rad, 6=psi-gt-rad]
        # uav = uav_ctrl.get_state([13, 14, 10, 1, 2, 3, 4, 0])
        uav = uav_ctrl.get_state([1, 2, 3, 4, 5, 6, 7, 8, 0])
        # UAV = [0=x, 1=y, 2=vt-fps, 3=udot-ft_sec2, 4=psi-gt-rad, 5=h-agl-ft]
        # ugv = ugv_ctrl.get_state([9, 10, 2, 3, 5, 6, 0])
        ugv = ugv_ctrl.get_state([1, 2, 4, 5, 6, 3, 0])

        # Aerial vehicle
        uav[0] -= 0.5
        uav[2] *= FEET2M  # altitude
        uav[3] *= FEET2M  # velocity
        uav[4] *= FEET2M  # acceleration
        # uav[5] *= 1  # gamma [rad]
        uav[6] += -math.radians(HEADING)  # psi [rad]
        # Ground vehicle
        ugv[2] *= FEET2M  # velocity
        ugv[3] *= FEET2M  # acceleration
        ugv[4] += -math.radians(HEADING)  # psi [rad]
        ugv[5] *= FEET2M  # h

        deltat = uav[-1] - ugv[-1]

        # Approximate current acceleration, velocity and position
        if deltat > 0:  # UAV data is newer
            a = ugv[3]*(1-11.99*deltat) + 11.54*deltat * \
                ugv_ctrl.setpoint['acceleration']
            v = ugv[2] + deltat/2*(a + ugv[3])
            x = ugv[0] + deltat/2*(v + ugv[2])
            ugv[2] = v
            ugv[0] = x
        else:  # UGV data is newer
            deltat *= -1
            a = uav[4]*(1-1.93*deltat) + 1.776*deltat * \
                uav_ctrl.setpoint['acceleration']
            v = uav[3] + deltat/2*(a + uav[4])
            x = uav[0] + deltat/2*(v + uav[3])
            uav[3] = v
            uav[0] = x

        self.uav_state = uav
        self.ugv_state = ugv

        self.deltah.append(uav[2])
        self.deltax.append(uav[0] - ugv[0])
        self.deltay.append(uav[1] - ugv[1])

        self.velocity_uav.append(uav[3])
        self.velocity_ugv.append(ugv[2])

        self.acceleration_uav.append(uav[4])
        self.acceleration_ugv.append(ugv[3])
        self.acceleration_uav_des.append(uav_ctrl.setpoint['acceleration'])
        self.acceleration_ugv_des.append(ugv_ctrl.setpoint['acceleration'])

        self.heading_uav.append(math.degrees(uav[6]))
        self.heading_ugv.append(math.degrees(ugv[4]))

        self.heading_uav_des.append(uav_ctrl.setpoint['heading'])
        self.heading_ugv_des.append(ugv_ctrl.setpoint['heading'])

        self.ta.append(uav[-1])
        self.tg.append(ugv[-1])

        if uav[2] < 1.6:
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
                slider.setValue(
                    uav_ctrl.setpoint[str(slider.objectName()).lower()])

        self.sim = simulation
        self.sim.send_command('both')
        # Start the control thread
        self.sim.start_control_thread()
        uav_ctrl.start_receive_state()
        ugv_ctrl.start_receive_state()

    def stop_control(self):
        self.sim.stop_control_thread()


# :::::::::::::::::: UPDATE DATA :::::::::::::::::::::::::::
app = QtGui.QApplication(sys.argv)
simulation = MainSimulation()
gui = MyGui(simulation, [uav, ugv])
gui.show()
app.exec_()


print("Simulation has terminated")
