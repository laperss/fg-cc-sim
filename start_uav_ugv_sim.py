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
from control import get_lqr_feedback, get_mpc_sets, get_mpc_costs,  get_invariant_set


from kalmanlib import StateFilter,  UAVinput, UGVinput, UAVstate, UGVstate

from datetime import datetime
from fgsock import UDPConnect, vector_of_double, InputProtocol, UAVProtocol
import scipy

from controllerlib import MPC, mpc_x0, mpc_u0, solution


controller_mpc = MPC()


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
                         'Acceleration': {'range': (-2, 2), 'value': 0}}

uav.control_variables = {'Altitude': {'range': (0, 100), 'value': 18},
                         'Velocity': {'range': (15, 35), 'value': 23},
                         'Heading': {'range': (-180, 180), 'value': 0},
                         'Acceleration': {'range': (-4, 4), 'value': 0},
                         'Gamma': {'range': (-15, 15), 'value': 0},
                         'Yawrate': {'range': (-5, 5), 'value': 0}}


class MainSimulation(object):
    UAV_FILE = './logs/Rascal.csv'
    UGV_FILE = './logs/ground-vehilce.csv'
    ap_mode = 'HOLD'
    uav_state = np.zeros((8, 1))
    ugv_state = np.zeros((5, 1))
    uav_time = 0
    ugv_time = 0
    plot_it = 0
    final_stage = False
    h_s = 2
    d_s = 2.3
    d_l = 0.5
    itr = 0
    count = 1
    xpath = None

    def __init__(self):

        self.ds = 0.1
        self.Nmax = controller_mpc.Nmax

        self.update_rate = 100
        self.N_data = 2000
        self.v_ref = 20  # m/s

        self.time_last = time.time()
        self.PID = ControllerPID(self.v_ref, 0.1, 0.1, 0.05, 0.1)

        self.predicted_state = np.zeros((1, 11))
        self.last_input = np.zeros((5, 1))
        self.ugv_data = np.zeros((self.N_data, 7))*np.NaN
        self.uav_data = np.zeros((self.N_data, 17))*np.NaN

        self.time = np.zeros((self.N_data))*np.NaN
        self.solve_time = []
        self.solve_horizon = []
        self.solve_horizon2 = []
        self.solve_cost = []
        self.solve_cost2 = []

        A_c_vrt, B_c_vrt, C_vrt, D_vrt = get_vertical_dynamics()
        A_c, B_c, C_hrz, D_hrz, B_dc = get_horizontal_dynamics(self.v_ref)

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

        Phi = np.eye(nx)*self.ds + \
            (np.linalg.matrix_power(A_c, 1)*self.ds**2)/2 + \
            (np.linalg.matrix_power(A_c, 2)*self.ds**3)/6 + \
            (np.linalg.matrix_power(A_c, 3)*self.ds**4)/24
        A_hrz = np.eye(nx) + np.matmul(A_c, Phi)
        B_hrz = np.matmul(Phi, B_c)
        Bd_hrz = np.matmul(Phi, B_dc)

        Phi = np.eye(A_c_vrt.shape[0])*self.ds + \
            (np.linalg.matrix_power(A_c_vrt, 1)*self.ds**2)/2 + \
            (np.linalg.matrix_power(A_c_vrt, 2)*self.ds**3)/6 + \
            (np.linalg.matrix_power(A_c_vrt, 3)*self.ds**4)/24
        A_vrt = np.eye(A_c_vrt.shape[0]) + np.matmul(A_c_vrt, Phi)
        B_vrt = np.matmul(Phi, B_c_vrt)

        Q_vrt, R_vrt, Fset_vrt, Yset_vrt, W_vrt, Q_hrz, R_hrz, Fset, Yset, W, q_hrz, r_hrz = get_mpc_sets(
            A_hrz, B_hrz, Bd_hrz, A_vrt, B_vrt)

        Q_vrt, R_vrt, Q_hrz, R_hrz, q_hrz, r_hrz, Qf, qf, const_hrz = get_mpc_costs(
            A_hrz, B_hrz, Bd_hrz, A_vrt, B_vrt, self.v_ref)

        self.const_hrz = const_hrz
        self.Fset = Fset
        self.mpc_hrz = ControllerOSQPRobustVariableHorizon(
            A_hrz, B_hrz, C_hrz, D_hrz, self.Nmax, self.ds, 5)
        self.mpc_vrt = ControllerOSQPRobustVariableHorizon(
            A_vrt, B_vrt, C_vrt, D_vrt, self.Nmax, self.ds, 2)

        Y, Q = self.mpc_hrz.reachability_matrices(W, Fset, Yset, 25)

        LQRgain, _, _ = get_lqr_feedback(A_hrz, B_hrz, Q_hrz, R_hrz, q_hrz)

        #get_invariant_set(A_hrz, B_hrz, C_hrz, D_hrz, LQRgain, Yset)
        self.LQRgain = LQRgain
        # Qf = Q2

        # COPY COST MATRICES
        for i in range(nx):
            for j in range(nx):
                controller_mpc.update_Q(i, j, float(Q_hrz[i, j]))
                controller_mpc.update_Qf(i, j, float(Qf[i, j]))
                controller_mpc.update_q(i, float(q_hrz[i, 0]))
                controller_mpc.update_qf(i, float(qf[i, 0]))

        for i in range(nu):
            for j in range(nu):
                controller_mpc.update_R(i, j, float(R_hrz[i, j]))
                controller_mpc.update_r(i, float(r_hrz[i, 0]))

        controller_mpc.print_cost()

        for i in range(self.Nmax):
            for j in range(self.mpc_hrz.ny):
                if i < len(Y):
                    controller_mpc.update_ymin(i*ny + j, Y[i].lb[j, 0])
                    controller_mpc.update_ymax(i*ny + j, Y[i].ub[j, 0])
                else:
                    controller_mpc.update_ymin(i*ny + j, Y[-1].lb[j, 0])
                    controller_mpc.update_ymax(i*ny + j, Y[-1].ub[j, 0])

        for i in range(self.Nmax):
            for j in range(len(Q[0].lb)):
                if i < len(Q):
                    controller_mpc.update_Xf_min(
                        i*len(Q[0].lb) + j, Q[i].lb[j, 0])
                    controller_mpc.update_Xf_max(
                        i*len(Q[0].lb) + j, Q[i].ub[j, 0])
                else:
                    controller_mpc.update_Xf_min(
                        i*len(Q[-1].lb) + j, Q[-1].lb[j, 0])
                    controller_mpc.update_Xf_max(
                        i*len(Q[-1].lb) + j, Q[-1].ub[j, 0])

        controller_mpc.print_cost()

        controller_mpc.final_setup()

        #self.mpc_hrz.set_cost_matrix(Q2, R2, Qf, q=q2, r=r2, qf=qf)
        #self.mpc_hrz.add_cross_terms(1, 1)
        # self.mpc_hrz.set_inequality_constraints(Y)
        # self.mpc_hrz.set_terminal_constraint(Q)
        # self.mpc_hrz.setup_problems()

        Y, Q = self.mpc_vrt.reachability_matrices(
            W_vrt, Fset_vrt, Yset_vrt, 20)

        # Qf = get_LQR_infinite_cost(A_vrt, B_vrt, Q_vrt, R_vrt)
        Qf = 5*Q_vrt
        self.mpc_vrt.set_cost_matrix(Q_vrt, R_vrt, Qf)
        self.mpc_vrt.set_inequality_constraints(Y)
        self.mpc_vrt.set_terminal_constraint(Q)
        self.mpc_vrt.setup_problems()

        self.state_filter = StateFilter()

        self.init_time = time.time()

    def start_control_thread(self):
        """ Start thread for sending current control commands"""
        # uav_ctrl.start()
        # ugv_ctrl.start()

        self.control = True
        self.state_filter.start_filter()

        self.next_call = time.time()

        self.control_thread()

    def stop_control_thread(self):
        """ Stop thread for sending current control commands"""
        self.control = False

    def control_thread(self):
        """ Thread for continousy sending control commands to JSB. """
        if not self.control:
            return

        self.next_call = max(self.next_call + 0.0099, time.time())
        self.update_state()
        if self.ap_mode != 'HOLD':
            if self.count == 10:
                time_now = time.time()
                print("time = ", time_now - self.time_last)
                self.compute_control()
                self.count = 1
                self.time_last = time_now
            else:
                self.count += 1

            if self.uav_state[2] < 1.6:
                print("SUCCESSFUL LANDING")
                uav_.pause()
                ugv_.pause()
                self.stop_control_thread()

        uav_input = UAVinput()
        ugv_input = UGVinput()

        uav_input.a = uav_ctrl.setpoint.a_ref / \
            (np.cos(self.uav_state[5, 0])*np.cos(self.uav_state[6, 0]))
        uav_input.psi = uav_ctrl.setpoint.psi_ref
        uav_input.yawrate = uav_ctrl.setpoint.yawrate_ref
        uav_input.gamma = uav_ctrl.setpoint.gamma_ref
        ugv_input.a = ugv_ctrl.setpoint.a_ref/(np.cos(self.uav_state[4, 0]))
        ugv_input.psi = ugv_ctrl.setpoint.psi_ref

        self.state_filter.update_uav_input(uav_input)
        self.state_filter.update_ugv_input(ugv_input)

        self.send_command('both')

        threading.Timer(self.next_call - time.time(),
                        self.control_thread).start()

    def compute_control(self):
        """ Calculates the desired inputs when system is
            in landing mode """
        # Current state
        state = np.array([[self.uav_state[i, 0] for i in [0, 1, 3, 4, 6, 7]]
                          + [self.ugv_state[i, 0] for i in [0, 1, 2, 3, 4]]]).T

        # Current input
        # input_ = [uav_ctrl.setpoint.a_ref, uav_ctrl.setpoint.yawrate_ref,
        #          ugv_ctrl.setpoint.a_ref, ugv_ctrl.setpoint.psi_ref]

        input_ = self.last_input[0: 4, :]

        if self.final_stage:
            self.compute_mpc(state, input_)
        else:
            self.compute_pid(state, input_)

    def compute_pid(self, state, input_t):
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

        if abs(deltax) < 15:
            print("INITIAL SOLVE: ")
            print(state)
            uav_.landing_mode_drone()
            ugv_.landing_mode()
            self.final_stage = True

            #status, path, N, c = self.mpc_hrz.initial_solve(state, input_t)
            #x1 = path[np.arange(4, N*self.nvar, self.nvar)]
            #x2 = path[np.arange(10, N*self.nvar, self.nvar)]
            #y1 = path[np.arange(5, N*self.nvar, self.nvar)]
            #y2 = path[np.arange(11, N*self.nvar, self.nvar)]
            x0 = mpc_x0()
            x0.x1 = state[0, 0]
            x0.y1 = state[1, 0]
            x0.v1 = state[2, 0]
            x0.a1 = state[3, 0]
            x0.psi1 = state[4, 0]
            x0.phi1 = state[5, 0]
            x0.x2 = state[6, 0]
            x0.y2 = state[7, 0]
            x0.v2 = state[8, 0]
            x0.a2 = state[9, 0]
            x0.psi2 = state[10, 0]

            u0 = mpc_u0()
            u0.u0 = input_t[0, 0]
            u0.u1 = input_t[1, 0]
            u0.u2 = input_t[2, 0]
            u0.u3 = input_t[3, 0]
            controller_mpc.set_initial(x0, u0)
            status = controller_mpc.solve()
            if (status):
                # print(type(result.path))
                print("Failed to solve INITIAL MPC")
            else:
                result = controller_mpc.get_solution()
                print("INPUT = ", result.u0, result.u1, result.u2, result.u3)
                N = result.N
                path = result.path
                x1 = path[np.arange(4, N*self.nvar, self.nvar)]
                x2 = path[np.arange(10, N*self.nvar, self.nvar)]
                y1 = path[np.arange(5, N*self.nvar, self.nvar)]
                y2 = path[np.arange(11, N*self.nvar, self.nvar)]

            dist = np.array([math.sqrt((x1[i]-x2[i])**2 + (y1[i]-y2[i])**2)
                             for i in range(len(x1))])

            scale = np.ones((self.mpc_vrt.ni, 1))
            scale[np.arange(1, N*self.mpc_vrt.ny,
                            self.mpc_vrt.ny), 0] = np.maximum(
                                np.minimum((self.h_s * dist -
                                            self.h_s*self.d_l)/(self.d_s-self.d_l), self.h_s),
                                -0.5)

            scale[np.arange(N*self.mpc_vrt.ny+1, self.mpc_vrt.ni, self.mpc_vrt.ny),
                  0] = np.ones(((self.Nmax-N)))*-0.5

            lower_bound = self.mpc_vrt.ineq_l.copy()
            lower_bound[self.mpc_vrt.ne: self.mpc_vrt.ni +
                        self.mpc_vrt.ne] *= scale

            status, path_vrt, N2, c2 = self.mpc_vrt.initial_solve(
                np.array([[self.uav_state[2, 0] - 1.3, self.uav_state[5, 0]]]).T,
                np.array([[uav_ctrl.setpoint.gamma_ref]]).T,
                lb=lower_bound)

            #self.mpc_hrz.initial_solve(state, input_t)

    def compute_mpc(self, state_t, input_t):
        """ Computes the optimal path for the final stage of the landing
            The method is MPC with two spearate controllers.
            Inputs: state_t The state at time t
                    input_t The input at time t """
        # try:
        # Save the past 4 UAV heading inputs
        start_time = time.time()

        print(np.matmul(self.Fset.A, state_t) - self.Fset.b <= 0.001)
        if ((np.matmul(self.Fset.A, state_t) - self.Fset.b) <= 0.001).all():
            print("*********INSIDE TERMINAL SET**************")
            print(state_t.T - [0, 0, 20, 0, 0, 0, 0, 0, 20, 0, 0])

            u = np.matmul(self.LQRgain, (state_t.T -
                                         [0, 0, 20, 0, 0, 0, 0, 0, 20, 0, 0]).T)
            print("GAIN")
            print(self.LQRgain)
            print(u)
            u0 = u[0, 0]
            u1 = u[1, 0]
            u2 = u[2, 0]
            u3 = u[3, 0]
            print(u0)
            print(u1)
            print(u2)

            scale = np.ones((self.mpc_vrt.ni, 1))

            scale[np.arange(1, self.mpc_vrt.ni, self.mpc_vrt.ny),
                  0] = np.ones(((self.Nmax)))*-0.01
            N = 0
            c = 0

        else:
            print("STATE = ")
            print(state_t.T)

            x0 = mpc_x0()
            x0.x1 = state_t[0, 0]
            x0.y1 = state_t[1, 0]
            x0.v1 = state_t[2, 0]
            x0.a1 = state_t[3, 0]
            x0.psi1 = state_t[4, 0]
            x0.phi1 = state_t[5, 0]
            x0.x2 = state_t[6, 0]
            x0.y2 = state_t[7, 0]
            x0.v2 = state_t[8, 0]
            x0.a2 = state_t[9, 0]
            x0.psi2 = state_t[10, 0]

            u0 = mpc_u0()
            u0.u0 = input_t[0, 0]
            u0.u1 = input_t[1, 0]
            u0.u2 = input_t[2, 0]
            u0.u3 = input_t[3, 0]

            controller_mpc.set_initial(x0, u0)
            status = controller_mpc.solve()
            if (status):
                # print(type(result.path))
                print("Failed to solve MPC")
            else:
                result = controller_mpc.get_solution()
                print("INPUT = ", result.u0, result.u1, result.u2, result.u3)
                N = result.N
                c = result.cost + self.const_hrz

                path = result.path
                #status, path, N, c = self.mpc_hrz.solve(state_t, input_t)
                # N = self.Nmax
                # print("STATE = ", state_t)
                # print("STATE DIFF = ", state_t-self.predicted_state)
                x1 = path[np.arange(4, N*self.nvar, self.nvar)]
                x2 = path[np.arange(10, N*self.nvar, self.nvar)]
                y1 = path[np.arange(5, N*self.nvar, self.nvar)]
                y2 = path[np.arange(11, N*self.nvar, self.nvar)]
                #u0 = path[self.nvar + 0]
                #u1 = path[self.nvar + 1]
                #u2 = path[self.nvar + 2]
                #u3 = path[self.nvar + 3]

                u0 = result.u0
                u1 = result.u1
                u2 = result.u2
                u3 = result.u3
                self.predicted_state = path[range(4, 15)]

                if u0 is None:
                    print("* ERROR HORIZONTAL", u0)
                    print("* SOLVE STATUS = ", status)
                    print("* x0 = ", state_t)
                    print("* u0 = ", input_t.T)
                    print("* x1 = ", (np.matmul(self.mpc_hrz.A,
                                                [state_t]) + np.matmul(self.mpc_hrz.B, input_t)).T)
                    u0_vrt = 0.0

                #print("NEW VS OLD: ", u0, input_t[0])
                # print("PRED = ", self.predicted_state)

                dist = np.array(
                    [math.sqrt((x1[i]-x2[i])**2 + (y1[i]-y2[i])**2) for i in range(len(x1))])

                scale = np.ones((self.mpc_vrt.ni, 1))
                scale[np.arange(1, N*self.mpc_vrt.ny, self.mpc_vrt.ny), 0] = np.maximum(
                    np.minimum((self.h_s * dist -
                                self.h_s*self.d_l)/(self.d_s-self.d_l), self.h_s),
                    -0.01)

                scale[np.arange(N*self.mpc_vrt.ny+1, self.mpc_vrt.ni, self.mpc_vrt.ny),
                      0] = np.ones(((self.Nmax-N)))*-0.01

        # ------------------------------------------------------------------------------------------
        # COMPUTE VERTICAL  SOLUTION
        # ------------------------------------------------------------------------------------------
        lower_bound = self.mpc_vrt.ineq_l.copy()
        lower_bound[self.mpc_vrt.ne: self.mpc_vrt.ni +
                    self.mpc_vrt.ne] *= scale

        # print("SOLVER = %i, N = %i" % (status, N))
        # print("1 deltax = ", x1[0:20] - x2[0:20])

        uav_ctrl.setpoint.a_ref = u0 / \
            (np.cos(self.uav_state[5, 0]) *
             np.cos(self.uav_state[6, 0]))  # [m/s2]
        ugv_ctrl.setpoint.a_ref = u2 /\
            (np.cos(self.ugv_state[4, 0]))  # [m/s2]
        # uav_ctrl.setpoint.psi_ref = u1  # [rad/s]
        uav_ctrl.setpoint.yawrate_ref = u1  # [rad/s]
        ugv_ctrl.setpoint.psi_ref = u3  # [rad/s]

        status, path_vrt, N2, c2 = self.mpc_vrt.solve(
            np.array([[self.uav_state[2, 0] - 1.3, self.uav_state[5, 0]]]).T,
            np.array([[self.last_input[-1]]]).T,
            time_limit=0.01, lb=lower_bound)
        u0_vrt = path_vrt[3]
        if u0_vrt is None:
            print("* VERTICAL ERROR", u0_vrt)
            print("  x0 = ", np.array(
                [[self.uav_state[2, 0] - 1.3, self.uav_state[5, 0]]]))
            print("  deltax", x1-x2)
            print("DISTANCE", dist)
            print("LB", lower_bound.T)
            u0_vrt = 0.0

        # h0 = path_vrt[np.arange(1, 3*N2, 3)]
        # g0 = path_vrt[np.arange(2, 3*N2, 3)]
        uav_ctrl.setpoint.gamma_ref = u0_vrt

        # except:
        #    print("Could not solve vertical...: ", status)
        #    print("state = ", np.array(
        #        [[self.uav_state[2, 0] - 1.3, self.uav_state[5, 0]]]))
        #    u0_vrt = 0.0
        # print(distance)
        # print("SCALING = ", scaling[1:3:])

        self.last_input = np.array([[u0, u1, u2, u3,
                                     u0_vrt]]).T
        self.solve_time.append(time.time()-start_time)
        self.solve_horizon.append(N)
        self.solve_horizon2.append(N2)

        self.solve_cost.append(c)
        self.solve_cost2.append(c2)

        if self.xpath is None:
            print("*****************************")
            self.xpath = [self.time[self.itr % self.N_data], N, path]

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

        # print("UAV YAW/ROLL = %f   %f" %
        #      (uav_state.psi*180/np.pi, uav_state.phi*180/np.pi))

        # hrz_state = np.array([[uav_state.x, uav_state.y, uav_state.v,
        #                       uav_state.a, uav_state.psi -
        #                       math.radians(HEADING), uav_state.phi,
        #                       ugv_state.x, ugv_state.y, ugv_state.v,
        #                       ugv_state.a, ugv_state.psi-math.radians(HEADING)]]).T

        # vrt_state = np.array([[uav_state.h, uav_state.gamma]]).T

        # if self.itr == 0:
        #    self.kalman_hrz.x = hrz_state
        #    self.kalman_vrt.x = vrt_state

        new_uav = not (uav_state.t == self.uav_time)
        new_ugv = not (ugv_state.t == self.ugv_time)
        self.uav_time = uav_state.t
        self.ugv_time = ugv_state.t

        # Predict state
        # self.kalman_hrz.predict(self.last_input[0:4, :], 1/self.update_rate)
        # self.kalman_vrt.predict(
        #    self.last_input[4, None, :], 1/self.update_rate)

        # Correct
        if new_uav:
            # Aerial vehicle
            deltat = time_now - uav_state.t
            uav_measurement = UAVstate()

            uav_measurement.x = uav_state.x
            uav_measurement.y = uav_state.y
            uav_measurement.h = uav_state.h
            uav_measurement.vx = uav_state.vx
            uav_measurement.vy = uav_state.vy
            uav_measurement.vz = uav_state.vz
            uav_measurement.ax = uav_state.ax
            uav_measurement.ay = uav_state.ay
            uav_measurement.az = uav_state.az
            uav_measurement.phi = uav_state.phi
            uav_measurement.gamma = uav_state.gamma
            uav_measurement.psi = uav_state.psi

            uav_measurement.pdot = uav_state.p
            uav_measurement.qdot = uav_state.q
            uav_measurement.rdot = uav_state.r

            self.state_filter.update_uav(uav_measurement)

            # hrz_state[0:6, 0] = self.kalman_hrz.prop_uav(hrz_state[0:6, :],
            #                                             self.last_input[0:2, :], deltat)
            # self.kalman_hrz.update_uav(hrz_state[0:6, :])
            # self.kalman_vrt.update_alt(vrt_state)

        if new_ugv:
            deltat = time_now - ugv_state.t  #
            ugv_measurement = UGVstate()

            ugv_measurement.x = ugv_state.x
            ugv_measurement.y = ugv_state.y
            ugv_measurement.vx = ugv_state.vx
            ugv_measurement.vy = ugv_state.vy
            ugv_measurement.ax = ugv_state.ax
            ugv_measurement.ay = ugv_state.ay
            ugv_measurement.psi = ugv_state.psi
            ugv_measurement.r = ugv_state.r

            self.state_filter.update_ugv(ugv_measurement)

            # Ground vehicle
            # hrz_state[6:11, 0] = self.kalman_hrz.prop_ugv(hrz_state[6:11, :],
            #                                              self.last_input[2:4, :], deltat)
            # self.kalman_hrz.update_ugv(hrz_state[6:11, :])
        uav_new_state = UAVstate()
        uav_new_state = self.state_filter.get_uav()

        self.uav_state[:, 0] = [uav_new_state.x, uav_new_state.y, uav_new_state.h,
                                uav_new_state.vx, uav_new_state.ax, uav_new_state.gamma,
                                uav_new_state.psi, uav_new_state.phi]

        # print("MEASURED = [%f  %f  %f] => %f" % (
        #    uav_state.ax, uav_state.ay, uav_state.az, uav_new_state.ax))
        # print("NEW UAV YAW/ROLL = %f   %f" %
        #      (uav_new_state.psi*180/np.pi, uav_new_state.phi*180/np.pi))

        ugv_new_state = UGVstate()
        ugv_new_state = self.state_filter.get_ugv()
        self.ugv_state[:, 0] = [ugv_new_state.x, ugv_new_state.y,
                                ugv_new_state.vx, ugv_state.ax, ugv_new_state.psi]

        # self.uav_state[[2, 5], 0] = self.kalman_vrt.x[:, 0]
        # self.uav_state[[0, 1, 3, 4, 6, 7], 0] = self.kalman_hrz.x[0: 6, 0]
        # self.ugv_state[0: 5, 0] = self.kalman_hrz.x[6:, 0]

        # print("STATES = ")
        # print(self.uav_state.T)
        # print(self.ugv_state.T)
        self.uav_data[self.itr % self.N_data, :] = [*self.uav_state[0:8, 0],
                                                    *self.last_input[[0, 1, 4], :],
                                                    uav_state.phi, uav_state.gamma, uav_state.psi,
                                                    uav_state.ax, uav_state.ay, uav_state.az]
        self.ugv_data[self.itr % self.N_data, :] = [*self.ugv_state[0:5, 0],
                                                    *self.last_input[[2, 3], :]]

        self.itr += 1

        self.time[self.itr % self.N_data] = time_now


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
                elif str(slider.objectName()).lower() == "yawrate":
                    slider.setValue(uav_ctrl.setpoint.yawrate_ref)

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
np.set_printoptions(precision=3, linewidth=200, threshold=2000,
                    edgeitems=15, suppress=True)


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
    try:
        N = sim.xpath[1]
        path = sim.xpath[2]
        x1 = path[np.arange(4, N*sim.nvar, sim.nvar)]
        x2 = path[np.arange(10, N*sim.nvar, sim.nvar)]
        y1 = path[np.arange(5, N*sim.nvar, sim.nvar)]
        y2 = path[np.arange(11, N*sim.nvar, sim.nvar)]
        v1 = path[np.arange(6, N*sim.nvar, sim.nvar)]
        v2 = path[np.arange(12, N*sim.nvar, sim.nvar)]
        a1 = path[np.arange(7, N*sim.nvar, sim.nvar)]
        a2 = path[np.arange(13, N*sim.nvar, sim.nvar)]
        psi1 = path[np.arange(8, N*sim.nvar, sim.nvar)]
        phi1 = path[np.arange(9, N*sim.nvar, sim.nvar)]
        psi2 = path[np.arange(14, N*sim.nvar, sim.nvar)]

        u0 = path[np.arange(0, N*sim.nvar, sim.nvar)]
        u1 = path[np.arange(1, N*sim.nvar, sim.nvar)]
        u2 = path[np.arange(2, N*sim.nvar, sim.nvar)]
        u3 = path[np.arange(3, N*sim.nvar, sim.nvar)]

        timer = np.linspace(sim.xpath[0], sim.xpath[0]+sim.ds*N, N)
    except:
        N = None
        x1 = np.zeros((1, 1))
        x2 = np.zeros((1, 1))
        y1 = np.zeros((1, 1))
        y2 = np.zeros((1, 1))
        v1 = np.zeros((1, 1))
        v2 = np.zeros((1, 1))
        a1 = np.zeros((1, 1))
        a2 = np.zeros((1, 1))
        psi1 = np.zeros((1, 1))
        psi2 = np.zeros((1, 1))
        phi1 = np.zeros((1, 1))
        u0 = np.zeros((1, 1))
        u1 = np.zeros((1, 1))
        u2 = np.zeros((1, 1))
        u3 = np.zeros((1, 1))
        timer = np.zeros((1, 1))

    print(len(u0))
    print(len(x1))
    print(len(timer))

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

    plt.plot(timer-t0, x1-x2, linestyle='dashed',
             linewidth=1.5, color='red')
    plt.plot(timer-t0, y1-y2, linestyle='dashed',
             linewidth=1.5, color='purple')

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

    plt.plot(timer-t0, v1, linestyle='dashed',
             linewidth=1.5, color='red')
    plt.plot(timer-t0, v2, linestyle='dashed',
             linewidth=1.5, color='purple')

    plt.legend()

    plt.subplot(3, 2, 4)
    plt.title("Acceleration")
    plt.ylabel("Acceleration")
    plt.xlabel("Time [s]")
    plt.plot(sim.time-t0, sim.uav_data[:, 4], label='UAV')
    plt.plot(sim.time-t0, sim.ugv_data[:, 3], label='UGV')

    plt.plot(sim.time-t0, sim.uav_data[:, 14],
             label='Raw', linestyle='dashed', color='green')
    plt.plot(sim.time-t0, sim.uav_data[:, 15],
             label='Raw', linestyle='dashed', color='c')
    plt.plot(sim.time-t0, sim.uav_data[:, 16],
             label='Raw', linestyle='dashed', color='yellow')

    # plt.plot(timer-t0, a1, linestyle='dashed',
    #         linewidth=1.5, color='red')
    # plt.plot(timer-t0, a2, linestyle='dashed',
    #         linewidth=1.5, color='purple')

    plt.legend()

    plt.subplot(3, 2, 5)
    plt.xlabel("Time [s]")
    plt.ylabel("Degrees")
    plt.title("Heading")
    plt.plot(sim.time-t0, np.degrees(sim.uav_data[:, 6]), label='UAV')
    # plt.plot(sim.time-t0, np.degrees(sim.uav_data[:, 7]), label='Roll')
    plt.plot(sim.time-t0, np.degrees(sim.ugv_data[:, 4]), label='UGV')

    plt.plot(timer-t0, np.degrees(psi1), linestyle='dashed',
             linewidth=1.5, color='red')
    plt.plot(timer-t0, np.degrees(psi2), linestyle='dashed',
             linewidth=1.5, color='purple')

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

    plt.plot(timer-t0, u0, linestyle='dashed',
             linewidth=1.5, color='red')
    plt.plot(timer-t0, u2, linestyle='dashed',
             linewidth=1.5, color='purple')

    plt.ylim([-5, 5])
    plt.legend()
    plt.subplot(1, 3, 2)
    plt.xlabel("Time [s]")
    plt.ylabel("Angles [deg]")
    plt.title("Heading inputs")
    plt.step(sim.time-t0, np.degrees(sim.uav_data[:, 9]), label='UAV')
    plt.step(sim.time-t0, np.degrees(sim.ugv_data[:, 6]), label='UGV')

    plt.plot(timer-t0, np.degrees(u1), linestyle='dashed',
             linewidth=1.5, color='red')
    plt.plot(timer-t0, np.degrees(u3), linestyle='dashed',
             linewidth=1.5, color='purple')

    plt.ylim([-45, 45])
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
    ax2.scatter(np.arange(0, len(sim.solve_horizon), dtype='float'),
                sim.solve_horizon, color='green')
    ax2.plot(sim.solve_horizon2, color='orange')
    ax2.scatter(np.arange(0, len(sim.solve_horizon), dtype='float'),
                sim.solve_horizon2, color='orange')

    distance = np.sqrt(np.power(sim.uav_data[:, 0]-sim.ugv_data[:, 0], 2) + np.power(
        sim.uav_data[:, 1]-sim.ugv_data[:, 1], 2))
    ax2.plot(distance[np.arange(10, len(distance), 10)], color='m')

    ax2.set_ylabel('Horizon', color='green')
    ax2.tick_params(axis='y', labelcolor='green')
    plt.title("MPC solve time and horizon")
    plt.tight_layout()  # otherwise the right y-label is slightly clipped

    plt.figure("MPC COST", (16, 10))
    plt.plot(sim.solve_cost, color='green', label='Horizontal')
    plt.scatter(np.arange(0, len(sim.solve_cost), dtype='float'),
                sim.solve_cost, color='green')
    plt.plot(sim.solve_cost2, color='orange', label='Vertical')
    plt.scatter(np.arange(0, len(sim.solve_cost2), dtype='float'),
                sim.solve_cost2, color='orange')
    plt.legend()
    plt.title("MPC cost")
    plt.tight_layout()  # otherwise the right y-label is slightly clipped

    plt.figure("HEADING DELAY", (16, 10))
    plt.subplot(2, 1, 1)
    plt.title("X")
    plt.plot(sim.time-t0, np.degrees(sim.uav_data[:, 6]), label='UAV Yaw')
    plt.plot(sim.time-t0, np.degrees(sim.uav_data[:, 7]), label='UAV Roll')
    plt.step(
        sim.time-t0, np.degrees(sim.uav_data[:, 9]), label='UAV input', color='k')
    plt.plot(timer-t0, np.degrees(phi1), linestyle='dashed',
             linewidth=1.5, color='red')
    plt.plot(
        sim.time-t0, np.degrees(sim.uav_data[:, 13]), label='Raw', linestyle='dashed', color='green')
    plt.plot(
        sim.time-t0, np.degrees(sim.uav_data[:, 11]), label='Raw', linestyle='dashed', color='yellow')

    plt.legend()
    plt.ylabel("Degrees")
    plt.xlabel("Time [s]")

    plt.subplot(2, 1, 2)
    plt.plot(sim.time-t0, np.degrees(sim.ugv_data[:, 4]), label='UGV Yaw')
    plt.step(
        sim.time-t0, np.degrees(sim.ugv_data[:, 6]), label='UGV input', color='k')
    plt.plot(timer-t0, np.degrees(psi2), linestyle='dashed',
             linewidth=1.5, color='red')

    plt.title("Y")
    plt.xlabel("Time [s]")

    plt.legend()
    plt.tight_layout()

    plt.figure("GAMMA DELAY", (16, 10))
    plt.title("Gamma")
    plt.plot(sim.time-t0, np.degrees(sim.uav_data[:, 5]), label='UAV Gamma')
    plt.step(
        sim.time-t0, np.degrees(sim.uav_data[:, 10]), label='UAV input', color='k')
    # plt.plot(timer-t0, np.degrees(phi1), linestyle='dashed',
    #         linewidth=1.5, color='red')
    plt.plot(
        sim.time-t0, np.degrees(sim.uav_data[:, 12]), label='Raw', linestyle='dashed')
    plt.legend()
    plt.ylabel("Degrees")
    plt.xlabel("Time [s]")
    plt.tight_layout()

    plt.show()

now = datetime.now()  # current date and time
date_time = now.strftime("%y%m%d_%H-%M-%S")
print("date and time:", date_time)

# np.savetxt("uav" + date_time + ".csv", sim.uav_data, delimiter=",")
# np.savetxt("ugv" + date_time + ".csv", sim.ugv_data, delimiter=",")
# np.savetxt("time" + date_time + ".csv", sim.time, delimiter=",")

solver = np.array([sim.solve_horizon, sim.solve_horizon2,
                   sim.solve_time, sim.solve_cost, sim.solve_cost2]).T
np.savetxt("solver" + date_time + ".csv", solver, delimiter=",")
