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
from PyQt4 import QtGui, QtCore

from simulation import SimulationGUI
from control import MPC, Positioner
import communication.fgsocket
import communication.fgtelnet
import cvxpy
import matplotlib.pyplot as plt


# REACHABLE SET
#----------------------------------------------------------
with open("control/A.csv", "rb") as f:
    readerA = csv.reader(f, delimiter=",")
    A = np.array(list(readerA)).astype("float")
with open("control/B.csv", "rb") as f:
    readerB = csv.reader(f, delimiter=",")
    b = np.array(list(readerB)).astype("float")


# MAIN PROGRAM PARAMETERS
#----------------------------------------------------------
ORIGIN = (42.186702238384, -71.00457277413)
HEADING = 199.67
POSITIONER = Positioner(ORIGIN, HEADING)
FEET2M = 0.3048

SIMULATION = False
CONTROL = False


# Telnet communication
uav_ = communication.fgtelnet.FlightGearVehicle('localhost', 6500)
ugv_ = communication.fgtelnet.FlightGearVehicle('localhost', 6600)

# TCP Communication
uav_ctrl = communication.fgsocket.UAV(5515, 5514)
ugv_ctrl = communication.fgsocket.UGV(5526, 5525)
ugv_ctrl.heading = HEADING
uav_ctrl.heading = HEADING


# -------------------- MAIN CLASS -------------------------

class MainSimulation(object):
    UAV_FILE = './logs/Rascal.csv'
    UGV_FILE = './logs/followme.csv'
    ap_mode = 'HOLD'
    uav_state = []
    ugv_state = []
    plot_it = 0
    past_inputs = [0, 0, 0, 0, 0]
    final_stage = False
    d_safe = 1
    start_final = None
    def __init__(self):
        self.ds = 0.18
        self.mpc, self.mpc_alt = self.initalize_mpc()
        self.init_time = time.time()

    def initalize_mpc(self):
        """ Initialize the two MPC with the correct values """
        mpc_uav_settings = MPC.Vehicle()
        mpc_ugv_settings = MPC.Vehicle()

        mpc_ugv_settings.v.min_ = 15
        mpc_uav_settings.a.min_ = -1.0
        mpc_uav_settings.a.max_ = 1.0
        mpc_uav_settings.a_des.min_ = -1.0
        mpc_uav_settings.a_des.max_ = 1.0

        T = 40
        mpc = MPC.Controller([mpc_uav_settings.x, mpc_uav_settings.y,
                              mpc_uav_settings.v, mpc_uav_settings.a,
                              mpc_uav_settings.chi,
                              mpc_ugv_settings.x, mpc_ugv_settings.y,
                              mpc_ugv_settings.v, mpc_ugv_settings.a,
                              mpc_ugv_settings.chi],
                             [mpc_uav_settings.a_des, mpc_uav_settings.chi_des,
                              mpc_ugv_settings.a_des, mpc_ugv_settings.chi_des],
                             MPC.A_lat, MPC.B_lat, T, self.ds)

        mpc_alt = MPC.Controller([mpc_uav_settings.h, mpc_uav_settings.gamma],
                                 [mpc_uav_settings.gamma_des],
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


        threading.Timer(self.next_call - time.time(), self.control_thread).start()

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
        self.past_inputs[-1] = [0, math.radians(uav_ctrl.setpoint['heading']), 0, 0]

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
                  -self.ugv_state[2]*math.cos(self.ugv_state[4]))
        deltav = (self.uav_state[3]*math.sin(self.uav_state[6])
                  -self.ugv_state[2]*math.sin(self.ugv_state[4]))
        # VELOCITY
        v_uav = 20 - 0.1*deltax - 0.1*deltau # m/s
        v_ugv = 20 + 0.05*deltax +  0.1*deltau # m/s
        uav_ctrl.setpoint['velocity'] = max(min(28, v_uav), 18)     # [m/s]
        ugv_ctrl.setpoint['velocity'] = max(min(30, v_ugv), 0)     # [m/s]

        x = np.array([deltax, deltay,
                      self.uav_state[3], self.uav_state[4], self.uav_state[6],
                      self.ugv_state[2], self.ugv_state[3], self.ugv_state[4]])[np.newaxis].T
        ans = not False in np.less_equal(np.dot(A,x),b)
        if ans:
            print("START FINAL STAGE OF LANDING")
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
            inputs = self.mpc.u[:, 1].value.A.flatten()
            uav_ctrl.setpoint['acceleration'] = inputs[0]     # [m/s2]
            ugv_ctrl.setpoint['acceleration'] = inputs[2]     # [m/s2]
            uav_ctrl.setpoint['heading'] = math.degrees(inputs[1])     # [rad/s]
            ugv_ctrl.setpoint['heading'] = math.degrees(inputs[3])     # [rad/s]

            x1 = self.mpc.x[0, :].value.A.flatten() # uav x
            x2 = self.mpc.x[1, :].value.A.flatten() # uav y
            x6 = self.mpc.x[5, :].value.A.flatten() # ugv x
            x7 = self.mpc.x[6, :].value.A.flatten() # ugv y
                
            distance = [math.sqrt((x1[i]-x6[i])**2 + (x2[i]-x7[i])**2) for i in range(self.mpc.T)]
            b1 = [int(distance[i] < self.d_safe) for i in range(self.mpc.T)]
            b2 = [int(not distance[i] < self.d_safe) for i in range(self.mpc.T)]

            self.mpc_alt.b1.value = b1
            self.mpc_alt.b2.value = b2
            self.mpc_alt.distance.value = distance

            print("time: ", time.time()-start_time)
        except:
            print("ERROR OCCURED: NO SOLUTION FOUND (LAT)")
            print(state_t)
            print(input_t)
            print(self.past_inputs[:-1])

            uav_.pause()
            ugv_.pause()
            self.stop_control_thread()

        try:
            result = self.mpc_alt.solve([self.uav_state[2] -2.5,
                                         self.uav_state[5]],
                                        [math.radians(uav_ctrl.setpoint['gamma'])])
            u = self.mpc_alt.u[0, 1].value
            uav_ctrl.setpoint['gamma'] = math.degrees(u)
        except:
            print("ERROR OCCURED: NO SOLUTION FOUND (LON)")
            uav_ctrl.setpoint['gamma'] = 0.0
            uav_ctrl.toggle_pause(1)
            ugv_ctrl.toggle_pause(1)
            self.stop_control_thread()


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
            raise ValueError("Acceptable arguments are 'uav', 'ugv' or 'both'. ")

    def get_state(self):
        """ Send command to control system of vehicle. """
        # UAV = [0=x, 1=y, 2=h-agl-ft, 3=vt-fps, 4=udot-ft_sec2, 5=gamma-rad, 6=psi-gt-rad]
        uav = uav_ctrl.get_state([13, 14, 10, 1, 2, 3, 4, 0])
        # UAV = [0=x, 1=y, 2=vt-fps, 3=udot-ft_sec2, 4=psi-gt-rad, 5=h-agl-ft]
        ugv = ugv_ctrl.get_state([9, 10, 2, 3, 5, 6, 0])

        # Aerial vehicle
        uav[2] *= FEET2M  # altitude
        uav[3] *= FEET2M  # velocity
        uav[4] *= FEET2M  # acceleration
        #uav[5] *= 1  # gamma [rad]
        uav[6] += -math.radians(HEADING) # psi [rad]
        # Ground vehicle
        ugv[2] *= FEET2M # velocity
        ugv[3] *= FEET2M # acceleration
        ugv[4] += -math.radians(HEADING) # psi [rad]
        ugv[5] *= FEET2M # h

        deltat = uav[-1] - ugv[-1]    

        # Approximate current acceleration, velocity and position
        if deltat > 0: # UAV data is newer
            a = ugv[3]*(1-11.99*deltat) + 11.54*deltat*ugv_ctrl.setpoint['acceleration']
            v = ugv[2] + deltat/2*(a + ugv[3])
            x = ugv[0] + deltat/2*(v + ugv[2])
            ugv[2] = v
            ugv[0] = x
        else: # UGV data is newer
            deltat *= -1
            a = uav[4]*(1-1.93*deltat) + 1.776*deltat*uav_ctrl.setpoint['acceleration']
            v = uav[3] + deltat/2*(a + uav[4])
            x = uav[0] + deltat/2*(v + uav[3])
            uav[3] = v
            uav[0] = x 
        self.uav_state = uav
        self.ugv_state = ugv

        if uav[2] < 1.6:
            print("SUCCESSFUL LANDING")
            uav_.pause()
            ugv_.pause()
            self.stop_control_thread()

# -------------------- GUI CLASS -------------------------
class MyGui(SimulationGUI):
    """ GUI for sending commands to FlightGear. """
    ap_mode = 'HOLD'

    def __init__(self, simulation, uav_, ugv_):
        SimulationGUI.__init__(self, uav_, ugv_)
        # Initial GUI setup
        for group in self.groups.values():
            for button in group.buttons():
                if button.isChecked():
                    button.click()

        for slider in self.sliders['uav'].values():
            slider.setValue(uav_ctrl.setpoint[str(slider.objectName()).lower()])

        for slider in self.sliders['ugv'].values():
            slider.setValue(ugv_ctrl.setpoint[str(slider.objectName()).lower()])

        self.sim = simulation
        self.sim.send_command('both')
        # Start the control thread
        self.sim.start_control_thread()
        uav_ctrl.start_update_state()
        ugv_ctrl.start_update_state()

# :::::::::::::::::: BUTTON CALLBACKS :::::::::::::::::::::::::::

    def set_hold_value(self, vehicle, property_, value):
        """ Sets the hold value of some autopilot property. """
        if vehicle == 'uav':
            uav_ctrl.hold[property_] = value
        elif vehicle == 'ugv':
            ugv_ctrl.hold[property_] = value
        else:
            raise ValueError("Acceptable arguments are 'uav' or 'ugv'.")

    def set_setpoint_value(self, vehicle, property_, value):
        """ Sets the value of a setpoint. """
        if vehicle == 'uav':
            uav_ctrl.setpoint[property_] = value
        elif vehicle == 'ugv':
            ugv_ctrl.setpoint[property_] = value
        else:
            raise ValueError("Acceptable arguments are 'uav' or 'ugv'.")

    def reset(self):
        """ Reset JSBSim to start (not working) """
        print("Restart vehicle simulations")
        self.time = 0
        if self.uav_file_exists:
            uav_.reset()
            time.sleep(0.5)
            files = [file for file in os.listdir('logs/') if file.startswith('Rascal_')]
            if len(files) > 0:
                self.UAV_FILE = './logs/' + sorted(files, reverse=True)[0]
        if self.ugv_file_exists:
            ugv_.reset()
            time.sleep(0.5)
            files = [file for file in os.listdir('logs/') if file.startswith('followme_')]
            if len(files) > 0:
                self.UGV_FILE = './logs/' + sorted(files, reverse=True)[0]

    def pause(self):
        """ Reset JSBSim to start (not working) """
        print("Pause vehicle simulations")
        if self.pause_sim:
            uav_.resume()
            ugv_.resume()
            self.pause_sim = False
            self.pause_btn.setText("Pause")

        else:
            uav_.pause()
            ugv_.pause()
            self.pause_sim = True
            self.sim.stop_control_thread()
            self.pause_btn.setText("Resume")

    def stop_control(self):
        self.sim.stop_control_thread()



app = QtGui.QApplication(sys.argv)
simulation = MainSimulation()
gui = MyGui(simulation, uav_, ugv_)
gui.show()
app.exec_()



print("Simulation has terminated")
