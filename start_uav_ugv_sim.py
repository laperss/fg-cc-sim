#! /usr/bin/python
"""
The main class of the cooperative control simulation.
The script starts the GUI from which commands can be sent to FlightGear,
and starts the control thread.
The control thread here computes a rendezvous trajectory dor the UAV and UGV using a simple PID controller.
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
#from control import MPC, Positioner
from control import Positioner, ControllerPID

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
    final_stage = False
    v_ref = 20.0

    def __init__(self):
        self.ds = 0.18
        self.init_time = time.time()
        self.PID = ControllerPID(self.v_ref, 0.1, 0.1, 0.05, 0.1)

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
        input_ = [uav_ctrl.setpoint['acceleration'],
                  math.radians(uav_ctrl.setpoint['heading']),
                  ugv_ctrl.setpoint['acceleration'],
                  math.radians(ugv_ctrl.setpoint['heading'])]

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

        v_uav, v_ugv = self.PID.get_control(deltax, deltay, deltav)

        uav_ctrl.setpoint['velocity'] = max(min(28.0, v_uav), 18.0)     # [m/s]
        ugv_ctrl.setpoint['velocity'] = max(min(30.0, v_ugv), 0) # [m/s]


        # Switch to landing mode after certain point
        # Change condition inside parenthesis. 
        if (abs(deltax) < 10):
            uav_.landing_mode()
            ugv_.landing_mode()
            self.final_stage = True


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
