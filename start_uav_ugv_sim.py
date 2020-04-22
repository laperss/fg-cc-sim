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
from fgpython import SimulationGUI, FGSocketConnection, Vehicle
#from control import MPC, Positioner
from control import Positioner, ControllerPID
# Import the configuration file
import config as cfg


# Update initial setpoints
cfg.uav.control.update_setpoint('altitude', 25.0)
cfg.uav.control.update_setpoint('velocity', 20.0)
cfg.ugv.control.update_setpoint('velocity', 23.0)


# -------------------- MAIN CLASS -------------------------

class MainSimulation(object):
    ap_mode = 'HOLD'  # Initial autopilot mode
    uav_state = []
    ugv_state = []
    final_stage = False
    v_ref = cfg.v_ref

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

        if cfg.uav.control.connected and cfg.ugv.control.connected:
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

        v_uav, v_ugv, heading_uav, heading_ugv = self.PID.get_control(
            deltax, deltay, deltav)

        cfg.uav.control.update_setpoint(
            'velocity', max(min(28.0, v_uav), 18.0))
        cfg.ugv.control.update_setpoint(
            'velocity', max(min(30.0, v_ugv), 0))

        cfg.uav.control.update_setpoint(
            'heading', max(min(5, heading_uav), -5))
        cfg.ugv.control.update_setpoint(
            'heading', max(min(5, heading_ugv), -5))

        # Switch to final stage of landing after certain point.
        # Change condition inside parenthesis.
        # Possibly switch mode after this point.
        if (abs(deltax) < 10):
            # cfg.uav.command.landing_mode()
            # cfg.uav.command.landing_mode()
            self.final_stage = True

    def send_command(self, vehicle):
        """ Send command to control system of vehicle. """
        if vehicle == 'both':
            cfg.uav.control.send_cmd()
            cfg.ugv.control.send_cmd()
        elif vehicle == 'uav':
            cfg.uav.control.send_cmd()
        elif vehicle == 'ugv':
            cfg.ugv.control.send_cmd()
        else:
            raise ValueError(
                "Acceptable arguments are 'uav', 'ugv' or 'both'. ")

    def get_state(self):
        """ Send command to control system of vehicle. """
        # UAV = [0=x, 1=y, 2=h-agl-ft, 3=vt-fps, 4=udot-ft_sec2, 5=gamma-rad, 6=psi-gt-rad]
        uav = cfg.uav.control.get_state([1, 2, 3, 4, 5, 6, 7, 8, 0])
        # UAV = [0=x, 1=y, 2=vt-fps, 3=udot-ft_sec2, 4=psi-gt-rad, 5=h-agl-ft]
        ugv = cfg.ugv.control.get_state([1, 2, 4, 5, 6, 3, 0])

        # Aerial vehicle
        uav[0] -= 0.5
        uav[2] *= cfg.FEET2M  # altitude
        uav[3] *= cfg.FEET2M  # velocity
        uav[4] *= cfg.FEET2M  # acceleration
        # uav[5] *= 1  # gamma [rad]
        uav[6] += -math.radians(cfg.heading)  # psi [rad]
        # Ground vehicle
        ugv[2] *= cfg.FEET2M  # velocity
        ugv[3] *= cfg.FEET2M  # acceleration
        ugv[4] += -math.radians(cfg.heading)  # psi [rad]
        ugv[5] *= cfg.FEET2M  # h

        deltat = uav[-1] - ugv[-1]

        # Approximate current acceleration, velocity and position
        if deltat > 0:  # UAV data is newer
            a = ugv[3]*(1-11.99*deltat) + 11.54*deltat * \
                cfg.ugv.control.get_setpoint('acceleration')
            v = ugv[2] + deltat/2*(a + ugv[3])
            x = ugv[0] + deltat/2*(v + ugv[2])
            ugv[2] = v
            ugv[0] = x
        else:  # UGV data is newer
            deltat *= -1
            a = uav[4]*(1-1.93*deltat) + 1.776*deltat * \
                cfg.uav.control.get_setpoint('acceleration')
            v = uav[3] + deltat/2*(a + uav[4])
            x = uav[0] + deltat/2*(v + uav[3])
            uav[3] = v
            uav[0] = x

        self.uav_state = uav
        self.ugv_state = ugv

        if uav[2] < 1.6:
            print("SUCCESSFUL LANDING")
            cfg.uav.command.pause()
            cfg.ugv.command.pause()
            self.stop_control_thread()

# -------------------- GUI CLASS -------------------------


class MyGui(SimulationGUI):
    """ GUI for sending commands to FlightGear. """
    ap_mode = 'HOLD'

    def __init__(self, simulation, vehicles):
        self.sim = simulation
        SimulationGUI.__init__(self, vehicles)

        # Set sliders to initial values
        for vehicle in self.vehicles:
            for slider in self.sliders.values():
                slider.setValue(
                    cfg.uav.control.get_setpoint(str(slider.objectName()).lower()))

        self.sim.send_command('both')

        # Start the control thread
        self.sim.start_control_thread()
        cfg.uav.control.start_receive_state()
        cfg.ugv.control.start_receive_state()

    def stop_control(self):
        self.sim.stop_control_thread()


# :::::::::::::::::: UPDATE DATA :::::::::::::::::::::::::::
app = QtGui.QApplication(sys.argv)
simulation = MainSimulation()
gui = MyGui(simulation, [cfg.uav, cfg.ugv])
gui.show()
app.exec_()


print("Simulation has terminated")
