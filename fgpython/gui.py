#! /usr/bin/python
""" Gui for controlling FlightGear. Commands are sent via TCP or Telnet to FlightGear.
Author: Linnea Persson, laperss@kth.se 
"""
from __future__ import print_function
import subprocess
import time
import signal
import os
from PyQt4 import QtCore, QtGui
import pyqtgraph.exporters
import pyqtgraph as pg
import pandas as pd


TITLE = 'FlightGear commands'
UPDATE_FREQ = 10

PATH = os.path.dirname(os.path.abspath(__file__))
UAV_SCRIPT = os.path.join(PATH, "../flightgear/uav_run.sh")
UGV_SCRIPT = os.path.join(PATH, "../flightgear/ugv_run.sh")

# START THE FLIGHTGEAR SCRIPTS
#---------------------------------------------------------------------------------
def uav_script():
    """ Run the UAV FlightGear startup script """
    proc = subprocess.Popen(UAV_SCRIPT, stdout=subprocess.PIPE,
                            stderr=subprocess.PIPE, preexec_fn=os.setsid)
    return proc

def ugv_script():
    """ Run the UGV FlightGear startup script """
    proc = subprocess.Popen(UGV_SCRIPT, stdout=subprocess.PIPE,
                            stderr=subprocess.PIPE, preexec_fn=os.setsid)
    return proc

# MAIN PROGRAM
#---------------------------------------------------------------------------------
class SimulationGUI(QtGui.QWidget):
    """ Base GUI class for sending commands to the JSBSim/FlightGear 
        landing simulation. """
    time = 0
    sliding_window_time = 30
    simulation_running = False
    uav_running = False
    ugv_running = False
    uav_file_exists = False
    ugv_file_exists = False
    pause_sim = False

    def __init__(self, uav, ugv):
        QtGui.QWidget.__init__(self)
        self.setGeometry(0, 0, 300, 700)
        self.main_layout = QtGui.QGridLayout(self)
        self.uav = uav
        self.ugv = ugv

        self.create_menus()

        self.main_layout.addLayout(self.buttons_layout, 0, 1, 2, 2)
        self.main_layout.addLayout(self.settings_layout, 2, 1, 2, 2)

    def init_telnet_uav(self):
        """ Send command to control system of vehicle. """
        if not self.uav_running:
            print("Aerial vehicle not runnning")
        else:
            self.uav.telnet_connect()
        self.uav.view_next()
        self.uav.view_next()
        self.uav.control_velocity()
        self.uav.control_heading()
        self.uav.control_altitude()

    def init_telnet_ugv(self):
        """ Send command to control system of vehicle. """
        if not self.ugv_running:
            print("Ground vehicle not runnning")
        else:
            self.ugv.telnet_connect()
        self.ugv.view_next()
        self.ugv.view_next()
        self.ugv.control_velocity()
        self.ugv.control_heading()

    def closeEvent(self, event):
        """ Close the window and end process. """
        self.stop_control()
        self.close_flightgear()
        self.remove_files()
        event.accept() # let the window close

    def create_menus(self):
        """ Create the buttons and radio buttons """
        self.buttons_layout = QtGui.QGridLayout()

        # Exit Simulation
        self.exit_btn = QtGui.QPushButton("Exit")
        self.exit_btn.clicked.connect(self.close)
        # reset JSBSim
        self.reset_btn = QtGui.QPushButton("Reset")
        self.reset_btn.clicked.connect(self.reset)
        # pause JSBSim
        self.pause_btn = QtGui.QPushButton("Pause")
        self.pause_btn.clicked.connect(self.pause)
        # starting simulation
        self.simulation_btn = QtGui.QPushButton("Start sim")
        self.simulation_btn.clicked.connect(self.simulation_start_stop)

        # starting uav
        self.start_sim_btn = QtGui.QPushButton("Start")
        self.start_sim_btn.clicked.connect(self.start_sim)

        # starting plotting updates
        self.stop_btn = QtGui.QPushButton("Stop")
        self.stop_btn.clicked.connect(self.stop_start)

        self.buttons_layout.addWidget(QtGui.QLabel('Simulation'), 0, 0, 1, 2)
        self.buttons_layout.addWidget(self.reset_btn, 2, 0, 1, 1)
        self.buttons_layout.addWidget(self.pause_btn, 2, 1, 1, 1)
        self.buttons_layout.addWidget(self.start_sim_btn, 3, 0, 1, 2)
        #self.buttons_layout.addWidget(self.simulation_btn, 2, 0, 1, 1)
        self.buttons_layout.addWidget(self.stop_btn, 4, 0, 1, 1)
        self.buttons_layout.addWidget(self.exit_btn, 4, 1, 1, 1)

        # RADIO BUTTONS
        self.settings_layout = QtGui.QGridLayout()
        ctrl_ = {'PID': {'check':False},
                 'TECS': {'check':True}, 'fn': self.toggle_ctrl}

        mode_ = {'Land':  {'check':False},
                 'Hold':  {'check':True},
                 'Align': {'check':False}, 'fn':self.toggle_mode}

        hold_ = {'Wings-level':  {'check':False},
                 'Heading': {'check':True}, 'fn':self.toggle_hold}

        acc_ = {'Acceleration':  {'check':False},
                'Velocity': {'check':True}, 'fn':self.toggle_acc_hold}

        alt_ = {'Altitude':  {'check':True},
                'gamma': {'check':False}, 'fn':self.toggle_alt_hold}

        groups = {'ctrl':ctrl_, 'mode':mode_, 'hold':hold_, 'acc':acc_, 'alt':alt_}
        rbs = {}
        for group_name, group in groups.iteritems():
            new_group = QtGui.QButtonGroup(self.buttons_layout)
            new_group.buttonClicked.connect(group.pop('fn', None))
            i = 0
            rbs[group_name] = {}
            for name, values in group.iteritems():
                rbs[group_name][i] = QtGui.QRadioButton(name)
                rbs[group_name][i].setObjectName(name)
                rbs[group_name][i].setChecked(values['check'])
                new_group.addButton(rbs[group_name][i], i)
                i += 1
            groups[group_name] = new_group

        self.groups = groups

        # Sliders
        uav_sliders = {'Altitude':{'range':(0, 100), 'ticks':5,
                                   'fn':self.altitude_slider},
                       'Velocity':{'range':(15, 35), 'ticks':5,
                                   'fn':self.velocity_slider},
                       'Heading':{'range':(-180, 180), 'ticks':10,
                                  'fn':self.heading_slider},
                       'Acceleration':{'range':(-5, 5), 'ticks':0.2,
                                       'fn':self.acceleration_slider},
                       'Gamma':{'range':(-15, 15), 'ticks':2,
                                       'fn':self.gamma_slider}}

        ugv_sliders = {'Velocity':{'range':(0, 35), 'ticks':5,
                                   'fn':self.velocity_slider_ugv},
                       'Heading':{'range':(-180, 180), 'ticks':10,
                                  'fn':self.heading_slider_ugv},
                       'Acceleration':{'range':(-5, 5), 'ticks':0.2,
                                       'fn':self.acceleration_slider_ugv}}

        self.sliders = dict()
        self.sliders['uav'] = dict()
        for name, values in uav_sliders.iteritems():
            slider = QtGui.QSlider(QtCore.Qt.Horizontal)
            slider.setRange(*values['range'])
            slider.setTickPosition(QtGui.QSlider.TicksBelow)
            slider.setTickInterval(values['ticks'])
            slider.setObjectName(name)
            slider.valueChanged.connect(values['fn'])
            self.sliders['uav'][name] = slider

        self.sliders['ugv'] = dict()
        for name, values in ugv_sliders.iteritems():
            slider = QtGui.QSlider(QtCore.Qt.Horizontal)
            slider.setRange(*values['range'])
            slider.setTickPosition(QtGui.QSlider.TicksBelow)
            slider.setTickInterval(values['ticks'])
            slider.setObjectName(name)
            slider.valueChanged.connect(values['fn'])
            self.sliders['ugv'][name] = slider

        uav_text = QtGui.QLabel('Aerial Vehicle\n')
        uav_text.setAlignment(QtCore.Qt.AlignCenter)
        uav_text.setStyleSheet('font-weight: 500;text-decoration:underline;')
        ugv_text = QtGui.QLabel('Ground Vehicle\n')
        ugv_text.setAlignment(QtCore.Qt.AlignCenter)
        ugv_text.setStyleSheet('font-weight: 500;text-decoration:underline;')
        self.vel_text = QtGui.QLabel('Velocity: 0.0f')
        self.alt_text = QtGui.QLabel('Altitude: 0.0')
        self.chi_text = QtGui.QLabel('Heading: 0.0')
        self.acc_text = QtGui.QLabel('Acceleration: 0.0')
        self.gam_text = QtGui.QLabel('Gamma: 0.0')
        self.vel_ugv_text = QtGui.QLabel('Velocity: 0.0')
        self.chi_ugv_text = QtGui.QLabel('Heading: 0.0')
        self.chi_ugv_text.setFixedWidth(100)
        self.acc_ugv_text = QtGui.QLabel('Acceleration: 0.0')

        self.settings_layout.addWidget(QtGui.QLabel('Control settings'), 0, 0, 1, 4)
        self.settings_layout.addWidget(rbs['ctrl'][0], 1, 0, 1, 1)
        self.settings_layout.addWidget(rbs['ctrl'][1], 1, 1, 1, 1)
        self.settings_layout.addWidget(rbs['mode'][2], 2, 0, 1, 1)
        self.settings_layout.addWidget(rbs['mode'][0], 2, 1, 1, 1)
        self.settings_layout.addWidget(rbs['mode'][1], 2, 2, 1, 1)
        self.settings_layout.addWidget(rbs['hold'][0], 3, 0, 1, 1)
        self.settings_layout.addWidget(rbs['hold'][1], 3, 1, 1, 1)
        self.settings_layout.addWidget(rbs['acc'][0], 4, 0, 1, 1)
        self.settings_layout.addWidget(rbs['acc'][1], 4, 1, 1, 1)
        self.settings_layout.addWidget(rbs['alt'][0], 5, 0, 1, 1)
        self.settings_layout.addWidget(rbs['alt'][1], 5, 1, 1, 1)

        i = 5
        self.settings_layout.addWidget(uav_text, i+1, 0, 1, 4)
        self.settings_layout.addWidget(self.vel_text, i+2, 0, 1, 1)
        self.settings_layout.addWidget(self.alt_text, i+3, 0, 1, 1)
        self.settings_layout.addWidget(self.chi_text, i+4, 0, 1, 1)
        self.settings_layout.addWidget(self.acc_text, i+5, 0, 1, 1)
        self.settings_layout.addWidget(self.gam_text, i+6, 0, 1, 1)
        self.settings_layout.addWidget(ugv_text, i+7, 0, 1, 4)
        self.settings_layout.addWidget(self.vel_ugv_text, i+8, 0, 1, 1)
        self.settings_layout.addWidget(self.chi_ugv_text, i+9, 0, 1, 1)
        self.settings_layout.addWidget(self.acc_ugv_text, i+10, 0, 1, 1)

        self.settings_layout.addWidget(self.sliders['uav']['Velocity'], i+2, 1, 1, 3)
        self.settings_layout.addWidget(self.sliders['uav']['Altitude'], i+3, 1, 1, 3)
        self.settings_layout.addWidget(self.sliders['uav']['Heading'], i+4, 1, 1, 3)
        self.settings_layout.addWidget(self.sliders['uav']['Acceleration'], i+5, 1, 1, 3)
        self.settings_layout.addWidget(self.sliders['uav']['Gamma'], i+6, 1, 1, 3)
        self.settings_layout.addWidget(self.sliders['ugv']['Velocity'], i+8, 1, 1, 3)
        self.settings_layout.addWidget(self.sliders['ugv']['Heading'], i+9, 1, 1, 3)
        self.settings_layout.addWidget(self.sliders['ugv']['Acceleration'], i+10, 1, 1, 3)


    def set_hold_value(self, vehicle, property_, value):
        """ Sets the hold value of some autopilot property. """
        if vehicle == 'uav':
            print("Set uav hold of %s to %i" %(property_, value))
        elif vehicle == 'ugv':
            print("Set ugv hold of %s to %i" %(property_, value))

    def set_setpoint_value(self, vehicle, property_, value):
        """ Sets the value of a setpoint. """
        if vehicle == 'uav':
            print("Set uav %s to %f" %(property_, value))
        elif vehicle == 'ugv':
            print("Set ugv %s to %f" %(property_, value))

    def send_command(self, vehicle):
        """ Send command to control system of vehicle. """
        if vehicle == 'both':
            print("Send command to both vehicles")
        elif vehicle == 'uav':
            print("Send command to UAV")
        elif vehicle == 'ugv':
            print("Send command to UGV")

    def init_telnet(self):
        """ Send command to control system of vehicle. """
        print("Initialize telnet connection")

# :::::::::::::::::: BUTTONS :::::::::::::::::::::::::::
    def reset(self):
        """ Callback for reset button """
        print("Pressed Reset: function reset()")

    def pause(self):
        """ Callback for reset button """
        print("Pressed Pause: function pause()")


    def start_sim(self):
        """ Callback for reset button """
        if not self.ugv_running:
            self.start_sim_btn.setEnabled(False)
            self.ugv_proc = ugv_script()
            time.sleep(0.3)
            self.uav_proc = uav_script()
            self.ugv_running = True
            self.uav_running = True

            self.init_telnet_ugv()
            self.init_telnet_uav()

        else:
            self.ugv_running = False
            self.start_sim_btn.setText('Start')
            self.ugv_proc.terminate()
            self.uav_proc.terminate()

    def simulation_start_stop(self):
        """ Callback for simulation button """
        print("Simulation: function simulation_start_stop()")
        if self.simulation_running:
            self.simulation_running = False
            self.simulation_btn.setText("Start Simulation")

        else:
            self.simulation_running = True
            self.simulation_btn.setText("Stop Simulation")

    def stop_start(self):
        """ Stop or start the plotting """
        if self.simulation_running:
            self.ugv_proc.terminate()
            self.uav_proc.terminate()
            self.stop_btn.setText("Start")
        else:
            self.stop_btn.setText("Stop")

    def remove_files(self):
        """ Delete old files before exiting. """
        if self.uav_file_exists:
            files = [file for file in os.listdir('logs/') if file.startswith('Rascal')]
            for file in files:
                if './logs/' + file != self.UAV_FILE:
                    os.remove('./logs/' + file)
                else:
                    new_file = file
            os.rename('./logs/' + new_file, './logs/Rascal.csv')
        if self.ugv_file_exists:
            files = [file for file in os.listdir('logs/') if file.startswith('followme')]
            for file in files:
                if './logs/' + file != self.UGV_FILE:
                    os.remove('./logs/' + file)
                else:
                    new_file = file
            os.rename('./logs/' +  new_file, './logs/followme.csv')

    def close_flightgear(self):
        if self.uav_running:
            print("Close UAV FlightGear instance")
            self.uav_running = False
            os.killpg(os.getpgid(self.uav_proc.pid), signal.SIGTERM)
        if self.ugv_running:
            print("Close UGV FlightGear instance")
            self.ugv_running = False
            os.killpg(os.getpgid(self.ugv_proc.pid), signal.SIGTERM)

    def stop_control(self):
        print("Stop control system")

    def toggle_mode(self, btn):
        """ Callback for mode button group. Changes the
            mode of the system. """
        print("Toggle flight mode")
        name = btn.objectName()
        if name == 'Align':
            print("Initiate align")
            self.ap_mode = 'ALIGN'
            self.sim.ap_mode = 'ALIGN'
            self.uav.align_mode()
            self.ugv.align_mode()
            for button in  self.groups['hold'].buttons():
                button.setEnabled(False)
            for name, slider in self.sliders['uav'].iteritems():
                if name == 'Altitude':
                    slider.setEnabled(True)
                else:
                    slider.setEnabled(False)
            for slider in self.sliders['ugv'].values():
                slider.setEnabled(False)

        elif name == 'Hold':
            self.ap_mode = 'HOLD'
            self.sim.ap_mode = 'HOLD'
            for button in self.groups['hold'].buttons():
                button.setEnabled(True)
            for slider in self.sliders['uav'].values():
                slider.setEnabled(True)
            for slider in self.sliders['ugv'].values():
                slider.setEnabled(True)

        elif name == 'Land':
            self.ap_mode = 'LAND'
            self.sim.ap_mode = 'LAND'
            self.uav.landing_mode()
            self.ugv.landing_mode()
            for button in  self.groups['hold'].buttons():
                button.setEnabled(False)
            for slider in self.sliders['uav'].values():
                slider.setEnabled(False)
            for slider in self.sliders['ugv'].values():
                slider.setEnabled(False)
        self.send_command('both')

    def toggle_ctrl(self, btn):
        """ Callback for control button group. Changes the
            control mode of the system. """
        print("Toggle control mode")
        name = btn.objectName()
        if name == 'PID':
            # Attitude hold / wings level
            print("Enable PID control system")
            self.uav.toggle_tecs(0)
        if name == 'TECS':
            # Heading follower
            print("Enable TECS control system")
            self.uav.toggle_tecs(1)

    def toggle_hold(self, btn):
        """ Callback for hold button group. Changes the
            hold mode of the system. """
        print("Toggle roll control")
        name = btn.objectName()
        if name == 'Wings-level':
            self.uav.wings_level()
            self.sliders['uav']['Heading'].setEnabled(False)
            self.sliders['ugv']['Heading'].setEnabled(False)
        elif name == 'Heading':
            self.uav.control_heading()
            self.sliders['uav']['Heading'].setEnabled(True)
            self.sliders['ugv']['Heading'].setEnabled(True)
        self.send_command('both')

    def toggle_acc_hold(self, btn):
        """ Callback for hold button group. Changes the
            hold mode of the system. """
        name = btn.objectName()
        if name == 'Velocity':
            self.uav.control_velocity()
            self.ugv.control_velocity()
            self.sliders['uav']['Acceleration'].setEnabled(False)
            self.sliders['ugv']['Acceleration'].setEnabled(False)
        elif name == 'Acceleration':
            self.uav.control_acceleration()
            self.ugv.control_acceleration()
            self.sliders['uav']['Acceleration'].setEnabled(True)
            self.sliders['ugv']['Acceleration'].setEnabled(True)
        self.send_command('both')
        
    def toggle_alt_hold(self, btn):
        """ Callback for hold button group. Changes the
            hold mode of the system. """
        name = btn.objectName()
        if name == 'Altitude':
            self.uav.control_altitude()
            self.sliders['uav']['Acceleration'].setEnabled(True)
            self.sliders['uav']['Gamma'].setEnabled(False)
        elif name == 'gamma':
            self.uav.control_flight_path()
            self.sliders['uav']['Acceleration'].setEnabled(False)
            self.sliders['uav']['Gamma'].setEnabled(True)
        self.send_command('both')

    def velocity_slider(self, value):
        """ Callback for UAV velocity slider. """
        self.set_setpoint_value('uav', 'velocity', value)
        self.vel_text.setText("Velocity: %2.2f" %value)
        self.send_command('uav')

    def altitude_slider(self, value):
        """ Callback for UAV altitude slider. """
        self.set_setpoint_value('uav', 'altitude', value)
        self.alt_text.setText("Altitude: %2.2f" %value)
        self.send_command('uav')

    def heading_slider(self, value):
        """ Callback for UAV heading slider. """
        self.set_setpoint_value('uav', 'heading', value)
        self.chi_text.setText("Heading: %4.2f" %value)
        self.send_command('uav')
                       
    def acceleration_slider(self, value):
        """ Callback for UAV acceleration slider. """
        self.set_setpoint_value('uav', 'acceleration', value)
        self.acc_text.setText("Acceleration: %4.1f" %value)
        self.send_command('uav')
        
    def gamma_slider(self, value):
        """ Callback for UAV gamma slider. """
        self.set_setpoint_value('uav', 'gamma', value)
        self.gam_text.setText("Gamma: %4.1f" %value)
        self.send_command('uav')

    def velocity_slider_ugv(self, value):
        """ Callback for UGV velocity slider. """
        self.set_setpoint_value('ugv', 'velocity', value)
        self.vel_ugv_text.setText("Velocity: %2.2f" %value)
        self.send_command('ugv')

    def heading_slider_ugv(self, value):
        """ Callback for UGV heading slider. """
        self.set_setpoint_value('ugv', 'heading', value)
        self.chi_ugv_text.setText("Heading: %4.2f" %value)
        self.send_command('ugv')

    def acceleration_slider_ugv(self, value):
        """ Callback for UGV acceleration slider. """
        self.set_setpoint_value('ugv', 'acceleration', value)
        self.acc_ugv_text.setText("Acceleration: %4.1f" %value)
        self.send_command('ugv')

# :::::::::::::::::: UPDATE FUNCTIONS :::::::::::::::::::::
    def start(self):
        pass


