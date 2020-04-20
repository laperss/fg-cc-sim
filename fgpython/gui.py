#! /usr/bin/python
""" Gui for visualising data from JSBSim """
from __future__ import print_function
import subprocess
import time
import signal
import os
from PyQt5 import QtCore, QtGui
import pyqtgraph.exporters
import pyqtgraph as pg


def run_fg_script(script, vehicle):
    """ Run a FlightGear startup script """
    command = [script]
    for key, value in vehicle.arguments.items():
        command.append('--' + key + '=' + str(value))
    command.append('--callsign="%s"' % vehicle.name)
    command.append('--multiplay=out,60,127.0.0.1,%i' %
                   (vehicle.mp_output_port))
    command.append('--multiplay=in,60,127.0.0.1,%i' % (vehicle.mp_input_port))
    command.append('--generic=socket,in,180,localhost,%i,udp,%s'
                   % (vehicle.control.output_port, vehicle.control.input_protocol))
    command.append('--generic=socket,out,180,localhost,%i,udp,%s'
                   % (vehicle.control.input_port, vehicle.control.output_protocol))
    command.append('--props=socket,bi,20,,%i,tcp' % vehicle.command.port)
    command.append('--prop:/engines/engine[0]/running=true')
    command.append('--prop:/engines/engine[1]/running=true')
    command.append('--prop:/engines/engine[2]/running=true')
    command.append('--prop:/engines/engine[3]/running=true')
    print("Attempt to start flightgear: ")
    print(*command, sep="  ")

    proc = subprocess.Popen(command, stdout=subprocess.PIPE,
                            stderr=subprocess.PIPE, preexec_fn=os.setsid)
    return proc

# MAIN PROGRAM
# ---------------------------------------------------------------------------------


class SimulationGUI(QtGui.QWidget):
    """ Base GUI class for sending commands to the JSBSim/FlightGear
        landing simulation. """
    simulation_running = False
    pause_sim = False

    def __init__(self, vehicles):
        QtGui.QWidget.__init__(self)
        self.setGeometry(0, 0, 300, 700)
        self.main_layout = QtGui.QGridLayout(self)
        self.vehicles = vehicles
        self.create_menus()
        self.main_layout.addLayout(self.buttons_layout, 0, 1, 2, 2)
        self.main_layout.addLayout(self.settings_layout, 2, 1, 2, 2)

    def init_telnet(self, vehicle):
        """ Send command to control system of vehicle. """
        if not vehicle.running:
            print("%s is not runnning" % vehicle.name)
        else:
            vehicle.command.telnet_connect()
        vehicle.command.view_next()
        vehicle.command.view_next()
        vehicle.command.control_velocity()
        vehicle.command.control_heading()
        vehicle.command.control_altitude()

    def closeEvent(self, event):
        """ Close the window and end process. """
        self.stop_control()
        self.close_flightgear()
        self.remove_files()
        event.accept()  # let the window close

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

        # starting the scripts
        self.start_sim_btn = QtGui.QPushButton("Start")
        self.start_sim_btn.clicked.connect(self.start_sim)

        # starting plotting updates
        self.stop_btn = QtGui.QPushButton("Stop")
        self.stop_btn.clicked.connect(self.stop_start)

        self.buttons_layout.addWidget(QtGui.QLabel('Simulation'), 0, 0, 1, 2)
        self.buttons_layout.addWidget(self.reset_btn, 2, 0, 1, 1)
        self.buttons_layout.addWidget(self.pause_btn, 2, 1, 1, 1)
        self.buttons_layout.addWidget(self.start_sim_btn, 3, 0, 1, 2)
        # self.buttons_layout.addWidget(self.simulation_btn, 2, 0, 1, 1)
        self.buttons_layout.addWidget(self.stop_btn, 4, 0, 1, 1)
        self.buttons_layout.addWidget(self.exit_btn, 4, 1, 1, 1)

        # RADIO BUTTONS
        self.settings_layout = QtGui.QGridLayout()
        ctrl_ = {'PID': {'check': False},
                 'TECS': {'check': True}, 'fn': self.toggle_ctrl}

        mode_ = {'Land':  {'check': False},
                 'Hold':  {'check': True},
                 'Align': {'check': False}, 'fn': self.toggle_mode}

        hold_ = {'Wings-level':  {'check': False},
                 'Heading': {'check': True}, 'fn': self.toggle_hold}

        acc_ = {'Acceleration':  {'check': False},
                'Velocity': {'check': True}, 'fn': self.toggle_acc_hold}

        alt_ = {'Altitude':  {'check': True},
                'gamma': {'check': False}, 'fn': self.toggle_alt_hold}

        groups = {'ctrl': ctrl_, 'mode': mode_,
                  'hold': hold_, 'acc': acc_, 'alt': alt_}
        rbs = {}
        for group_name, group in groups.items():
            new_group = QtGui.QButtonGroup(self.buttons_layout)
            new_group.buttonClicked.connect(group.pop('fn', None))
            i = 0
            rbs[group_name] = {}
            for name, values in group.items():
                rbs[group_name][i] = QtGui.QRadioButton(name)
                rbs[group_name][i].setObjectName(name)
                rbs[group_name][i].setChecked(values['check'])
                new_group.addButton(rbs[group_name][i], i)
                i += 1
            groups[group_name] = new_group

        self.groups = groups

        self.settings_layout.addWidget(
            QtGui.QLabel('Control settings'), 0, 0, 1, 4)
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

        # Sliders
        for vehicle in self.vehicles:
            # Vehicle label
            vehicle.text = QtGui.QLabel('%s\n' % vehicle.name)
            vehicle.text.setAlignment(QtCore.Qt.AlignCenter)
            vehicle.text.setStyleSheet(
                'font-weight: 500;text-decoration:underline;')
            # One slider for every property
            for name, values in vehicle.control_variables.items():
                slider = QtGui.QSlider(QtCore.Qt.Horizontal)
                slider.setRange(*values['range'])
                slider.setTickPosition(QtGui.QSlider.TicksBelow)
                slider.setObjectName(name)
                slider.id = vehicle.id
                slider.valueChanged.connect(self.slider_moving)
                vehicle.control_variables[name]['slider'] = slider
                vehicle.control_variables[name]['text'] = QtGui.QLabel(
                    '%s: 0.0' % name)
        i = 5
        j = 1
        for vehicle in self.vehicles:
            self.settings_layout.addWidget(vehicle.text, i+j, 0, 1, 4)
            j += 1
            for name, prop in vehicle.control_variables.items():
                slider = prop['slider']
                text = prop['text']
                self.settings_layout.addWidget(text, i+j, 0, 1, 1)
                self.settings_layout.addWidget(slider, i+j, 1, 1, 3)
                j += 1

# :::::::::::::::::: BUTTONS :::::::::::::::::::::::::::
    def reset(self):
        """ Reset JSBSim """
        print("Restart vehicle simulations (NOT WORKING)")
        return
        # OLD CODE. FIX!
        if self.uav_file_exists:
            self.uav.command.reset()
            time.sleep(0.5)
            files = [file for file in os.listdir(
                'logs/') if file.startswith('Rascal_')]
            if len(files) > 0:
                self.UAV_FILE = './logs/' + sorted(files, reverse=True)[0]
        if self.ugv_file_exists:
            self.ugv.command.reset()
            time.sleep(0.5)
            files = [file for file in os.listdir(
                'logs/') if file.startswith('ground-vehicle_')]
            if len(files) > 0:
                self.UGV_FILE = './logs/' + sorted(files, reverse=True)[0]

    def pause(self):
        """ Pause/Resume FlightGear  """
        if self.pause_sim:
            print("Resume vehicle simulations")
            for vehicle in self.vehicles:
                vehicle.command.resume()
            self.pause_sim = False
            self.pause_btn.setText("Pause")

        else:
            print("Pause vehicle simulations")
            for vehicle in self.vehicles:
                vehicle.command.pause()
            self.pause_sim = True
            self.sim.stop_control_thread()
            self.pause_btn.setText("Resume")

    def start_sim(self):
        """ Callback for reset button """
        if not self.simulation_running:
            self.start_sim_btn.setEnabled(False)
            self.proc = []
            for vehicle in self.vehicles:
                self.proc.append(run_fg_script(vehicle.path, vehicle))
                time.sleep(0.3)

            for vehicle in self.vehicles:
                vehicle.running = True
                self.init_telnet(vehicle)
        else:
            self.simulation_running = False
            self.start_sim_btn.setText('Start')
            for proc in self.proc:
                proc.terminate()

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
            for proc in self.proc:
                proc.terminate()
            self.stop_btn.setText("Start")
        else:
            self.stop_btn.setText("Stop")

    def remove_files(self):
        """ Delete old files before exiting. """
        if self.uav_file_exists:
            files = [file for file in os.listdir(
                'logs/') if file.startswith('Rascal')]
            for file in files:
                if './logs/' + file != self.UAV_FILE:
                    os.remove('./logs/' + file)
                else:
                    new_file = file
            os.rename('./logs/' + new_file, './logs/Rascal.csv')
        if self.ugv_file_exists:
            files = [file for file in os.listdir(
                'logs/') if file.startswith('followme')]
            for file in files:
                if './logs/' + file != self.UGV_FILE:
                    os.remove('./logs/' + file)
                else:
                    new_file = file
            os.rename('./logs/' + new_file, './logs/followme.csv')

    def close_flightgear(self):
        """ Kill the FG processes before shutting down Python """
        try:
            for proc in self.proc:
                os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
            print("FlightGear processes closed")
        except:
            print("No FlightGear processes running")
        for vehicle in self.vehicles:
            vehicle.running = False

    def stop_control(self):
        print("Stop control system")
# :::::::::::::::::: RADIO BTN :::::::::::::::::::::::::::

    def toggle_mode(self, btn):
        """ Callback for mode button group. Changes the
            mode of the system. """
        print("Toggle flight mode")
        name = btn.objectName()
        if name == 'Align':
            print("Initiate align")
            self.ap_mode = 'ALIGN'
            self.sim.ap_mode = 'ALIGN'
            for vehicle in self.vehicles:
                vehicle.command.align_mode()
            for button in self.groups['hold'].buttons():
                button.setEnabled(False)
            for vehicle in self.vehicles:
                for prop in vehicle.control_variables.values():
                    prop['slider'].setEnabled(False)
            for uav in [vehicle for vehicle in self.vehicles if vehicle.type == 'uav']:
                uav.control_variables['Altitude']['slider'].setEnabled(True)

        elif name == 'Hold':
            self.ap_mode = 'HOLD'
            self.sim.ap_mode = 'HOLD'
            for button in self.groups['hold'].buttons():
                button.setEnabled(True)
            for vehicle in self.vehicles:
                for prop in vehicle.control_variables.values():
                    prop['slider'].setEnabled(True)

        elif name == 'Land':
            self.ap_mode = 'LAND'
            self.sim.ap_mode = 'LAND'
            for vehicle in self.vehicles:
                vehicle.command.landing_mode()
            for button in self.groups['hold'].buttons():
                button.setEnabled(False)
            for vehicle in self.vehicles:
                for prop in vehicle.control_variables.values():
                    prop['slider'].setEnabled(False)

    def toggle_ctrl(self, btn):
        """ Callback for control button group. Changes the control mode of the system. """
        print("Toggle control mode")
        name = btn.objectName()
        for uav in [vehicle for vehicle in self.vehicles if vehicle.type == 'uav']:
            if name == 'PID':
                print("Enable PID control system")
                uav.command.toggle_tecs(0)
            elif name == 'TECS':
                print("Enable TECS control system")
                uav.command.toggle_tecs(1)

    def toggle_hold(self, btn):
        """ Callback for hold button group. Changes the hold mode of the system. """
        print("Toggle roll control")
        name = btn.objectName()
        for uav in [vehicle for vehicle in self.vehicles if vehicle.type == 'uav']:
            if name == 'Wings-level':
                uav.command.wings_level()
                for vehicle in self.vehicles:
                    vehicle.control_variables['Heading']['slider'].setEnabled(
                        False)
            elif name == 'Heading':
                uav.command.control_heading()
                for vehicle in self.vehicles:
                    vehicle.control_variables['Heading']['slider'].setEnabled(
                        True)

    def toggle_acc_hold(self, btn):
        """ Callback for acceleration button group. Changes the hold mode of the system. """
        name = btn.objectName()
        if name == 'Velocity':
            for vehicle in self.vehicles:
                vehicle.command.control_velocity()
            for vehicle in self.vehicles:
                vehicle.control_variables['Acceleration']['slider'].setEnabled(
                    False)
                vehicle.control_variables['Velocity']['slider'].setEnabled(
                    True)
        elif name == 'Acceleration':
            for vehicle in self.vehicles:
                vehicle.command.control_acceleration()
            for vehicle in self.vehicles:
                vehicle.control_variables['Acceleration']['slider'].setEnabled(
                    True)
                vehicle.control_variables['Velocity']['slider'].setEnabled(
                    False)

    def toggle_alt_hold(self, btn):
        """ Callback for altitude button group. Changes the hold mode of the system. """
        name = btn.objectName()
        for uav in [vehicle for vehicle in self.vehicles if vehicle.type == 'uav']:
            if name == 'Altitude':
                uav.command.control_altitude()
                uav.control_variables['Altitude']['slider'].setEnabled(True)
                uav.control_variables['Gamma']['slider'].setEnabled(False)

            elif name == 'gamma':
                uav.command.control_flight_path()
                uav.control_variables['Altitude']['slider'].setEnabled(False)
                uav.control_variables['Gamma']['slider'].setEnabled(True)

    def slider_moving(self, value):
        prop = str(self.sender().objectName())
        id_ = str(self.sender().id)
        # Get the correct vehicle from the ID
        vehicle, = [v for v in self.vehicles if v.id == id_]
        try:
            vehicle.control.setpoint[prop.lower()] = value
        except:
            raise ValueError(
                "ID given: %s. Acceptable IDs are 'uav' or 'ugv'." % id_)
        vehicle.control_variables[prop]['text'].setText(
            '%s: %2.1f' % (prop, value))
