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

    proc = subprocess.Popen(command, stdout=subprocess.PIPE,
                            stderr=subprocess.PIPE, preexec_fn=os.setsid)
    return proc


# Vehicle class
class Vehicle(object):
    id_counter = 0
    path = None

    def __init__(self, name, type_):
        self.name = name
        self.id = type_ + str(self.id_counter)
        self.type = type_
        self.running = False


# MAIN PROGRAM
# ---------------------------------------------------------------------------------
class SimulationGUI(QtGui.QWidget):
    """ Base GUI class for sending commands to the JSBSim/FlightGear
        landing simulation. """
    simulation_running = False
    pause_sim = False
    uav_file_exists = False
    ugv_file_exists = False

    def __init__(self, vehicles):
        QtGui.QWidget.__init__(self)
        self.setGeometry(0, 0, 300, 700)
        main_layout = QtGui.QGridLayout(self)
        self.vehicles = vehicles
        self.create_menus(main_layout)

        # Click all checked buttons
        for group in self.groups.values():
            for button in group.buttons():
                if button.isChecked():
                    button.click()

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

    def create_menus(self, layout):
        """ Create the menu buttons and control settings layout """
        main_menu_layout = QtGui.QGridLayout()
        ctrl_settings_layout = QtGui.QGridLayout()

        menu = (("Start", self.start_sim),
                ("Exit", self.close),
                ("Pause", self.pause),
                ("Reset", self.reset),
                ("Start sim", self.simulation_start_stop),
                ("Stop", self.stop_start))

        button_groups = (('ctrl', self.toggle_ctrl, (('PID', False),
                                                     ('TECS', True))),
                         ('mode', self.toggle_mode, (('Land', False),
                                                     ('Hold', True),
                                                     ('Align', False))),
                         ('hold', self.toggle_hold, (('Wings-level', False),
                                                     ('Yaw-rate', False),
                                                     ('Heading', True))),
                         ('acc', self.toggle_acc_hold, (('Acceleration', False),
                                                        ('Velocity', True))),
                         ('alt', self.toggle_alt_hold, (('Altitude', True),
                                                        ('gamma', False))))

        main_menu_layout.addWidget(QtGui.QLabel('Simulation'), 1, 0, 1, 2)
        self.menu = dict()
        itr = 2
        for name, function in menu:
            btn = QtGui.QPushButton(name)
            btn.clicked.connect(function)
            self.menu[name] = btn
            main_menu_layout.addWidget(btn, itr - itr % 2, itr % 2, 1, 1)
            itr += 1

        # CONTROL SETTINGS ------------------------------
        ctrl_settings_layout.addWidget(
            QtGui.QLabel('Control settings'), 0, 0, 1, 4)
        self.groups = dict()
        j = 1
        for group in button_groups:
            new_group = QtGui.QButtonGroup(main_menu_layout)
            new_group.buttonClicked.connect(group[1])
            i = 0
            for option in group[2]:
                btn = QtGui.QRadioButton(option[0])
                btn.setObjectName(option[0])
                btn.setChecked(option[1])
                new_group.addButton(btn, i)
                ctrl_settings_layout.addWidget(btn, j, i, 1, 1)
                i += 1
            self.groups[group[0]] = new_group
            j += 1

        # Sliders ---------------------------------------
        self.sliders = dict()
        for vehicle in self.vehicles:
            vehicle.text = QtGui.QLabel('%s\n' % vehicle.name)
            vehicle.text.setStyleSheet('font-weight:500;')
            ctrl_settings_layout.addWidget(vehicle.text, j, 0, 1, 4)
            j += 1
            # One slider for every property
            for name, values in vehicle.control_variables.items():
                slider = QtGui.QSlider(QtCore.Qt.Horizontal)
                slider.setRange(*values['range'])
                slider.setTickPosition(QtGui.QSlider.TicksBelow)
                slider.setObjectName(name)
                slider.id = vehicle.id
                slider.valueChanged.connect(self.slider_moving)
                slider.text = QtGui.QLabel('%s: 0.0' % name)

                ctrl_settings_layout.addWidget(slider.text, j, 0, 1, 1)
                ctrl_settings_layout.addWidget(slider, j, 1, 1, 3)
                self.sliders[slider.id+'_'+name.lower()] = slider
                j += 1

        layout.addLayout(main_menu_layout, 0, 1, 2, 2)
        layout.addLayout(ctrl_settings_layout, 2, 1, 2, 2)


# :::::::::::::::::: BUTTONS :::::::::::::::::::::::::::
    def reset(self):
        """ Reset JSBSim """
        print("* Restart vehicle simulations (NOT WORKING)")
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
            print("* Resume vehicle simulations")
            for vehicle in self.vehicles:
                vehicle.command.resume()
            self.pause_sim = False
            self.menu["Pause"].setText("Pause")

        else:
            print("* Pause vehicle simulations")
            for vehicle in self.vehicles:
                vehicle.command.pause()
            self.pause_sim = True
            self.sim.stop_control_thread()
            self.menu["Pause"].setText("Resume")

    def start_sim(self):
        """ Callback for reset button """
        if not self.simulation_running:
            self.menu["Start"].setEnabled(False)
            self.proc = []
            for vehicle in self.vehicles:
                self.proc.append(run_fg_script(vehicle.path, vehicle))
                time.sleep(0.3)

            for vehicle in self.vehicles:
                vehicle.running = True
                self.init_telnet(vehicle)
        else:
            self.simulation_running = False
            self.menu["Start"].setText('Start')
            for proc in self.proc:
                proc.terminate()

    def simulation_start_stop(self):
        """ Callback for simulation button """
        print("* Simulation: function simulation_start_stop()")
        if self.simulation_running:
            self.simulation_running = False
            self.menu["Start sim"].setText("Start Simulation")

        else:
            self.simulation_running = True
            self.menu["Start sim"].setText("Stop Simulation")

    def stop_start(self):
        """ Stop or start the plotting """
        if self.simulation_running:
            for proc in self.proc:
                proc.terminate()
            self.menu["Stop"].setText("Start")
        else:
            self.menu["Stop"].setText("Stop")

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
        print("* Stop control system (not implemented)")


# :::::::::::::::::: RADIO BTN :::::::::::::::::::::::::::
    def toggle_mode(self, btn):
        """ Callback for mode button group. Changes the
            mode of the system. """
        mode = btn.objectName()
        print("* Toggle flight mode: ", mode)
        self.ap_mode = mode.upper()
        self.sim.ap_mode = mode.upper()

        if mode == 'Align':
            for vehicle in self.vehicles:
                vehicle.command.align_mode()
            for name, slider in self.sliders.items():
                if "altitude" in name:
                    slider.setEnabled(True)
                else:
                    slider.setEnabled(False)

        elif mode == 'Hold':
            for vehicle in self.vehicles:
                vehicle.command.hold_mode()
            for slider in self.sliders.values():
                slider.setEnabled(True)

        elif mode == 'Land':
            for vehicle in self.vehicles:
                vehicle.command.landing_mode()
            for slider in self.sliders.values():
                slider.setEnabled(False)

    def toggle_ctrl(self, btn):
        """ Callback for control button group. Changes the control mode of the system. """
        print("* Toggle control mode: ", end='')
        ctrl_mode = btn.objectName()
        for uav in [vehicle for vehicle in self.vehicles if vehicle.type == 'uav']:
            uav.command.toggle_tecs(ctrl_mode == 'TECS')

    def toggle_hold(self, btn):
        """ Callback for hold button group. Changes the hold mode of the system. """
        print("* Toggle roll control")
        enable = btn.objectName()

        if enable == 'Wings-level':
            for vehicle in self.vehicles:
                vehicle.command.wings_level()
        elif enable == 'Heading':
            for vehicle in self.vehicles:
                vehicle.command.control_heading()

        for name, slider in self.sliders.items():
            if "heading" in name:
                slider.setEnabled(enable == 'Heading')

    def toggle_acc_hold(self, btn):
        """ Callback for acceleration button group. Changes the hold mode of the system. """
        enable_velocity = btn.objectName() == 'Velocity'
        if enable_velocity:
            for vehicle in self.vehicles:
                vehicle.command.control_velocity()
        else:
            for vehicle in self.vehicles:
                vehicle.command.control_acceleration()
        for name, slider in self.sliders.items():
            if "acceleration" in name:
                slider.setEnabled(not enable_velocity)
            elif "velocity" in name:
                slider.setEnabled(enable_velocity)

    def toggle_alt_hold(self, btn):
        """ Callback for altitude button group. Changes the hold mode of the system. """
        btn_name = btn.objectName()
        if btn_name == 'Altitude':
            for vehicle in self.vehicles:
                vehicle.command.control_altitude()
            for name, slider in self.sliders.items():
                if "altitude" in name:
                    slider.setEnabled(True)
                elif "gamma" in name:
                    slider.setEnabled(False)

        elif btn_name == 'gamma':
            for vehicle in self.vehicles:
                vehicle.command.control_flight_path()
            for name, slider in self.sliders.items():
                if "altitude" in name:
                    slider.setEnabled(False)
                elif "gamma" in name:
                    slider.setEnabled(True)

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
        self.sender().text.setText('%s: %2.1f' % (prop, value))
