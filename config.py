#!/usr/bin/env python3
import os
from fgpython import comm, Vehicle, FGTelnetConnection, FGSocketConnection
from control import Positioner

# GLOBAL SETTINGS -----------------------------------------
RAD2DEG = 57.2957795
DEG2RAD = 0.0174532925
FEET2M = 0.3048
M2FEET = 3.28084

# Set reference origin and heading
origin = (42.37824878120545, -71.00457885362507)
heading = 199.67
positioner = Positioner(origin, heading)
FGSocketConnection.heading = heading
v_ref = 20.0  # Reference velocity

# The path to the FlightGear folder
PATH = os.path.dirname(os.path.abspath(__file__))
SCRIPT = os.path.join(PATH, 'flightgear/run_flightgear.sh')
Vehicle.path = SCRIPT

# Location of the log files
UAV_FILE = './logs/Rascal.csv'
UGV_FILE = './logs/ground-vehilce.csv'

# Communication setup
multiplayer_ports = (5000, 5002)   # multiplayer ports
telnet_ports = (6500, 6600)        # telnet ports
input_ports = (5526, 5515)         # input socket ports
output_ports = (5525, 5514)        # output socket ports
in_protocol = ('InputProtocol', 'InputProtocol')
out_protocol = ('UAVProtocol', 'UGVProtocol')

# VEHICLE 1 SETUP  ---------------------------------------
uav = Vehicle("Aerial Vehicle", "uav")
uav_initial = positioner.get_global_position(-70, 5)
com1 = comm(in_protocol[0], out_protocol[0], input_ports[0], output_ports[0])
uav.control_variables = {'Altitude': {'range': (0, 100), 'value': 18},
                         'Velocity': {'range': (15, 35), 'value': 23},
                         'Heading': {'range': (-180, 180), 'value': 0},
                         'Acceleration': {'range': (-5, 5), 'value': 0},
                         'Gamma': {'range': (-15, 15), 'value': 0}}

uav.mp_input_port = multiplayer_ports[0]
uav.mp_output_port = multiplayer_ports[1]
uav.command = FGTelnetConnection('localhost', telnet_ports[0])
uav.control = FGSocketConnection(com1)

uav.arguments = {'aircraft': 'Rascal110-JSBSim',
                 'prop:/fdm/jsbsim/positioning/ref-origin-lat': origin[0],
                 'prop:/fdm/jsbsim/positioning/ref-origin-lon': origin[1],
                 'prop:/fdm/jsbsim/positioning/ref-heading': heading,
                 'lat': uav_initial[0], 'lon': uav_initial[1],
                 'altitude': 25,
                 'uBody': 25,
                 'heading': 199,
                 'glideslope': 0,
                 'roll': 0}

# VEHICLE 2 SETUP  ---------------------------------------
ugv = Vehicle("Ground Vehicle", "ugv")
ugv_initial = positioner.get_global_position(10, 0)
com2 = comm(in_protocol[1], out_protocol[1], input_ports[1], output_ports[1])

ugv.control_variables = {'Velocity': {'range': (0, 35), 'value': 23},
                         'Heading': {'range': (-180, 180), 'value': 0},
                         'Acceleration': {'range': (-5, 5), 'value': 0}}

ugv.mp_input_port = multiplayer_ports[1]
ugv.mp_output_port = multiplayer_ports[0]
ugv.command = FGTelnetConnection('localhost', telnet_ports[1])
ugv.control = FGSocketConnection(com2)

ugv.arguments = {'aircraft': 'ground-vehicle',
                 'prop:/fdm/jsbsim/positioning/ref-origin-lat': origin[0],
                 'prop:/fdm/jsbsim/positioning/ref-origin-lon': origin[1],
                 'prop:/fdm/jsbsim/positioning/ref-heading': heading,
                 'lat': ugv_initial[0], 'lon': ugv_initial[1],
                 'heading': 199}


# Setup the correct scaling and offset for variables
uav.control.update_scale('altitude', M2FEET)
uav.control.update_scale('velocity', M2FEET)
uav.control.update_scale('acceleration', M2FEET)
uav.control.update_scale('heading', DEG2RAD)
uav.control.update_bias('heading', heading)
uav.control.update_scale('gamma', DEG2RAD)

ugv.control.update_scale('velocity', M2FEET)
ugv.control.update_scale('acceleration', M2FEET)
ugv.control.update_scale('heading', DEG2RAD)
ugv.control.update_bias('heading', heading)
