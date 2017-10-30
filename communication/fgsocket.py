#! /usr/bin/python
""" Control and sending commands between Python and JSBSim over TCP.

Author: Linnea Persson, laperss@kth.se 

Suitable for sending many commands in a stream.
Usage:
    uav_socket = control.fgsocket.UAV(5515, 5514)
    ugv_socket = control.fgsocket.UGV(5526, 5525)
Set the reference values to send:
    uav_socket.setpoint['acceleration'] = 0.3
    ugv_socket.setpoint['heading'] = 12

"""
from __future__ import print_function
import thread
import socket
import re
import math

HEADING = 199.67
RAD2DEG = 57.2957795
DEG2RAD = 0.0174532925
FEET2M = 0.3048
M2FEET = 3.28084

class Vehicle(object):
    """ Base class for JSBSim controller. """
    setpoint = {'altitude': 15, 'velocity': 25, 'heading': 0, 'acceleration': 0}
    hold = {'altitude': 1, 'velocity': 1, 'heading': 1, 'attitude': 0, 'acceleration': 0, 'gamma':0}
    heading = HEADING
    pause = 0
    reset = 0
    data = []
    def __init__(self, input_port, output_port):
        self.output_port = output_port

        self.socket_in = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket_out = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        self.socket_in.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.socket_out.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

        self.socket_in.bind(('localhost', input_port))
        self.update = True


    def send_command_udp(self, command, port):
        """ Send actuator commands to JSBSim/FlightGear via UDP. """
        message = ''
        for cmd in command[:-1]:
            message = '%s%f, ' %(message, cmd)
        message = '%s%s\n' %(message, str(command[-1]))
        self.socket_out.sendto(message, ('localhost', port))


    def get_state(self, idx):
        """ Return the current vehicle state """
        return [self.data[i] for i in idx]

    def start_update_state(self):
        self.update = True
        thread.start_new_thread(self.update_state,())

    def update_state(self):
        while self.update == True:
            data, addr = self.socket_in.recvfrom(2048)
            data.rstrip('\n')
            if not data: break
            self.data = [float(i) for i in re.split(r'\t+', data)]
##################################################################################


class UAV(Vehicle):
    setpoint = {'altitude': 20, 'velocity': 23, 'heading': 0, 'acceleration': 0, 'gamma':0}
    hold = {'altitude': 1, 'velocity': 1, 'heading': 1, 'attitude': 0, 'acceleration': 0, 'gamma':0}

    """ Control signals to and from a JSBSim UAV"""
    def __init__(self, input_port, output_port):
        Vehicle.__init__(self, input_port, output_port)

    def send_cmd(self):
        """ Define the message and send to UDP function. """
        command = [self.setpoint['altitude']*3.28084,
                   (self.setpoint['heading'] + HEADING)*DEG2RAD,
                   self.setpoint['velocity']*3.28084,
                   self.setpoint['acceleration']*3.28084,
                   math.radians(self.setpoint['gamma'])]
        self.send_command_udp(command, self.output_port)



##################################################################################


class UGV(Vehicle):
    setpoint = {'velocity': 23, 'heading': 0, 'acceleration': 0}
    hold = {'velocity': 1, 'heading': 1, 'acceleration': 0}
    """ Control signals to and from a JSBSim UGV"""
    def __init__(self, input_port, output_port):
        Vehicle.__init__(self, input_port, output_port)

    def send_cmd(self):
        """ Define the message and send to UDP function. """
        command = [0.0,
                  (self.setpoint['heading'] + HEADING)*DEG2RAD,
                   self.setpoint['velocity']*3.28084,
                   self.setpoint['acceleration']*3.28084,
                   0]
        self.send_command_udp(command, self.output_port)


