#! /usr/bin/python
""" Send commands between Python and JSBSim over TCP.

Author: Linnea Persson, laperss@kth.se 

Suitable for sending many commands in a stream.
It is mainly used to update the reference values for the vehicles. 
You must set the heading with which you define the heading relative to.

Usage:
    FGSocketConnection.heading = HEADING
    uav_socket = FGSocketConnection(5515, 5514)
    ugv_socket = FGSocketConnection(5526, 5525)
Set the reference values to send:
    uav_socket.setpoint['acceleration'] = 0.3
    ugv_socket.setpoint['heading'] = 12
"""
from __future__ import print_function
import math
import _thread
import socket
import re
import xml.etree.ElementTree as ET
import os 

RAD2DEG = 57.2957795
DEG2RAD = 0.0174532925
FEET2M = 0.3048
M2FEET = 3.28084


class FGSocketConnection(object):
    """ JSBSim communication system. 
        Uses UDP to reieve data from and send data to FlightGear.

        The protocols for receiving data are defined in:
            * flightgear/protocols/UAVProtocol.xml
            * flightgear/protocols/UAVProtocol.xml
        The protocol for sending data is: 
            * flightgear/protocols/InputProtocol.xml
    """
    heading = 199.67

    def __init__(self, input_port, output_port, in_protocol='InProtocol', out_protocol='OutProtocol'):
        self.data = []

        self.output_port = output_port
        self.input_port = input_port
        self.input_protocol = in_protocol
        self.output_protocol = out_protocol
        self.update = False

        path = os.path.dirname(os.path.abspath(__file__))
        output_script = os.path.join(path, '../flightgear/protocols/'+out_protocol+'.xml')
        input_script = os.path.join(path, '../flightgear/protocols/'+in_protocol+'.xml')
        input_root = ET.parse(input_script).getroot()
        self.setpoint = dict()
        for chunk in input_root[0][0].findall('chunk'):
            name = chunk.find('name').text
            self.setpoint[name] = 0.0

        self.setup_sockets()

    def update_setpoint(prop, value, scale=1.0, bias=0.0):
        self.setpoint[prop] = value*scale + bias

    def setup_sockets(self):
        """ Setup the socket communication """
        self.socket_in = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket_out = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        self.socket_in.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.socket_out.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

        self.socket_in.bind(('localhost', self.input_port))

    def send_command_udp(self, command, port):
        """ Send actuator commands to JSBSim/FlightGear via UDP. """
        message = ''
        for cmd in command[:-1]:
            message = '%s%f, ' % (message, cmd)
        message = '%s%s\n' % (message, str(command[-1]))
        self.socket_out.sendto(message.encode(), ('localhost', port))

    def start_receive_state(self):
        """ Starts the thread for reading data from flightGear """
        self.update = True
        _thread.start_new_thread(self.receive_state, ())

    def receive_state(self):
        """ Separates data and updates the "data" variable"""
        while self.update == True:
            data, addr = self.socket_in.recvfrom(2048)
            data = data.decode()
            data.rstrip('\n')
            if not data:
                break
            self.data = [float(i) for i in re.split(r'\t+', data)]

    def get_state(self, idx):
        """ Return the current vehicle state """
        return [self.data[i] for i in idx]

    def send_cmd(self):
        """ Define the message and send to UDP function. """
        command = [self.setpoint['altitude']*M2FEET,
                   math.radians(self.setpoint['heading'] + self.heading),
                   self.setpoint['velocity']*M2FEET,
                   self.setpoint['acceleration']*M2FEET,
                   math.radians(self.setpoint['gamma'])]
        self.send_command_udp(command, self.output_port)
