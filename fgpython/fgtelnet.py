#! /usr/bin/python
""" Send commands from Python to FlightGear over Telnet. 

Author: Linnea Persson, laperss@kth.se 

This is suitable for sending individual commands.

Usage:
    uav_tnet = fgtnet.FlightGearVehicle('localhost', 6500)
    ugv_tnet = fgtnet.FlightGearVehicle('localhost', 6600)
Send some commands:
    uav_tnet.landing_mode()
    uav_tnet.pause()

"""
from __future__ import print_function
import time
import socket
#from string import split
from telnetlib import Telnet


def is_number(string):
    """ Return true if the string is a number """
    try:
        float(string)
        return True
    except ValueError:
        return False


class FGTelnet(Telnet):
    """ Class for Telnet interface of Flightgear

        From the FlightGear wiki:
        cd - change directory
        data - switch to data mode
        dump - dump to xml
        get - show a property value
        help - show available commands
        ls - list directories and paramaters
        pwd - display current path
        quit - terminate connection
        run - run a built in command
        set - set value of property
    """

    def __init__(self, host, port):
        Telnet.__init__(self, host, port)
        self.timeout = 5
        # Go into data mode
        self.data()

    def cd(self, dir):
        """ Change directory"""
        self._put('cd ' + dir)
        self._get()

    def data(self):
        """Switch to data mode."""
        self._put('data')

    def dump(self):
        """Dump to XML"""
        self._put('dump')
        return self._get()

    def get(self, prop):
        """Get value of a FlightGear property """
        self._put('get %s' % prop)
        return self._get()

    def help(self):
        """Return available commands """
        self._put('help')
        help = self._get().split('\n')[2:-1]
        dictionary = dict(zip([split(line, '  ', 1)[0] for line in help],
                              [split(line, '  ', 1)[1] for line in help]))
        return dictionary

    def ls(self, dir=None):
        """Returns a list of properties """
        if dir == None:
            self._put('ls')
        else:
            self._put('ls %s' % dir)
        return self._get()

    def pwd(self):
        """Display current path """
        self._put('pwd')
        return self._get()

    def quit(self):
        """Terminate connection """
        self._put('quit')
        self.close()
        return

    def run(self):
        pass

    def set(self, prop, value):
        """Set FlightGear property to value """
        self._put('set %s %s' % (prop, value))

    def _put(self, cmd):
        """ Send command to telnet """
        Telnet.write(self, (cmd + '\r\n').encode())
        return

    def _get(self):
        """ Get response from telnet """
        resp = Telnet.read_until(self, '\n', self.timeout)
        return resp


class FGTelnetConnection:
    """Interface for sending commands from Python to FlightGear """
    telnet = False

    def __init__(self, host='localhost', port=5500):
        self.host = host
        self.port = port

    def telnet_connect(self):
        """ Connect to the telnet port """
        while not self.telnet:
            try:
                self.telnet = FGTelnet(self.host, self.port)
            except (socket.error):
                time.sleep(0.4)
        print("Successfully connected to '%s %s'" % (self.host, self.port))

    def __del__(self):
        """ Exit safely """
        self.close()

    def get(self, key):
        """Get value from a FlightGear property """
        value = self.telnet.get(key)
        if is_number(value):
            value = float(value)
            return value
        elif value.strip() == "true":
            return True
        elif value.strip() == "false":
            return False
        else:
            return value

    def set(self, key, value):
        """Set a FlightGear property value."""
        if self.telnet:
            self.telnet.set(key, value)

    def close(self):
        """Close the telnet connection to FlightGear."""
        if self.telnet:
            self.telnet.quit()
        self.telnet = False

    def pause(self):
        """ Pause simulation """
        if self.telnet:
            self.telnet.set("/sim/freeze/master", 1)
            self.telnet.set("/sim/freeze/clock", 1)

    def resume(self):
        """ Resume simulation """
        if self.telnet:
            self.telnet.set("/sim/freeze/master", 0)
            self.telnet.set("/sim/freeze/clock", 0)

    def reset(self):
        """ Resume simulation """
        if self.telnet:
            self.telnet.set("/fdm/jsbsim/simulation/reset", )

    def toggle_tecs(self, mode):
        """ Toggle Total Energy Control System (TECS) on or off """
        if self.telnet:
            self.telnet.set("/fdm/jsbsim/ap/tecs", mode)

    def control_heading(self):
        """ Control aircraft altitude """
        if self.telnet:
            self.telnet.set("/fdm/jsbsim/ap/heading_hold", 1)
            self.telnet.set("/fdm/jsbsim/ap/attitude_hold", 0)

    def wings_level(self):
        """ Control aircraft altitude """
        if self.telnet:
            self.telnet.set("/fdm/jsbsim/ap/heading_hold", 0)
            self.telnet.set("/fdm/jsbsim/ap/attitude_hold", 1)

    def control_altitude(self):
        """ Control aircraft altitude """
        if self.telnet:
            self.telnet.set("/fdm/jsbsim/ap/altitude_hold", 1)
            self.telnet.set("/fdm/jsbsim/ap/gamma_hold", 0)

    def control_flight_path(self):
        """ Control aircraft flight path """
        if self.telnet:
            self.telnet.set("/fdm/jsbsim/ap/altitude_hold", 0)
            self.telnet.set("/fdm/jsbsim/ap/gamma_hold", 1)

    def control_velocity(self):
        """ Control aircraft by desired velocity """
        if self.telnet:
            self.telnet.set("/fdm/jsbsim/ap/velocity_hold", 1)
            self.telnet.set("/fdm/jsbsim/ap/acceleration_hold", 0)

    def control_acceleration(self):
        """ Control aircraft by desired velocity """
        print("Control acceleration")
        if self.telnet:
            self.telnet.set("/fdm/jsbsim/ap/velocity_hold", 0)
            self.telnet.set("/fdm/jsbsim/ap/acceleration_hold", 1)

    def landing_mode(self):
        """ Land aircraft on top of ground vehicle """
        self.control_acceleration()
        self.control_flight_path()
        self.control_heading()

    def align_mode(self):
        """ Align the vehicles in x- and y """
        self.control_velocity()
        self.control_altitude()
        self.control_heading()

    def view_next(self):
        """ Switch to the next view in FlightGear """
        if self.telnet:
            self.telnet.set("/command/view/next", "true")
