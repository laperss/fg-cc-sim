#! /usr/bin/python
""" Class Positioner for converting between local and global reference frames. 

Author: Linnea Persson, laperss@kth.se

This module contains a class used for converting between global (GPS) 
coordinates and a user defined local coordinate system. The local 
coordinate system is defined by: 
    1) Origin in global coordinates (latitude, longitude) in degrees
    2) Direction of the x-axis, in degrees. 

The class 'Positioner' uses the utm package to first convert degrees to UTM, 
then convert UTM to the local system. 
The module was written as a part of the fg-sim package (https://github.com/laperss/fg-sim)

Usage: 
    from positioner import Positioner

    ORIGIN = (37.427729, -122.357193)
    HEADING = 297.9
    pos = Positioner(ORIGIN, HEADING)

    LAT, LON = pos.get_global_position(5, 10)
    X, Y = pos.get_local_position(37.426855, -122.35790)

"""
from __future__ import print_function
import math
import utm
import numpy as np


class Positioner(object):
    """ Deals with convertion between local and global positioning
        relative an origin and heading. """
    def __init__(self, origin=(37.427729, -122.357193), heading=297.9):
        self.ORIGIN = origin
        self.RUNWAY_HEADING = heading

        (self.EAST_0, self.NORTH_0,
         self.ZONE, self.LETTER) = utm.from_latlon(self.ORIGIN[0], self.ORIGIN[1])

        cos = math.cos(math.radians(self.RUNWAY_HEADING))
        sin = math.sin(math.radians(self.RUNWAY_HEADING))
        self.ROTATION = np.array([[cos, -sin],
                                  [sin, cos]])
        self.ROTATION_INV = np.linalg.inv(self.ROTATION)

    def get_local_position(self, lat, lon):
        """Get the local position from origin defined at east0, north0"""

        if (type(lat) == np.ndarray) or (type(lon) == np.ndarray):
            d_east = []
            d_north = []
            for lt, ln in zip(lat, lon):
                east, north, _, _ = utm.from_latlon(lt, ln)
                d_east.append(east - self.EAST_0)
                d_north.append(north - self.NORTH_0)

        elif lat <= -80 or lat >= 84:
            print("Latitude not feasible: ", lat)
            return
        else:
            east, north, _, _ = utm.from_latlon(lat, lon)
            d_east = east - self.EAST_0
            d_north = north - self.NORTH_0

        pos = np.array([d_east, d_north])
        y, x = np.dot(self.ROTATION, pos)
        return x, y

    def get_global_position(self, x, y):
        """Get the global position from origin defined at (0,0)"""
        pos = np.array([y, x])
        d_east, d_north = np.dot(self.ROTATION_INV, pos)
        east_1 = d_east + self.EAST_0
        north_1 = d_north + self.NORTH_0
        latitude, longitude = utm.to_latlon(east_1, north_1, self.ZONE, self.LETTER)
        return latitude, longitude

    def get_relative_distance(self, lat1, lon1, lat2, lon2):
        """ Calculate position from pos1 to pos2 """
        east_1, north_1, _, _ = utm.from_latlon(lat1, lon1)
        east_2, north_2, _, _ = utm.from_latlon(lat2, lon2)
        d_east = east_2 - east_1
        d_north = north_2 - north_1
        pos = np.array([d_east, d_north])
        deltay, deltax = np.dot(self.ROTATION, pos)
        return deltax, deltay

    def get_origin(self):
        """ Returns origin from which the reference frame is defined. """
        return self.ORIGIN

    def get_runway_heading(self):
        """ Returns heading from which the reference frame is defined. """
        return self.RUNWAY_HEADING

    def set_origin(self, origin):
        """ Set the origin from which the reference frame is defined. """
        self.ORIGIN = origin

    def set_heading(self, heading):
        """ Set the origin from which the reference frame is defined. """
        self.RUNWAY_HEADING = heading

    def compute_rotation_matrices(self):
        """ Recompute the rotation matrices relative to the desired heading """
        cos = math.cos(math.radians(self.RUNWAY_HEADING))
        sin = math.sin(math.radians(self.RUNWAY_HEADING))
        self.ROTATION = np.array([[cos, -sin],
                                  [sin, cos]])
        self.ROTATION_INV = np.linalg.inv(self.ROTATION)

    def compute_utm_origin(self):
        """ Recompute the UTM coordinate, zone and letter. """
        (self.EAST_0, self.NORTH_0,
         self.ZONE, self.LETTER) = utm.from_latlon(self.ORIGIN[0], self.ORIGIN[1])


if __name__ == "__main__":
    ORIGIN = (42.186702238384, -71.00457277413)
    #ORIGIN = (37.427729, -122.357193)
    HEADING = 199.67
    poser = Positioner(ORIGIN, HEADING)

    print("\nExample calculations for positioner module.")
    print("- Origin: (%2.5f, %2.5f) deg" %(ORIGIN[0], ORIGIN[1]))
    print("- Heading: %2.2f deg\n" %(HEADING))

    print("Test cases: \n----------------")
    COORD = (42.3768549, -71.0047138267)
    COORD = (42.178040, -71.008968) #(1027.47771, 42.22683)
    COORD = (42.178200, -71.008896) #(1008.73757, 42.16626)



    #COORD = (42.177604, -71.009115) #_posuav: (1077.26695, 38.53340) m
    #COORD = (42.177591, -71.009128) #_posugv: (1078.97527, 39.09908) 
    #COORD = (42.180309, -71.007812) #POSUGV: (758.21503, 30.78734) m
    #COORD = (42.181332, -71.007294) #POSUAV: (636.89866, 25.88235)

    #COORD = (42.180351, -71.007797) #POSUGV-1: (753.39704, 31.07740) m som att den ar fosenad
    #COORD = (42.181362, -71.007286) #POSUAV-1: (633.52773, 26.30243) 



    x, y = poser.get_local_position(COORD[0], COORD[1])
    print("Global: (%2.5f, %2.5f) deg \t->\t " %(COORD[0], COORD[1]), end='')
    print("Local:  (%2.5f, %2.5f) m" %(x, y))

    COORD = (-200, 10)
    LAT, LON = poser.get_global_position(COORD[0], COORD[1])
    print("Local:  (%2.5f, %2.5f) m \t->\t " %(COORD[0], COORD[1]), end='')
    print("Global: (%2.10f, %2.10f) deg" %(LAT, LON))

    xpos, ypos = poser.get_local_position(LAT, LON)
    print("Global: (%2.5f, %2.5f) deg \t->\t " %(LAT, LON), end='')
    print("Local:  (%2.5f, %2.5f) m" %(xpos, ypos))
