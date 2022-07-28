#!/usr/bin/env python

'''
ECEF = NED orientation, located at north pole
NWU = NWU orientation, located at north pole
down = p = body fixed coordinate axes: forward, right, down
up = body fixed coordinate axes: forward, left, up

lla-ecef conversions: https://microem.ru/files/2012/08/GPS.G1-X-00006.pdf
latlon2xy, xy2latlon functions taken from https://wiki.nps.edu/display/RC/Local+Coordinate+Frames

All functions return coordinates in vertical vectors: [x, y, z]' or [lat, lon, alt]'

'''


import rospy
import math
import numpy as np
import tf

from tf.transformations import euler_from_quaternion, quaternion_from_euler
from math import cos, sin, pi, pow

from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from sensor_msgs.msg import NavSatFix


class Transformations:
    def __init__(self):

        self.base_lat = 0
        self.base_lon = 0
        self.base_alt = 0

        self.R_psi = np.zeros((3,3))

        self.position_ned = np.zeros((3,1))

    def enu_to_ned(self, x_enu, y_enu, z_enu):
        return [y_enu, x_enu, -1*z_enu]

    # R^n_b = R_z,psi (rotation mat from body to ned = rotation psi about z)
    # Returns [x, y, z]
    def ned_to_body(self, x_ned, y_ned, z_ned, psi):
        self.R_psi = np.array([
        [cos(psi), -sin(psi), 0],
        [sin(psi), cos(psi), 0],
        [0, 0, 1]
        ])
        body_coords = np.matmul(self.R_psi.T, np.array([[x_ned, y_ned, z_ned]]).T)
        return [body_coords[0,0], body_coords[1,0], body_coords[2,0]]

    def body_to_ned(self, x_b, y_b, z_b, psi):
        self.R_psi = np.array([
        [cos(psi), -sin(psi), 0],
        [sin(psi), cos(psi), 0],
        [0, 0, 1]
        ])
        return np.matmul(self.R_psi, np.array([[x_b, y_b, z_b]]).T)

    # Takes in lat, lon, base_lat, base_lon
    # Returns [Northing, Easting]
    def lla_to_ned(self, lat, lon, lat0, lon0):
        x = (lat - lat0)*self.mdeglat(lat0)
        y = (lon - lon0)*self.mdeglon(lat0)
        return [x,y]

    # Takes in Northing, Easting, base_lat, base_lon
    # Returns [lat, lon]
    def ned_to_lla(self, x, y, lat0, lon0):
        lon = y/self.mdeglon(lat0) + lon0
        lat = x/self.mdeglat(lat0) + lat0
        return [lat, lon]

    def mdeglon(self, lat0):
        lat0rad = math.radians(lat0)
        return (111415.13*cos(lat0rad) - 94.55*cos(3.0*lat0rad) - 0.12*cos(5.0*lat0rad))

    def mdeglat(self, lat0):
        lat0rad = math.radians(lat0)
        return (111132.09 - 566.05*cos(2.0*lat0rad) + 1.2*cos(4*lat0rad) - .002*cos(6*lat0rad))


    def body_to_ned_quat(self, x_b, y_b, z_b, x_quat, y_quat, z_quat, w_quat):
        self.R_psi = np.array([
        [1 - 2*(y_quat**2 + z_quat**2), 2*(x_quat*y_quat - z_quat*w_quat), 2*(x_quat*z_quat + y_quat*w_quat)],
        [2*(x_quat*y_quat + z_quat*w_quat), 1 - 2*(x_quat**2 + z_quat**2), 2*(y_quat*z_quat - x_quat*w_quat)],
        [2*(x_quat*z_quat - y_quat*w_quat), 2*(y_quat*z_quat + x_quat*w_quat), 1 - 2*(x_quat**2 + y_quat**2)]
        ])
        return np.matmul(self.R_psi, np.array([[x_b, y_b, z_b]]).T)
