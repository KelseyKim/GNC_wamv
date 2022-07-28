#!/usr/bin/env python2

''' Variables:
x_k, y_k: desired waypoint
x_k0, y_k0: previous desired waypoint
x_pos, y_pos: vehicle's current position
r_los: line of sight radius (specified by user)
psi_los: desired heading in rad, positive clockwise, 0 corresponding to North
'''


import rospy
from math import sin, cos, atan2, sqrt

class LosGuidance:
    def __init__(self):
        self.r_los = 25
        self.psi_los = 0

    def calc_los_psi(self, x_k, y_k, x_k0, y_k0, x, y, r_los):
        self.r_los = r_los

        x_los = 1j
        y_los = 1j

        while(isinstance(x_los, complex) or isinstance(y_los, complex)):
            # Calculate a, b, c
            a = -(x_k - x_k0)**2 - (y_k - y_k0)**2
            b = 2*x*(x_k-x_k0)**2 + 2*(y_k-y_k0)*(y*(x_k-x_k0)-y_k0*x_k+y_k*x_k0)
            c = (self.r_los**2-x**2)*(x_k-x_k0)**2 - (y*(x_k-x_k0)-y_k0*x_k+y_k*x_k0)**2

            if(x_k >= x):
                x_los = (-b - sqrt(b**2 - 4*a*c))/(2*a)
            else:
                x_los = (-b + sqrt(b**2 - 4*a*c))/(2*a)

            if(y_k >= y):
                y_los = y + sqrt(self.r_los**2-(x_los-x)**2)
            else:
                y_los = y - sqrt(self.r_los**2-(x_los-x)**2)

            self.r_los += 1

        self.psi_los = atan2(y_los-y, x_los-x)


        return self.psi_los
