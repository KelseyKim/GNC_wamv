#! /usr/bin/env python2

import rospy
import numpy as np
from math import cos, sin

from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler

class enuToBodyTwist:
    def __init__(self):
        rospy.init_node("twist_enu_to_body")

        self.psi = 0
        self.R_enu2body = np.zeros((3,3), dtype = float)

        self.enuOdom = Odometry()
        self.enuOdom.header.frame_id = "ENU"
        self.enuOdom.child_frame_id = "body"

        self.nedOdom = Odometry()
        self.nedOdom.header.frame_id = "NED"
        self.nedOdom.child_frame_id = "body"

        self.bodyOdom = Odometry()
        self.bodyOdom.header.frame_id = "body"

        rospy.Subscriber("/odometry/filtered", Odometry, self.odomFiltCallback)

    def odomFiltCallback(self, odomMsg):
        self.enuOdom.pose.pose.position = odomMsg.pose.pose.position
        self.enuOdom.pose.pose.orientation = odomMsg.pose.pose.orientation
        self.enuOdom.twist.twist.linear = odomMsg.twist.twist.linear
        self.enuOdom.twist.twist.angular = odomMsg.twist.twist.angular


        quat = [self.enuOdom.pose.pose.orientation.x, self.enuOdom.pose.pose.orientation.y, self.enuOdom.pose.pose.orientation.z, self.enuOdom.pose.pose.orientation.w]
        eul = euler_from_quaternion(quat)
        self.psi = -eul[2]

        self.enu2body()

    def enu2body(self):
        # Convert from enu to ned
        self.nedOdom.twist.twist.linear.x = self.enuOdom.twist.twist.linear.y
        self.nedOdom.twist.twist.linear.y = self.enuOdom.twist.twist.linear.x
        self.nedOdom.twist.twist.angular.x = self.enuOdom.twist.twist.angular.y
        self.nedOdom.twist.twist.angular.y = self.enuOdom.twist.twist.angular.x
        self.nedOdom.twist.twist.angular.z = -1*self.enuOdom.twist.twist.angular.z

        # Convert from ned to body: x forward, y right, z down
        self.R_enu2body = np.array[[cos(self.psi)]]
