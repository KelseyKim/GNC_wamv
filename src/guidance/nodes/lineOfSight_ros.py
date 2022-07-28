#!/usr/bin/env python2

'''
x_k, y_k = current waypoint in NED,
x_k0, y_k0 = previous waypoint, in NED,
x, y = current position in NED
'''

import rospy
import pymap3d as pm

from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
from heron_msgs.msg import Course
from heron_msgs.msg import Drive
from geometry_msgs.msg import Vector3Stamped
from guidance.lineOfSight import LosGuidance
from math import pi, sqrt

from std_msgs.msg import Float32


class LineOfSightWrapper:
    def __init__(self):

        rospy.init_node("line_of_sight_node")

        self.desired_speed = rospy.get_param('/desired_speed')
        self.switch_radius = rospy.get_param('/switch_radius')

        self.base_lat = rospy.get_param('/base/latitude')
        self.base_lon = rospy.get_param('/base/longitude')
        self.base_alt = rospy.get_param('/base/altitude')

        self.los = LosGuidance()

        self.cmdCourse = Course()
        self.cmdCourse.speed = self.desired_speed

        self.cmdDrive = Drive()
        self.cmdDrive.left = 0
        self.cmdDrive.right = 0

        self.yaw = 0

        self.lat_desired = 0
        self.lon_desired = 0

        self.x_k = 0
        self.y_k = 0

        self.x_k0 = 0
        self.y_k0 = 0

        self.x = 0
        self.y = 0

        self.psi_los = 0

        self.r_los = rospy.get_param('/r_los')

        rospy.Subscriber('/imu/rpy', Vector3Stamped, self.rpy_callback)

        rospy.Subscriber('/desired/gps_coords', NavSatFix, self.goal_callback)
        rospy.Subscriber('/odometry/filtered', Odometry, self.update_pos)

        self.cmdCoursePub = rospy.Publisher('/cmd_course', Course, queue_size=10)
        self.cmdDrivePub = rospy.Publisher('/cmd_drive', Drive, queue_size=10)

        self.testPub = rospy.Publisher('/TESTING', Float32, queue_size=10)

    def rpy_callback(self, data):
        self.yaw = data.vector.z

    def publish_cmd(self):

        # self.testPub.publish(self.los.calc_los_psi(self.x_k, self.y_k, self.x_k0, self.y_k0, self.x, self.y, self.r_los))
        # self.testPub.publish(self.los.calc_los_psi(self.x_k, self.y_k, self.x_k0, self.y_k0, self.x, self.y, self.r_los))
        self.testPub.publish(self.y)

        self.cmdCourse.yaw = pi/2 - self.los.calc_los_psi(self.x_k, self.y_k, self.x_k0, self.y_k0, self.x, self.y, self.r_los)

        if(self.cmdCourse.yaw < -1*pi):
            self.cmdCourse.yaw = self.cmdCourse.yaw + 2*pi
        if(self.cmdCourse.yaw > pi):
            self.cmdCourse.yaw = self.cmdCourse.yaw - 2*pi

        if(sqrt((self.x_k - self.x)**2 + (self.y_k - self.y)**2) > self.switch_radius):
            self.cmdCoursePub.publish(self.cmdCourse)
        else:
            self.cmdDrivePub.publish(self.cmdDrive)


    def goal_callback(self, data):
        self.lat_desired = data.latitude
        self.lon_desired = data.longitude

        ned_coords = pm.geodetic2ned(self.lat_desired, self.lon_desired, 0, self.base_lat, self.base_lon, self.base_alt)

        if((self.x_k != ned_coords[0]) or (self.y_k != ned_coords[1])):
            self.x_k0 = self.x_k
            self.y_k0 = self.y_k

        self.x_k = ned_coords[0]
        self.y_k = ned_coords[1]

        self.publish_cmd()

    def update_pos(self, data):
        self.x = data.pose.pose.position.y
        self.y = data.pose.pose.position.x

        self.publish_cmd()



if __name__=='__main__':

    losWrapper = LineOfSightWrapper()
    rospy.spin()
