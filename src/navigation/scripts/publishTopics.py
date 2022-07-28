#!/usr/bin/env python2

import rospy
import numpy as np
from math import pi
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from navigation.transform import Transformations

from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Wrench
from sensor_msgs.msg import Imu
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry

class TopicPublisher:
    def __init__(self):
        rospy.init_node("tuning_topics")

        self.trans = Transformations()
        self.base_lat = 21.3107514071
        self.base_lon = -157.889371362

        self.wrench = Wrench()
        self.wrench.force.x = 0
        self.wrench.force.y = 0
        self.wrench.force.z = 0
        self.wrench.torque.x = 0
        self.wrench.torque.y = 0
        self.wrench.torque.z = 0

        self.gpsNed = Vector3()
        self.gpsNed.x = 0
        self.gpsNed.y = 0
        self.gpsNed.z = 0

        self.imu = Imu()
        self.imu.orientation.x = 0
        self.imu.orientation.y = 0
        self.imu.orientation.z = 0
        self.imu.orientation.w = 1
        self.imu.angular_velocity.x = 0
        self.imu.angular_velocity.y = 0
        self.imu.angular_velocity.z = 0
        self.imu.linear_acceleration.x = 0
        self.imu.linear_acceleration.y = 0
        self.imu.linear_acceleration.z = 0

        self.gt = Odometry()
        self.gt.pose.pose.position.x = 0
        self.gt.pose.pose.position.y = 0
        self.gt.pose.pose.position.z = 0
        self.gt.pose.pose.orientation.x = 0
        self.gt.pose.pose.orientation.y = 0
        self.gt.pose.pose.orientation.z = 0
        self.gt.pose.pose.orientation.w = 1
        self.gt.twist.twist.linear.x = 0
        self.gt.twist.twist.linear.y = 0
        self.gt.twist.twist.linear.z = 0
        self.gt.twist.twist.angular.x = 0
        self.gt.twist.twist.angular.y = 0
        self.gt.twist.twist.angular.z = 0


        rospy.Subscriber("/kalman/wrench", Wrench, self.update_wrench)
        rospy.Subscriber("/wamv/sensors/imu/imu/data", Imu, self.update_imu)
        rospy.Subscriber("/wamv/sensors/gps/gps/fix", NavSatFix, self.update_gps)
        rospy.Subscriber("/wamv/sensors/position/p3d_wamv", Odometry, self.update_gt)

        self.wrenchPub = rospy.Publisher("/tuning/wrench", Wrench, queue_size=10)
        self.gpsPub = rospy.Publisher("/tuning/gps_ned", Vector3, queue_size=10)
        self.imuPub = rospy.Publisher("/tuning/imu", Imu, queue_size=10)
        self.gtPub = rospy.Publisher("/tuning/gt", Odometry, queue_size=10)


    def update_wrench(self, data):
        self.wrench.force.x = data.force.x
        self.wrench.force.y = data.force.y
        self.wrench.torque.z = data.torque.z

    def update_gps(self, data):
        lat = data.latitude
        lon = data.longitude
        alt = data.altitude

        ned_coords = self.trans.lla_to_ned(lat, lon, self.base_lat, self.base_lon)

        self.gpsNed.x = ned_coords[0]
        self.gpsNed.y = ned_coords[1]
        self.gpsNed.z = 0

    def update_imu(self, data):

        self.imu.orientation.x = data.orientation.x
        self.imu.orientation.y = data.orientation.y
        self.imu.orientation.z = data.orientation.z
        self.imu.orientation.w = data.orientation.w

        self.imu.angular_velocity.x = data.angular_velocity.x
        self.imu.angular_velocity.y = data.angular_velocity.y
        self.imu.angular_velocity.z = data.angular_velocity.z

        self.imu.linear_acceleration.x = data.linear_acceleration.x
        self.imu.linear_acceleration.y = data.linear_acceleration.y
        self.imu.linear_acceleration.z = data.linear_acceleration.z

    def update_gt(self, data):
        # GT POSE NOT IN NED, TWIST NOT IN BODY!!
        self.gt.pose.pose.position.x = data.pose.pose.position.x
        self.gt.pose.pose.position.y = data.pose.pose.position.y
        self.gt.pose.pose.position.z = data.pose.pose.position.z
        self.gt.pose.pose.orientation.x = data.pose.pose.orientation.x
        self.gt.pose.pose.orientation.y = data.pose.pose.orientation.y
        self.gt.pose.pose.orientation.z = data.pose.pose.orientation.z
        self.gt.pose.pose.orientation.w = data.pose.pose.orientation.w
        self.gt.twist.twist.linear.x = data.twist.twist.linear.x
        self.gt.twist.twist.linear.y = data.twist.twist.linear.y
        self.gt.twist.twist.linear.z = data.twist.twist.linear.z
        self.gt.twist.twist.angular.x = data.twist.twist.angular.x
        self.gt.twist.twist.angular.y = data.twist.twist.angular.y
        self.gt.twist.twist.angular.z = data.twist.twist.angular.z

    def publishTopics(self):
        self.wrenchPub.publish(self.wrench)
        self.gpsPub.publish(self.gpsNed)
        self.imuPub.publish(self.imu)
        self.gtPub.publish(self.gt)


if __name__ == "__main__":
    tp = TopicPublisher()
    rate = rospy.Rate(5)

    while not rospy.is_shutdown():
        tp.publishTopics()
        rate.sleep()
