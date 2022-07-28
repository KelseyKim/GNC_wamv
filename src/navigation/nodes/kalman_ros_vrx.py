#!/usr/bin/env python2

import rospy
import numpy as np
from math import pi
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from navigation.kalman_vrx import KalmanFilterVrx
from navigation.transform import Transformations

from std_msgs.msg import Float32
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import MultiArrayDimension
from geometry_msgs.msg import Wrench
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import Imu
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry

class KalmanFilterWrapper:
    def __init__(self):
        rospy.init_node("kalman_filter_node")

        self.kf = KalmanFilterVrx()
        self.trans = Transformations()

        self.import_parameters()
        self.kf.set_dynamic_model()
        self.kf.set_kalman_matrices()

        self.nedFilt = Odometry()
        self.nedFilt.header.frame_id = 'ned'
        self.nedFilt.header.stamp = rospy.Time.now()

        self.llaFilt = NavSatFix()
        self.llaFilt.header.frame_id = 'lla'
        self.llaFilt.header.stamp = rospy.Time.now()

        self.nedRpy = Vector3()

        self.gpsNed = Vector3()

        rospy.Subscriber("/wrench", Wrench, self.update_wrench)
        rospy.Subscriber("/wamv/sensors/imu/imu/data", Imu, self.update_imu)
        rospy.Subscriber("/wamv/sensors/gps/gps/fix", NavSatFix, self.update_gps)

        self.nedFiltPub = rospy.Publisher("/kalman/filtered_ned", Odometry, queue_size=10)
        self.llaFiltPub = rospy.Publisher("/kalman/filtered_gps", NavSatFix, queue_size=10)
        self.nedRpyPub = rospy.Publisher("/kalman/rpy_ned", Vector3, queue_size=10)

        self.gpsNedPub = rospy.Publisher("/kalman/gps_ned", Vector3, queue_size=10)

    def import_parameters(self):
        # Import sensor offsets
        self.kf.xp = rospy.get_param('/gps/x')
        self.kf.yp = rospy.get_param('/gps/y')

        # Import base station coordinates
        self.kf.base_lat = rospy.get_param('/base/latitude')
        self.kf.base_lon = rospy.get_param('/base/longitude')
        self.kf.base_alt = rospy.get_param('/base/altitude')

        # Import time step
        self.kf.dt = rospy.get_param('/dt')

        # Import mass matrix parameters
        self.kf.m = rospy.get_param('/massMatrix/m')
        self.kf.xg = rospy.get_param('/massMatrix/xg')
        self.kf.Xudot = rospy.get_param('/massMatrix/Xudot')
        self.kf.Yvdot = rospy.get_param('/massMatrix/Yvdot')
        self.kf.Yrdot = rospy.get_param('/massMatrix/Yrdot')
        self.kf.Iz = rospy.get_param('/massMatrix/Iz')
        self.kf.Nrdot = rospy.get_param('/massMatrix/Nrdot')

        # Import damping matrix parameters
        self.kf.Xu = rospy.get_param('/dampingMatrix/Xu')
        self.kf.Yv = rospy.get_param('/dampingMatrix/Yv')
        self.kf.Yr = rospy.get_param('/dampingMatrix/Yr')
        self.kf.Nv = rospy.get_param('dampingMatrix/Nv')
        self.kf.Nr = rospy.get_param('/dampingMatrix/Nr')

        # Import covariance matrices
        self.kf.Q = np.reshape(rospy.get_param('/processNoiseCovariance'), (8,8))
        self.kf.R = np.reshape(rospy.get_param('/measurementNoiseCovariance'), (6,6))

    def update_wrench(self, wrench):
        self.kf.u[0,0] = wrench.force.x
        self.kf.u[1,0] = wrench.force.y
        self.kf.u[2,0] = wrench.torque.z

    def update_gps(self, data):
        lat = data.latitude
        lon = data.longitude
        alt = data.altitude

        ned_coords = self.trans.lla_to_ned(lat, lon, self.kf.base_lat, self.kf.base_lon)
        self.kf.z[0,0] = ned_coords[0]
        self.kf.z[1,0] = ned_coords[1]

        self.gpsNed.x = ned_coords[0]
        self.gpsNed.y = ned_coords[1]
        self.gpsNed.z = 0


    def update_imu(self, data):
        quat = [data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w]
        roll, pitch, yaw = euler_from_quaternion(quat)
        yaw = -yaw + pi/2   #   CONVERT TO NED FROM ENU

        if yaw > pi:
            yaw = yaw - 2*pi
        if yaw < -pi:
            yaw = yaw + 2*pi

        self.kf.z[2,0] = yaw

        self.kf.z[3,0] = -data.angular_velocity.z
        self.kf.z[4,0] = data.linear_acceleration.x
        self.kf.z[5,0] = -data.linear_acceleration.y

    def publishFiltered(self):
        self.nedFilt.pose.pose.position.x = self.kf.X_k[0,0]
        self.nedFilt.pose.pose.position.y = self.kf.X_k[1,0]
        self.nedFilt.pose.pose.position.z = 0

        phi = 0
        theta = 0
        psi = self.kf.X_k[2,0]

        self.nedRpy.x = phi
        self.nedRpy.y = theta
        self.nedRpy.z = psi

        nedQuat = quaternion_from_euler(phi, theta, psi)

        self.nedFilt.pose.pose.orientation.x = nedQuat[0]
        self.nedFilt.pose.pose.orientation.y = nedQuat[1]
        self.nedFilt.pose.pose.orientation.z = nedQuat[2]
        self.nedFilt.pose.pose.orientation.w = nedQuat[3]

        self.nedFilt.pose.covariance[0] = self.kf.P_k[0,0]
        self.nedFilt.pose.covariance[7] = self.kf.P_k[1,1]
        self.nedFilt.pose.covariance[14] = 0
        self.nedFilt.pose.covariance[21] = 0
        self.nedFilt.pose.covariance[28] = 0
        self.nedFilt.pose.covariance[35] = self.kf.P_k[2,2]

        self.nedFilt.twist.twist.linear.x = self.kf.X_k[3,0]
        self.nedFilt.twist.twist.linear.y = self.kf.X_k[4,0]
        self.nedFilt.twist.twist.linear.z = 0

        self.nedFilt.twist.twist.angular.x = 0
        self.nedFilt.twist.twist.angular.y = 0
        self.nedFilt.twist.twist.angular.z = self.kf.X_k[5,0]

        self.nedFilt.twist.covariance[0] = self.kf.P_k[3,3]
        self.nedFilt.twist.covariance[7] = self.kf.P_k[4,4]
        self.nedFilt.twist.covariance[14] = 0
        self.nedFilt.twist.covariance[21] = 0
        self.nedFilt.twist.covariance[28] = 0
        self.nedFilt.twist.covariance[35] = self.kf.P_k[5,5]

        self.nedFilt.header.stamp = rospy.Time.now()

        self.nedFiltPub.publish(self.nedFilt)
        self.nedRpyPub.publish(self.nedRpy)

        self.gpsNedPub.publish(self.gpsNed)

        lla = self.trans.ned_to_lla(self.kf.X_k[0,0], self.kf.X_k[1,0], self.kf.base_lat, self.kf.base_lon)
        self.llaFilt.latitude = lla[0]
        self.llaFilt.longitude = lla[1]
        self.llaFilt.header.stamp = rospy.Time.now()

        self.llaFiltPub.publish(self.llaFilt)

        print(self.kf.P_k)


if __name__ == "__main__":
    kfw = KalmanFilterWrapper()
    rate = rospy.Rate(5)

    while not rospy.is_shutdown():
        kfw.kf.update_state()
        kfw.publishFiltered()
        rate.sleep()
