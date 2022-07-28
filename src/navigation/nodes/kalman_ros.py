#!/usr/bin/env python2

import rospy
import numpy as np
from math import pi
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from navigation.kalman import KalmanFilter
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

        self.kf = KalmanFilter()
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

        self.gtFilt = Odometry()
        self.gtFilt.header.frame_id = 'ned'
        self.gtFilt.header.stamp = rospy.Time.now()

        self.nedRpy = Vector3()

        self.gpsNedPort = Vector3()
        self.gpsNedStarboard = Vector3()

        rospy.Subscriber("/wrench", Wrench, self.update_wrench)
        rospy.Subscriber("/sensors/imu/data", Imu, self.update_imu)
        rospy.Subscriber("/sensors/portFix", NavSatFix, self.update_port_gps)
        rospy.Subscriber("/sensors/starboardFix", NavSatFix, self.update_starboard_gps)
        rospy.Subscriber("/sensors/gpsHeading", Float32, self.update_diff_gps_heading)

        self.nedFiltPub = rospy.Publisher("/kalman/filtered_ned", Odometry, queue_size=10)
        self.llaFiltPub = rospy.Publisher("/kalman/filtered_gps", NavSatFix, queue_size=10)
        self.nedRpyPub = rospy.Publisher("/kalman/rpy_ned", Vector3, queue_size=10)

        self.gpsNedPortPub = rospy.Publisher("/kalman/gps_port_ned", Vector3, queue_size=10)
        self.gpsNedStarboardPub = rospy.Publisher("/kalman/gps_starboard_ned", Vector3, queue_size=10)

        # # Publish Kalman Gain
        # self.K = [0] * 48
        # self.K_msg = Float32MultiArray()
        # self.K_msg.layout.dim = [MultiArrayDimension(), MultiArrayDimension()]
        # self.K_msg.layout.data_offset = 0
        # self.K_msg.layout.dim[0].label = "channels"
        # self.K_msg.layout.dim[0].size = 8
        # self.K_msg.layout.dim[0].stride = 48
        # # self.K_msg.layout.dim[1].data_offset = 0
        # self.K_msg.layout.dim[1].label = "samples"
        # self.K_msg.layout.dim[1].size = 6
        # self.K_msg.layout.dim[1].stride = 6
        # self.KPub = rospy.Publisher("/kalman/kalman_gain", Float32MultiArray, queue_size=10)

    def import_parameters(self):
        # Import sensor offsets
        self.kf.xp_port = rospy.get_param('/portGps/x')
        self.kf.yp_port = rospy.get_param('/portGps/y')
        self.kf.xp_starboard = rospy.get_param('/starboardGps/x')
        self.kf.yp_starboard = rospy.get_param('/starboardGps/y')

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
        self.kf.R = np.reshape(rospy.get_param('/measurementNoiseCovariance'), (8,8))

    def update_wrench(self, wrench):
        self.kf.u[0,0] = wrench.force.x
        self.kf.u[1,0] = wrench.force.y
        self.kf.u[2,0] = wrench.torque.z

    def update_port_gps(self, port_data):
        lat = port_data.latitude
        lon = port_data.longitude
        alt = port_data.altitude

        ned_coords = self.trans.lla_to_ned(lat, lon, self.kf.base_lat, self.kf.base_lon)
        self.kf.z[0,0] = ned_coords[0]
        self.kf.z[1,0] = ned_coords[1]# - self.yOffsetPort

        self.gpsNedPort.x = ned_coords[0]
        self.gpsNedPort.y = ned_coords[1]
        self.gpsNedPort.z = 0


    def update_starboard_gps(self, starboard_data):
        lat = starboard_data.latitude
        lon = starboard_data.longitude
        alt = starboard_data.altitude

        ned_coords = self.trans.lla_to_ned(lat, lon, self.kf.base_lat, self.kf.base_lon)
        self.kf.z[2,0] = ned_coords[0]
        self.kf.z[3,0] = ned_coords[1]# - self.yOffsetStarboard

        self.gpsNedStarboard.x = ned_coords[0]
        self.gpsNedStarboard.y = ned_coords[1]
        self.gpsNedStarboard.z = 0

    def update_diff_gps_heading(self, data):
        self.kf.z[4,0] = data.data

    def update_imu(self, imu_data):
        self.kf.z[5,0] = -imu_data.angular_velocity.z       # -, assuming gyro axes z upwards
        self.kf.z[6,0] = imu_data.linear_acceleration.x
        self.kf.z[7,0] = -imu_data.linear_acceleration.y

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

        self.nedFilt.twist.twist.linear.x = self.kf.X_k[3,0]
        self.nedFilt.twist.twist.linear.y = self.kf.X_k[4,0]
        self.nedFilt.twist.twist.linear.z = 0

        self.nedFilt.twist.twist.angular.x = 0
        self.nedFilt.twist.twist.angular.y = 0
        self.nedFilt.twist.twist.angular.z = self.kf.X_k[5,0]
        self.nedFilt.header.stamp = rospy.Time.now()

        self.nedFiltPub.publish(self.nedFilt)
        self.nedRpyPub.publish(self.nedRpy)

        self.gpsNedPortPub.publish(self.gpsNedPort)
        self.gpsNedStarboardPub.publish(self.gpsNedStarboard)


        lla = self.trans.ned_to_lla(self.kf.X_k[0,0], self.kf.X_k[1,0], self.kf.base_lat, self.kf.base_lon)
        self.llaFilt.latitude = lla[0]
        self.llaFilt.longitude = lla[1]
        self.llaFilt.header.stamp = rospy.Time.now()

        self.llaFiltPub.publish(self.llaFilt)

        # # Publish Kalman Gain
        # count = 0
        # for i in range(8):
        #     for j in range(6):
        #         self.K[count] = float(self.kf.K[i,j])
        #         count = count + 1
        # self.K_msg.data = self.K
        # self.KPub.publish(self.K_msg)


if __name__ == "__main__":
    kfw = KalmanFilterWrapper()
    rate = rospy.Rate(5)

    while not rospy.is_shutdown():
        kfw.kf.update_state()
        kfw.publishFiltered()
        rate.sleep()
