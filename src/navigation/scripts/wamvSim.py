#! /usr/bin/env python2

import rospy
import math

from std_msgs.msg import Int32
from sensor_msgs.msg import Imu
from sensor_msgs.msg import NavSatFix

class wamvSim:
    def __init__(self):
        rospy.init_node("wamv_simulation")

        self.portThrust = 500
        self.starboardThrust = 500

        self.imu = Imu()
        self.imu.header.stamp = rospy.Time.now()
        self.imu.header.frame_id = "imu_ned"
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


        self.portGps = NavSatFix()
        self.portGps.header.stamp = rospy.Time.now()
        self.portGps.header.frame_id = "port_gps"
        self.portGps.latitude = 21.311773
        self.portGps.longitude = -157.889313
        self.portGps.altitude = 0

        self.starboardGps = NavSatFix()
        self.starboardGps.header.stamp = rospy.Time.now()
        self.starboardGps.header.frame_id = "starboard_gps"
        self.starboardGps.latitude = 21.311773
        self.starboardGps.longitude = -157.889293
        self.starboardGps.altitude = 0

        self.portThrustPub = rospy.Publisher("/q3_thruster_input", Int32, queue_size=10)
        self.starboardThrustPub = rospy.Publisher("/q4_thruster_input", Int32, queue_size=10)

        self.imuPub = rospy.Publisher("/sensors/imu/data", Imu, queue_size=10)
        self.portGpsPub = rospy.Publisher("/sensors/portFix", NavSatFix, queue_size=10)
        self.starboardGpsPub = rospy.Publisher("/sensors/starboardFix", NavSatFix, queue_size=10)

    def updateGps(self):
        self.portGps.latitude = self.portGps.latitude + 0.000001
        self.portGps.header.stamp = rospy.Time.now()
        self.starboardGps.latitude = self.starboardGps.latitude + 0.000001
        self.starboardGps.header.stamp = rospy.Time.now()

    def publishTopics(self):
        self.portThrustPub.publish(self.portThrust)
        self.starboardThrustPub.publish(self.starboardThrust)
        self.imuPub.publish(self.imu)
        self.portGpsPub.publish(self.portGps)
        self.starboardGpsPub.publish(self.starboardGps)


if __name__ == '__main__':
    sim = wamvSim()
    while not rospy.is_shutdown():
        sim.updateGps()
        sim.publishTopics()
        rospy.Rate(10)
