#! /usr/bin/env python2

import rospy
import numpy as np
from std_msgs.msg import Float32
from geometry_msgs.msg import Wrench

class wrenchTopic:
    def __init__(self):
        rospy.init_node("wrench_topic_publisher")

        self.wrench = Wrench()
        # self.thrusterOffset = 0.377654
        self.thrusterOffset = 1.25
        # 160/1000*4.448 converts from lbf to N
        self.leftThrust = 0 #500 * 160/1000*4.448
        self.rightThrust = 0 #1000 * 160/1000*4.448
        self.leftTorque = self.leftThrust*self.thrusterOffset
        self.rightTorque = -1*self.rightThrust*self.thrusterOffset

        rospy.Subscriber("/wamv/thrusters/left_thrust_cmd", Float32, self.updateLeft)
        rospy.Subscriber("/wamv/thrusters/right_thrust_cmd", Float32, self.updateRight)
        self.wrenchPub = rospy.Publisher("/kalman/wrench", Wrench, queue_size=10)

    def updateLeft(self,data):
        self.leftThrust = data.data*160.0*4.448
        self.leftTorque = self.leftThrust*self.thrusterOffset
        self.publishWrench()

    def updateRight(self,data):
        self.rightThrust = data.data*160.0*4.448
        self.rightTorque = -1*self.rightThrust*self.thrusterOffset
        self.publishWrench()

    def publishWrench(self):
        self.wrench.force.x = self.leftThrust + self.rightThrust
        self.wrench.force.y = 0
        self.wrench.force.z = 0
        self.wrench.torque.x = 0
        self.wrench.torque.y = 0
        self.wrench.torque.z = self.leftTorque + self.rightTorque

        self.wrenchPub.publish(self.wrench)


if __name__=='__main__':
    wrenchPublisher = wrenchTopic()
    # while not rospy.is_shutdown():
    #     wrenchPublisher.publishWrench()
    rospy.spin()
