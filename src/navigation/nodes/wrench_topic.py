#! /usr/bin/env python2

import rospy
import numpy as np
from std_msgs.msg import Int32
from std_msgs.msg import Float32
from geometry_msgs.msg import Wrench

class wrenchTopic:
    def __init__(self):
        rospy.init_node("wrench_topic_publisher")

        self.wrench = Wrench()
        # self.thrusterOffset = 0.377654
        self.thrusterOffset = 1.25
        self.leftThrust = 0
        self.rightThrust = 0
        self.leftTorque = 0
        self.rightTorque = 0

        rospy.Subscriber("/q3_thruster_input", Int32, self.update_left)
        rospy.Subscriber("/q4_thruster_input", Int32, self.update_right)

        self.wrenchPub = rospy.Publisher("/kalman/wrench", Wrench, queue_size=10)

    def publish_wrench(self):
        self.wrench.force.x = self.leftThrust + self.rightThrust
        self.wrench.force.y = 0
        self.wrench.force.z = 0
        self.wrench.torque.x = 0
        self.wrench.torque.y = 0
        self.wrench.torque.z = self.leftTorque + self.rightTorque

        self.wrenchPub.publish(self.wrench)

    def update_left(self, left_thrust):
        if left_thrust.data >= 0:
            self.leftThrust = left_thrust.data * 160/1000*4.448
        else:
            self.leftThrust = left_thrust.data * 80/1000*4.448

        self.leftTorque = self.leftThrust*self.thrusterOffset
        self.publish_wrench()

    def update_right(self, right_thrust):
        if right_thrust.data >= 0:
            self.rightThrust = right_thrust.data * 160/1000*4.448
        else:
            self.rightThrust = right_thrust.data * 80/1000*4.448

        self.rightTorque = -1*self.rightThrust*self.thrusterOffset
        self.publish_wrench()

if __name__=='__main__':
    wrenchPublisher = wrenchTopic()
    rospy.spin()
