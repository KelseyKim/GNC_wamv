#! /usr/bin/env python2

import rospy
import time
from heron_msgs.msg import Drive

rospy.init_node("thrust")

cmdDrive = Drive()
cmdPub = rospy.Publisher("/cmd_drive", Drive, queue_size=10)


close_time = time.time() + 5
while close_time > time.time():
    cmdDrive.left = 0.25
    cmdDrive.right = 0.75
    cmdPub.publish(cmdDrive)

close_time = time.time() + 5
while close_time > time.time():
    cmdDrive.left = 0.75
    cmdDrive.right = 0.25
    cmdPub.publish(cmdDrive)

close_time = time.time() + 5
while close_time > time.time():
    cmdDrive.left = 0.25
    cmdDrive.right = 0.75
    cmdPub.publish(cmdDrive)

close_time = time.time() + 5
while close_time > time.time():
    cmdDrive.left = 0.75
    cmdDrive.right = 0.25
    cmdPub.publish(cmdDrive)

close_time = time.time() + 5
while close_time > time.time():
    cmdDrive.left = 0.25
    cmdDrive.right = 0.75
    cmdPub.publish(cmdDrive)

close_time = time.time() + 5
while close_time > time.time():
    cmdDrive.left = 0.75
    cmdDrive.right = 0.25
    cmdPub.publish(cmdDrive)

close_time = time.time() + 5
while close_time > time.time():
    cmdDrive.left = 0.25
    cmdDrive.right = 0.75
    cmdPub.publish(cmdDrive)

close_time = time.time() + 5
while close_time > time.time():
    cmdDrive.left = 0.75
    cmdDrive.right = 0.25
    cmdPub.publish(cmdDrive)


close_time = time.time() + 5
while close_time > time.time():
    cmdDrive.left = 0.0
    cmdDrive.right = 0.0
    cmdPub.publish(cmdDrive)
