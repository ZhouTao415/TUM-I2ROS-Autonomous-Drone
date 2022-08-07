#!/usr/bin/env python

import rospy
import time
from rosflight_msgs.msg import Status
from std_srvs.srv import Trigger

UNCALIBRATED_IMU = (1 << 5)

status = None

rospy.init_node('calibrate_imu', anonymous=True)

def statusCallback(msg):
	global status
	status = msg

sub = rospy.Subscriber("/rosflight/status", Status, statusCallback)
calibrate_imu = rospy.ServiceProxy('/rosflight/calibrate_imu', Trigger)

print "Waiting for status..."
while not status:
	time.sleep(1)

print "Calibrating IMU..."
time.sleep(5)
calibrate_imu()
time.sleep(1)
while status.error_code & UNCALIBRATED_IMU:
	time.sleep(1)
