#!/usr/bin/env python

import rospy
import math
import numpy as np
import yaml
import sys
from sensor_msgs.msg import LaserScan
from wall_following.msg import pid_input
import pdb

ANGLE_RANGE = 270 # Hokuyo 10LX has 270 degrees scan
DESIRED_DISTANCE_RIGHT = 0.9 # meters
DESIRED_DISTANCE_LEFT = 0.55
CAR_LENGTH = 0.50 # Traxxas Rally is 20 inches or 0.5 meters

pub = rospy.Publisher('error', pid_input, queue_size=10)

def getRange(data, angle):
	# data: single message from topic /scan
	# angle: between -45 to 225 degrees, where 0 degrees is directly to the right
	# Outputs length in meters to object with angle in lidar scan field of view
	if angle > 179.9:
		angle = 179.9
	index = len(data.ranges) * (angle + 45) / ANGLE_RANGE
	dist = data.ranges[int(index)]
	if math.isinf(dist):
		return 10.0
	if math.isnan(dist):
		return 4.0
	return data.ranges[int(index)]



def lookahead(data):
	front = getRange(data, 90)
	print "front distance", front
	if front > 2.0:
		return 1
	else:
		return front * 0.5

def callback(data):
	global error
	global alpha
	global final_direction
	global prev_direction

	msg = pid_input()
	msg.header.stamp = rospy.Time.now()
	msg.pid_error = 0
	msg.pid_vel = lookahead(data)
	pub.publish(msg)


if __name__ == '__main__':
	print("Laser node started")
	rospy.init_node('dist_finder',anonymous = True)
	rospy.Subscriber("scan",LaserScan,callback)
	rospy.spin()
