#!/usr/bin/env python

import rospy
import message_filters
import math
from race.msg import drive_param
from wall_following.msg import pid_input
from sensor_msgs.msg import Joy
from sensor_msgs.msg import LaserScan
import numpy as np

pub = rospy.Publisher('drive_parameters', drive_param, queue_size=1)

servo_offset = 0.0
deadman_butt = 0
throttle = 0.0
turn = 0.0
VMAX = 2.00 # meters per second
ANGLE_RANGE = 270 # Hokuyo 10LX has 270 degrees scan

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

def filter(laser_data):
	filter_array = np.array([0.25, 0.25, 0.25, 0.25])
	return np.convolve(laser_data, filter_array)

def get_min_dist(laser_data):
	global turn
	laser_array = filter(laser_data.ranges)
	length = len(laser_array)
	min_dist = 30.0
	for i in range(11):
		index = length *(turn*24 + 90 + 45 + (i-5)*15) / ANGLE_RANGE
		dist = laser_array[int(index)]
		if math.isinf(dist):
			dist = 10.0
		if math.isnan(dist):
			dist = 4.0
		if np.sin(np.abs((i-5)*15) * np.pi / 180) < 0.2 / dist:
			min_dist = min(min_dist, dist)

	return min_dist

def laser_callback(laser_data):
	global throttle
	global turn
	global deadman_butt
	global VMAX
	min_dist = get_min_dist(laser_data)
	velocity_ratio = 0.5 * min_dist
	msg = drive_param()
	msg.angle = turn * 24 * np.pi / 180 + servo_offset
	if min_dist < 0.5 and throttle * VMAX < -1:
		velocity_ratio = 0
	if velocity_ratio > 1:
		velocity_ratio = 1

	if throttle < 0:
		print("velocity ratio", velocity_ratio)
		msg.velocity = velocity_ratio * throttle * VMAX
	else:
		msg.velocity = throttle * 0.5
		
	if not deadman_butt:
		msg.velocity = 0
		msg.angle = 0

	print "Velocity", msg.velocity
	print "Angle in Degrees", msg.angle*180/np.pi

	pub.publish(msg)


def joy_callback(data):
	global throttle
	global turn
	global deadman_butt
	button = data.buttons
	deadman_butt = button[4]
	axis = data.axes
	throttle = axis[1] * -1
	turn = axis[2]
	

def listener():
	rospy.init_node('pid_controller', anonymous=True)
 	rospy.Subscriber("scan", LaserScan, laser_callback)
	rospy.Subscriber("vesc/joy", Joy, joy_callback)
	print("control.py finished setup")
	rospy.spin()


if __name__ == '__main__':
	print("Listening to error for PID")
	listener()
