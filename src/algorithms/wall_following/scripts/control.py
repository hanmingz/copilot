#!/usr/bin/env python

import rospy
import message_filters
from race.msg import drive_param
from wall_following.msg import pid_input
from sensor_msgs.msg import Joy
import numpy as np

pub = rospy.Publisher('drive_parameters', drive_param, queue_size=1)

kp = 5.0
kd = 0.01
ki = 0.0
servo_offset = 0.0
prev_error = 0.0 
error = 0.0
integral = 0.0
deadman_butt = 0
throttle = 0.0
turn = 0.0
VMAX = 2.00 # meters per second

def control(error_data):
	global integral
	global prev_error
	global kp
	global ki
	global kd
	global throttle
	global turn
	global deadman_butt
	global VMAX
	velocity_ratio = error_data.pid_vel

	msg = drive_param()
	msg.angle = turn * 24 * np.pi / 180 + servo_offset
	if throttle < 0:
		print("velocity ratio", velocity_ratio)
		msg.velocity = velocity_ratio * throttle * VMAX
	else:
		msg.velocity = throttle
		
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
 	rospy.Subscriber("error", pid_input, control)
	rospy.Subscriber("vesc/joy", Joy, joy_callback)
	print("control.py finished setup")
	rospy.spin()


if __name__ == '__main__':
	print("Listening to error for PID")
	listener()
