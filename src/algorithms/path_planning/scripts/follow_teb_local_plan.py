#!/usr/bin/env python

import rospy
from race.msg import drive_param
from geometry_msgs.msg import Twist
import math
import numpy as np
import pdb

WHEELBASE_LENGTH = .325 # meters, distance between front and rear axles

# Publisher for 'drive_parameters' (speed and steering angle)
pub = rospy.Publisher('drive_parameters', drive_param, queue_size=1)

# Callback for the topic '/move_base/TebLocalPlannerROS/teb_feedback' of type teb_local_planner/FeedbackMsg.
# Breaks down twist velocity components into drive_param speed and steering angle
def callback(data):

    print " "
    x_dot = data.linear.x # dot is the representation of velocity (derivative of x)
    y_dot = data.linear.y
    theta_dot = data.angular.z # Velocity which car rotates with respect to map
    
    # velocity = math.sqrt(x_dot**2 + y_dot**2)
    velocity = x_dot
    # angle = math.atan2(WHEELBASE_LENGTH * theta_dot, velocity)
    angle = theta_dot
#    angle = -1.0 * angle # Need to flip the sign because left steering is negative

    print "Computed velocity: ", velocity
    print "Computed angle: ", angle
    print "Angle in Degrees", angle*180/np.pi
    angle = np.clip(angle, -0.4189, 0.4189) # 0.4189 radians = 24 degrees because car can only turn 24 degrees max
    if velocity < 1.0 and velocity >= 0.0:
      velocity = 1.0
#    if angle < 0.035 and angle > -0.035: # 2 degrees
#        velocity = 3.0
    print "Output velocity: ", velocity
    print "Output angle: ", angle

    msg = drive_param()
    msg.velocity = velocity
    msg.angle = angle
    pub.publish(msg)
    
if __name__ == '__main__':
    rospy.init_node('pure_pursuit')
    rospy.Subscriber('/cmd_vel', Twist, callback, queue_size=1)
    rospy.spin()

