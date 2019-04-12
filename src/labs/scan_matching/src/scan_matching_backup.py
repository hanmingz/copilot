#!/usr/bin/env python

import rospy
import math
import numpy as np
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Pose2D
import pdb

# Scan matching node subscribes to laser scan topic and returns the car's Pose2D.

ANGLE_RANGE = 270 # Hokuyo 10LX has 270 degrees scan
prev_pose = Pose2D(0, 0, 0)
prev_scan = LaserScan()

# Did we get the first scan?
got_initial_scan = False

pub = rospy.Publisher('location', Pose2D, queue_size=1)

# TODO: Here will probably need a global variable which keeps track of the car's previous Pose2D. This is where we can initialize the global variable to (0, 0) with no rotation. 

# Input data is the laser scan data
# Outputs the car's estimated position in Pose2D
def scan_callback(data):

    global prev_pose
    global prev_scan
    global got_initial_scan
    
    print "Len of data.ranges: ", len(data.ranges)
    print "Len of prev_scan.ranges: ", len(prev_scan.ranges)
    if not got_initial_scan:
        prev_scan = data
        got_initial_scan = True
        #print "Set initial scan"
        #while True:
        #    pass
        return
    
    # First implement the simple match function (from slide 32 of Houssam's Scan Matching slide)

    # We expect that this will not be robust to abrupt geometry changes, measurement errors, or displacements. But it is the simpler solution so will try this first.
    
    # M(q) = sum over i of abs(s_i - p_i^w roto-translated with q)
    q = get_min_q(data)
    
    # Transform the prev_pose by q to get the current pose.
    current_pose = roto_translate(prev_pose, q)
    
    # Think about what we want students to output. The Pose2D will be the location of the robot with respect to the starting laser scan. In Rviz when we visualize we may want to see the laser scans with respect to the car location. But to do that we will need a static transform between the laser and base_link. Or actually we will want to visualize Rviz in the map frame, so that we can see the laser's Pose2D moving in the world. So I guess we will need a map frame, a base_link frame, and a laser frame.
    
    print " "

    msg = current_pose
    pub.publish(msg)
    prev_pose = current_pose
    prev_scan = data
    
def getRange(data, angle):
    # data: single message from topic /scan
    # angle: between -45 to 225 degrees, where 0 degrees is directly to the right
    # Outputs length in meters to object with angle in lidar scan field of view
    if angle > 224.9:
        angle = 224.9
    index = int(len(data.ranges) * (angle + 45) / ANGLE_RANGE)
    dist = data.ranges[index]
    if math.isinf(dist):
        return 10.0
    if math.isnan(dist):
        return 4.0
    return data.ranges[int(index)]

# We will use a global variable for previous scan data
# Takes in as input the current scan data
# Iterates over the possible roto translations for q.
# May want the window for q to be set at the top as a variable in all caps (as a constant)
# Return q as a (x, y, theta) triple
def get_min_q(cur_scan):
    min_cost = float('Inf') # Keep track of the min cost to determine which is the best q
    q_min = Pose2D()
    
    # Instead of looking at i from 0 to 1080 (for each of the laser scan points), we may
    # want to limit the angles that we look at i because there is not overlap with every single
    # laser scan point. For instance, when the car moves forward, the furthest back point
    # seen by the previous laser scan won't be seen anymore by the current laser scan.
    
    # Since Lidar is operating at 40 Hz, we can assume car won't move in x, y much between scans.
    # And can probably also assume that it will only change theta by small amount.
    window_x = .10 # in meters, current position relative to previous position
    window_y = .05 # in meters, current position relative to previous position
    theta = 0.349 # in radians, current orientation relative to previous orientation
    # Up to 20 degrees in left or right (0.349 radians)
    
    # This means that we also need to iterate over the q's. The roto-translation matrices will have translations in x, y, and rotations. Perhaps we can do something like over a 10cm x 10 cm box in front of the car (iterating over each cm) and over 360 degree rotations. This would create 10 x 10 x 360 = 36,000 possible q's for each one. We could also try limiting the number of degree rotations to see if that makes any improvement. 
    for x in np.arange(0, .10, .01): # by 1 cm increments
        for y in np.arange(-.05, .05, .01): # by 1 cm increments
            for theta in np.arange(-.349, .349, .0175): # by 1 degree increments
                q = Pose2D(x, y, theta)
                #pdb.set_trace()
                cost = 0.0 # Current cost starts at 0 and gets added to
                for i in np.arange(-45, 225, .25): # in degrees
                    s_i_dist = getRange(prev_scan, i) # previous point distance in meters
                    s_i_x = s_i_dist * np.cos(i * np.pi / 180.0)
                    s_i_y = s_i_dist * np.sin(i * np.pi / 180.0)
                    s_i = Pose2D(s_i_x, s_i_y, 0.0)
                    
                    p_i_dist = getRange(cur_scan, i) # current point distance in meters
                    p_i_x = p_i_dist * np.cos(i * np.pi / 180.0)
                    p_i_y = p_i_dist * np.sin(i * np.pi / 180.0)
                    p_i = Pose2D(p_i_x, p_i_y, 0.0)
                    
                    pi_roto_translated_by_q = roto_translate(p_i, q)
                    abs_difference = (s_i.x - pi_roto_translated_by_q.x) ** 2 + (s_i.y - pi_roto_translated_by_q.y) ** 2
                    cost += abs_difference
                if cost <= min_cost:
                    min_cost = cost
                    q_min = q
            print "Finished theta loop"
        print "Finished y loop"
    return q_min

# Apply the roto-translation by q to a Pose2D p   
# Return the new point coordinates as Pose2D (x, y, theta)
def roto_translate(p, q):
    x = p.x
    y = p.y
    theta = p.theta
    
    new_point = Pose2D()
    new_point.x = p.x * np.cos(q.theta) - p.y * np.sin(q.theta) + q.x
    new_point.y = p.x * np.sin(q.theta) + p.y * np.cos(q.theta) + q.y
    new_point.theta = theta + q.theta
    
    return new_point

# TODO: Need a helper method that takes in scan point in new reference frame (p_i) and outputs scan point in previous scan's reference frame (p_i^w)


if __name__ == '__main__':
    print("Scan matching node started")
    rospy.init_node('scan_matching', anonymous = True)
    rospy.Subscriber("scan", LaserScan, scan_callback)
    rospy.spin()
