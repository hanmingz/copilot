#!/usr/bin/env python

import csv
import math
import numpy as np
import os
import rospy
from sensor_msgs.msg import LaserScan
from race.msg import pid_input

# This file loads a list of turn and velocity instructions. Each time the car
# sees a new opening, it fetches the next instruction and executes for TURN_DELAY
# seconds. The reason for the delay is that sometimes the car can continue
# to see openings while in mid-turn, and we don't want the car to prematurely
# fetch the next turn instruction while mid-turn.

# We did testing of this file in the hallways around Levine Hall 2nd floor, around
# the mLab. This strategy of wall following with explicit instructions does not work
# well in other maps such as the Moore Building 2nd floor loop or Towne because there are many unpredictable "openings", such as ramps and stair openings.

# In order to get this running well on your car you will probably need to tune
# the TURN_DELAY and angles of lidar scan at which the car searches for openings.

# Load the turn and velocity instructions from instructions.csv
dirname = os.path.dirname(__file__)
filename = os.path.join(dirname, '../explicit_instructions/instructions.csv')
with open(filename) as f:
	INSTRUCTIONS = [tuple(line) for line in csv.reader(f)]

TURN_DELAY = 3.5 # number of seconds to wait before detecting next opening 
OPENING_THRESHOLD = 0.50 # meters
ANGLE_RANGE = 270 # Hokuyo 10LX has 270 degrees scan
DESIRED_DISTANCE_RIGHT = 0.55 # meters
DESIRED_DISTANCE_LEFT = 0.55
CAR_LENGTH = 0.50 # Traxxas Rally is 20 inches or 0.5 meters
RIDICULOUS_DISTANCE_THRESHOLD = 40 # metal reflections give inaccurate distance measurements
TURN_VELOCITY = 1.0 # constant speed which car turns at

instruction_index = -1 # zero-indexed for the instructions.csv instructions
independence_time = 0 # This is the time in rospy seconds when the robot is free to detect its next opening. Like a counter. It is set according to TURN_DELAY.
turn = "center" # initially robot is just going down straight
velocity = 2.0 # initially speed in m/s but will change with instructions
detected_opening = False

pub = rospy.Publisher('error', pid_input, queue_size=10)

# Takes in an angle between -45 to 225, where 0 degrees is directly to the right
# Outputs length in meters to object with angle in lidar scan.
def getRange(data, angle):
    if angle > 224.9:
        angle = 224.9
    index = len(data.ranges) * (angle + 45) / ANGLE_RANGE
    dist = data.ranges[int(index)]
    if math.isinf(dist):
        return 10.0
    if math.isnan(dist):
        return 4.0
    return data.ranges[int(index)]

# Given the laser scan data, determine if there is a left opening, right opening,
# left and right opening, or no openings. 
def detectOpening(data):
    # Check left between 20 and 30 degrees, and right between 150 and 160 degrees
    if detectOpeningHelper(data, 20.0, 30.0) or detectOpeningHelper(data, 150.0, 160.0):
        return True
    else:
        return False
    
# Helper method so we don't have duplicate code in detectOpening for left and right
def detectOpeningHelper(data, minAngle, maxAngle):
    for angle in np.arange(minAngle, maxAngle, .25):
        dist_1 = getRange(data, angle)
        dist_2 = getRange(data, angle + .25)
        dist_11 = getRange(data, angle + 2.50)
        dist_12 = getRange(data, angle + 2.75)
        
        # Skip iteration if lidar is seeing metal reflections
        if dist_1 > RIDICULOUS_DISTANCE_THRESHOLD or dist_2 > RIDICULOUS_DISTANCE_THRESHOLD or dist_11 > RIDICULOUS_DISTANCE_THRESHOLD or dist_12 > RIDICULOUS_DISTANCE_THRESHOLD:
            continue # skip iteration if sees metal reflection since not true opening
        # For each angle, compare it with 2.5 degrees past it and see if there is
        # a difference in distance in meters greater than OPENING_THRESHOLD
        if(abs(dist_1 - dist_11) > OPENING_THRESHOLD and abs(dist_2 - dist_12) > OPENING_THRESHOLD):
            return True
    return False

def followRight(data,desired_trajectory):
    global alpha

    a = getRange(data,60)
    b = getRange(data,0)
    swing = math.radians(60)
    alpha = math.atan((a*math.cos(swing)-b)/(a*math.sin(swing)))
    print "a","b", a, b
    print "Alpha right",math.degrees(alpha)
    curr_dist = b*math.cos(alpha)

    future_dist = curr_dist + CAR_LENGTH * math.sin(alpha)
    print "Right : ",future_dist
    error = desired_trajectory - future_dist

    print "Current Distance Right: ", curr_dist
    return error, curr_dist

def followLeft(data,desired_trajectory):
    global alpha

    a = getRange(data,120)
    b = getRange(data,179.9)
    swing = math.radians(60)
    print "a","b", a, b
    alpha = -math.atan((a*math.cos(swing)-b)/(a*math.sin(swing)))
    print "Alpha left",math.degrees(alpha)
    curr_dist = b*math.cos(alpha)
    
    future_dist = curr_dist - CAR_LENGTH * math.sin(alpha)
    print "Left : ",future_dist
    error = future_dist - desired_trajectory

    print "Current Distance Left: ", curr_dist
    return error, curr_dist
    
def followCenter(data):
    global alpha

    a = getRange(data,120)
    b = getRange(data,179.9)
    swing = math.radians(60)
    print "left distances: ", a, b
    a = truncateMetalDist(a)
    b = truncateMetalDist(b)
    alpha = -math.atan((a*math.cos(swing)-b)/(a*math.sin(swing)))
    print "Alpha left",math.degrees(alpha)
    curr_dist1 = b*math.cos(alpha)
    future_dist1 = curr_dist1-CAR_LENGTH*math.sin(alpha)

    a = getRange(data,60)
    b = getRange(data,0)
    print "right distances: ", a, b
    a = truncateMetalDist(a)
    b = truncateMetalDist(b)
    swing = math.radians(60)
    alpha = math.atan((a*math.cos(swing)-b)/(a*math.sin(swing)))
    print "Alpha right",math.degrees(alpha)
    curr_dist2 = b*math.cos(alpha)
    future_dist2 = curr_dist2+CAR_LENGTH*math.sin(alpha)

    print "dist 1 : ",future_dist1
    print "dist 2 : ",future_dist2
    
    error = future_dist1 - future_dist2
    print "Error : ",error
    return error, curr_dist2 - curr_dist1

def truncateMetalDist(dist):
    # Since our lidar can only see 10m, any value really big is probably erroneous such as from a metal reflection. 
    if dist > 20.0:
        return 4.0
    else:
        return dist

# Takes in turn which is either "left", "right", or "center" and outputs corresponding error. Also takes in laser scan data.
# Outputs error that corresponds with the turn direction. Either does left wall
# follow, right wall follow, or center wall follow.
def followWallInDirection(turn, data):
    if (turn == "left"):
        error_left, curr_dist_left = followLeft(data, DESIRED_DISTANCE_LEFT)
        error = error_left
    elif (turn == "right"):
        error_right, curr_dist_right = followRight(data, DESIRED_DISTANCE_RIGHT)
        error = error_right
    elif(turn == "center"):
        error_center, curr_dist_center = followCenter(data)
        error = error_center
    else:
        error = 0
    return error

def callback(data):
    global error
    global detected_opening
    global independence_time # amount of time before can fetch next turn instruction
    global velocity
    global turn
    global instruction_index
    
    print " "

    detected_opening = detectOpening(data) 
    print "Detected opening? ", detected_opening
    
    current_time = rospy.get_time()

    # If detected new opening for the first time:
    if detected_opening == True and current_time >= independence_time:
        # set independence_time to the full timer duration
        independence_time = TURN_DELAY + current_time
        if turn != "stop":
            instruction_index += 1 # so that next time detecting opening will call next instruction
        turn = INSTRUCTIONS[instruction_index][0] # get first item of tuple
        if turn == "stop":
            velocity = 0.0
        else:
            velocity = TURN_VELOCITY

    # Case where robot is free to move because it has already exceeded the previously set timer
    elif current_time >= independence_time and instruction_index >= 0:
        velocity = float(INSTRUCTIONS[instruction_index][1]) # get second item of tuple
        turn = "center"

    # Regardless of which case above, robot will be doing a wall follow (left, center, or right)
    error = followWallInDirection(turn, data)

    print "On turn instruction index:", instruction_index
    print "Turn: ", turn

    msg = pid_input()
    msg.pid_error = error
    msg.pid_vel = velocity
    pub.publish(msg)


if __name__ == '__main__':
    print("Wall following with explicit instructions started")
    rospy.init_node('pid_error_explicit_instructions', anonymous = True)
    rospy.Subscriber("scan", LaserScan, callback)
    rospy.spin()
