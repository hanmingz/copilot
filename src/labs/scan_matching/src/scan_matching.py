#!/usr/bin/env python

import rospy
import math
import numpy as np
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Pose2D, PoseStamped, Point, Pose, Quaternion
import pdb
import cvxpy as cp
from tf.transformations import quaternion_from_euler

# Scan matching node subscribes to laser scan topic and returns the car's Pose2D.

ANGLE_RANGE = 270 # Hokuyo 10LX has 270 degrees scan
prev_pose = Pose2D(0, 0, 0)
prev_roto_trans = Pose2D(0, 0, 0) # q_(k-1) is the previous roto translation
prev_scan = LaserScan()
first_scan = True

pub = rospy.Publisher('location', PoseStamped, queue_size=1)

# Returns whether the two arguments n1 and n2 differ by no more than tolerance.
# Useful for comparing floating point numbers.
def is_close(n1, n2, tolerance = 1e-9):
    return abs(n1 - n2) <= tolerance

# Returns the closest point on the line defined by the points B and A to the
# given point P.
# Here, we assume B is to the "left" A, as shown in the
# following picture. The closest point on the line is denoted by C.
#
#   y
#   |
#   |    B
#   |         C
#   |               A
#   |       P
#   |
#    ------------------x
#
# If the closest point on the line lies outside of the line segment between
# A and B, that closest point is still returned. (What this means is that the
# closest point is not "clipped" between A and B and may be outside of that
# line srgment.)
#
# Source for algorithm:
# https://en.wikipedia.org/wiki/Distance_from_a_point_to_a_line
def get_closest_point(P, A, B):
    # If A and B are on the same vertical line (with infinite slope), return
    # (x-coordinate of A, y-coordinate of P).
    if is_close(A.x, B.x):
        return Point(A.x, P.y, 0.0)
    
    # Swap A and B if A has a lower x-coordinate than B.
    if A.x < B.x:
        tmp = A
        A = B
        B = tmp

    # m is the slope of the line between points A and B
    m = (A.y - B.y) / (A.x - B.x)

    # a, b, and c are the coefficients of the standard equation for a line:
    # ax + by + c = 0.
    # Given the slope m, and using the point-slope form y - y1 = m(x - x1),
    # we can derive the following expressions for a, b, and c:
    a = m
    b = -1.0
    c = A.y - m * A.x

    # C is the closest point on the line defined by A and B.
    numerator_parens = b * P.x - a * P.y
    denominator = a * a + b * b
    C_x = (b * (numerator_parens) - a * c) / denominator
    C_y = (a * (-numerator_parens) - b * c) / denominator
    return Point(C_x, C_y, 0.0)

# Input data is the laser scan data
# Outputs the car's estimated position in Pose2D
def scan_callback(scan):

    global prev_pose
    global prev_scan
    global prev_roto_trans
    global first_scan
    
    # Skips the first scan since there won't have been a previous scan to project to yet
    if first_scan == True:
        first_scan = False
        prev_scan = scan
        return
    
    print "Len of scan.ranges: ", len(scan.ranges)
    
    # Initialize matrix M
    M = np.zeros((4, 4))
    g = np.zeros((1, 4))
    
    # Initialize vector g
    print "Computing M, W, and g..."
    # Approximate p_i^w = p_i roto translated by q_(k-1) for every i <= n
    # So this would be over a for loop.
    for i in np.arange(-45, 225, .25): # in degrees
        p_i_dist = getRange(scan, i) # current point distance in meters
        p_i_x = p_i_dist * np.cos(i * np.pi / 180.0)
        p_i_y = p_i_dist * np.sin(i * np.pi / 180.0)
        p_i = Point(p_i_x, p_i_y, 0.0)
        p_i_w = roto_translate_point(p_i, prev_roto_trans)
        
        # Construct M_i for each scan point
        M_i = np.matrix([[1, 0, p_i_x, -p_i_y],
                         [0, 1, p_i_y, p_i_x]])
        
        # Add to 4x4 matrix M
        M = np.add(M, np.dot(M_i.T, M_i))
        
        # Also add to 4x1 vector g. 
        # pi_i is going to be the projection of current scan point onto previous scan's closest line segment
        # Hence this implies that we need to first find the closest line segment from the previous scan, to p_i_w.
        # Then we need to be able to project p_i_w onto this closest line segment
        pi_i = project_onto_prev_surface(p_i_w, i)
        pi_i = np.array([[pi_i.x],
                         [pi_i.y]]) # 2 x 1 matrix
        g = np.add(g, np.dot(pi_i.T, M_i)) # 1 x 4 matrix since (1 x 2) * (2 x 4) 
    
    # Then compute M, W and g as per assignment.
    # Here just compute the static matrix W.
    W = np.matrix([[0, 0, 0, 0],
                   [0, 0, 0, 0],
                   [0, 0, 1, 0],
                   [0, 0, 0, 1]])
    g = -2 * g.T # g is a 4 x 1 matrix
    
    # Solve x* = argmin_x(x^T*M*x + g^T * x) s.t. x^T*W*x = 1
    # We will brute force for the optimal x* over a finite set of discrete values for tx, ty, and theta.
    min_cost = float('Inf') # Keep track of the min cost to determine which is the best q
    min_x = 0.0
    min_y = 0.0
    min_theta = 0.0
    
    window_x = .10 # in meters, current position relative to previous position
    window_y = .05 # in meters, current position relative to previous position
    theta = 0.349 # in radians, current orientation relative to previous orientation
    # Up to 20 degrees in left or right (0.349 radians)
    
   # Over a 10cm x 10 cm box in front of the car (iterating over each cm) and over 40 degree rotations. This would create 10 x 10 x 40 = 4,000 possible q's for each one. We could also try limiting the number of degree rotations to see if that makes any improvement. 
    print "Finding optimal x by brute force..."
    for x in np.arange(0, window_x, .01): # by 1 cm increments
        for y in np.arange(-window_y, window_y, .01): # by 1 cm increments
            for theta in np.arange(-.349, .349, .0175): # by 1 degree increments
                q = Pose2D(x, y, theta)
                
                x_vec = np.matrix([[x], [y], [np.cos(theta)], [np.sin(theta)]])
                cost = x_vec.T*M*x_vec + g.T*x_vec
                
                if cost <= min_cost:
                    min_cost = cost
                    min_x = x
                    min_y = y
                    min_theta = theta

    # At this point we should have x* x_star_vec solved for
    
    # Set q_k <- x*
    q = Pose2D()
    q.x = min_x
    q.y = min_y
    q.theta = min_theta
    
    # Transform the prev_pose by q to get the current pose.
    current_pose = roto_translate_pose(prev_pose, q)
    print "Current pose: " + str(current_pose)
    
    print " "
    prev_scan = scan
    prev_pose = current_pose
    prev_roto_trans = q
    
    # Returns a PoseStamped because Rviz visualizes PoseStamped but not Pose2D
    msg = PoseStamped()
    quat = quaternion_from_euler(0.0, 0.0, current_pose.theta)
    msg.pose = Pose(Point(current_pose.x, current_pose.y, 0.0), Quaternion(quat[0], quat[1], quat[2], quat[3]))
    msg.header.frame_id = 'laser'
    print(msg)
    pub.publish(msg)
    
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

# Apply the roto-translation by q to a Point p   
# Return the new point coordinates as Point (x, y, z)
def roto_translate_point(p, q):
    x = p.x
    y = p.y
    
    new_point = Point()
    new_point.x = p.x * np.cos(q.theta) - p.y * np.sin(q.theta) + q.x
    new_point.y = p.x * np.sin(q.theta) + p.y * np.cos(q.theta) + q.y
    
    return new_point
    
# Apply the roto-translation by q to a Pose2D p   
# Return the new point coordinates as Pose2D (x, y, theta)
def roto_translate_pose(p, q):
    x = p.x
    y = p.y
    theta = p.theta
    
    new_point = Pose2D()
    new_point.x = p.x * np.cos(q.theta) - p.y * np.sin(q.theta) + q.x
    new_point.y = p.x * np.sin(q.theta) + p.y * np.cos(q.theta) + q.y
    new_point.theta = theta + q.theta
    
    return new_point

# Projects current scan point onto closest line segment from previous scan
# Input p is the p_i_w, the current scan point in previous reference frame (approximated)
def project_onto_prev_surface(p, p_index):
    global prev_scan
    
    p_x = p.x
    p_y = p.y
    
    # First need to find the closest line segment on previous scan to p. Or just find closest point. Current scan point called p and previous point called pp.
    min_dist = float('Inf')
    closest_point_1 = Point()
    closest_point_2 = Point()
    window = 25
    min_index = max(-45, p_index - window)
    max_index = min(225, p_index + window)
    for i in np.arange(min_index, max_index, .25): # in degrees
        pp_dist = getRange(prev_scan, i) # current point distance in meters
        pp_x = pp_dist * np.cos(i * np.pi / 180.0)
        pp_y = pp_dist * np.sin(i * np.pi / 180.0)
    
        dist = math.sqrt((p_x - pp_x)**2 + (p_y - pp_y)**2)
        if dist < min_dist:
            min_dist = dist
            closest_point_1 = Point(pp_x, pp_y, 0.0) # may need to manually set z to 0
            pp2_dist = getRange(prev_scan, i+.25)
            pp2_x = pp2_dist * np.cos(i * np.pi / 180.0)
            pp2_y = pp2_dist * np.sin(i * np.pi / 180.0)
            closest_point_2 = Point(pp2_x, pp2_y, 0.0)
   
    # At this point should have the closest point and the closest line segment

    p_projected_onto_surface = get_closest_point(p, closest_point_1,
        closest_point_2)
    
    return p_projected_onto_surface

if __name__ == '__main__':
    print("Scan matching node started")
    rospy.init_node('scan_matching', anonymous = True)
    rospy.Subscriber("scan", LaserScan, scan_callback)
    rospy.spin()
