#!/usr/bin/env python  
import rospy

# Because of transformations
import tf_conversions

import tf2_ros
from geometry_msgs.msg import PoseStamped, TransformStamped


def handle_pose(msg):
    br = tf2_ros.TransformBroadcaster()
    t = TransformStamped()

    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "world"
    t.child_frame_id = "laser"
    t.transform.translation.x = msg.pose.position.x
    t.transform.translation.y = msg.pose.position.y
    t.transform.translation.z = 0.0
    t.transform.rotation = msg.pose.orientation

    br.sendTransform(t)

if __name__ == '__main__':
    rospy.init_node('transform broadcaster')
    rospy.Subscriber('/location', PoseStamped, handle_pose)
    rospy.spin()
