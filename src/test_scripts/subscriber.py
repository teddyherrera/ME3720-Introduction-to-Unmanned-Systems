#!/usr/bin/env python
# Every Python ROS Node will have the line 1 declaration at the top. The first line makes sure your script is executed
# as a Python script.
# From ROS wiki
import rospy
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import numpy as np


# Global variables
current_depth = None
current_heading = None


# build callback function
def fusion_state_callback(msg):
    global current_depth, current_heading
    # Convert quaternion to Euler angles to get heading...
    quat = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w]
    angles = euler_from_quaternion(quat)
    current_heading = np.rad2deg(angles[2])
    current_depth = msg.pose.pose.position.z

    rospy.loginfo(current_heading)
    rospy.loginfo(current_depth)


def listener():
    rospy.init_node('listener', anonymous=True)

    fusion_state_sub = rospy.Subscriber('/fusion/pose_gt', Odometry, fusion_state_callback)

    rospy.spin()


if __name__ == '__main__':
    listener()

