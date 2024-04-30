#!/usr/bin/env python
# Every Python ROS Node will have the line 1 declaration at the top. The first line makes sure your script is executed
# as a Python script.
# From ROS wiki
import rospy
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
from tf.transformations import quaternion_from_euler, euler_from_quaternion
import numpy as np
import json
from plot_data import plot_data
from pid_controller import PIDController
from update_thrusters import update_thrusters

# Initialize ROS node
# Documentation on how to initialize ROS Node in Python:
# http://wiki.ros.org/rospy/Overview/Initialization%20and%20Shutdown#Initializing_your_ROS_Node
rospy.init_node('Lab1_main', anonymous=True)

# create the publisher object
# documentation for rospy.Publisher: http://wiki.ros.org/rospy/Overview/Publishers%20and%20Subscribers
bow_port_thruster_pub = rospy.Publisher('/bow_port_thruster', Float64, queue_size=10)
bow_stbd_thruster_pub = rospy.Publisher('/bow_stbd_thruster', Float64, queue_size=10)
vert_port_thruster_pub = rospy.Publisher('/vert_port_thruster', Float64, queue_size=10)
vert_stbd_thruster_pub = rospy.Publisher('/vert_stbd_thruster', Float64, queue_size=10)
aft_port_thruster_pub = rospy.Publisher('/aft_port_thruster', Float64, queue_size=10)
aft_stbd_thruster_pub = rospy.Publisher('/aft_stbd_thruster', Float64, queue_size=10)
aft_vert_thruster_pub = rospy.Publisher('/aft_vert_thruster', Float64, queue_size=10)

# Global variables
fusion_state_sub = np.array([])
desired_rate = 4  # Hz
kp_depth, ki_depth, kd_depth = 300.0, 10.0, 5.0
kp_heading, ki_heading, kd_heading = 20.0, 1.0, 2.0


# Initialize PID Controllers
depth_PID = PIDController(kp_depth, ki_depth, kd_depth)
heading_PID = PIDController(kp_heading, ki_heading, kd_heading)


# build callback function
def fusion_state_callback(msg):
    global current_depth, current_heading, fusion_state_sub
    # Convert quaternion to Euler angles to get heading...
    quat = np.array([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z,
                    msg.pose.pose.orientation.w])
    angles = euler_from_quaternion(quat)
    current_heading = np.rad2deg(angles[2])
    current_depth = msg.pose.pose.position.z
    fusion_state_sub = np.array([current_depth, current_heading])


# subscriber function
rospy.Subscriber('/fusion/pose_gt', Odometry, fusion_state_callback, queue_size=desired_rate)

rospy.sleep(1.0)

current_depth = fusion_state_sub[0]
current_heading = fusion_state_sub[1]


# PID control setup
desired_depth = -10.0
desired_heading = 90.0
rate = rospy.Rate(desired_rate)  # 4 Hz
dt = 1.0 / desired_rate
# Data for plotting
plot_time = np.empty((0,))
actual_depth_data = np.empty((0,))
actual_heading_data = np.empty((0,))
depth_error_data = np.empty((0,))
heading_error_data = np.empty((0,))
control_signal_data = np.empty((0,))
heading_control_signal_data = np.empty((0,))

# Main control loop
current_count = 0
total_duration = 300 * desired_rate  # duration in seconds multiplied by sampling ros rate
try:
    while current_count < total_duration:
        depth_control_signal, depth_error = depth_PID.update(current_depth, desired_depth, dt)
        heading_control_signal, heading_error = heading_PID.update(current_heading, desired_heading, dt)

        # Update thrusters
        thruster_msgs = update_thrusters(depth_control_signal, heading_control_signal)
        bow_port_thruster_pub.publish(thruster_msgs['bow_port'])
        bow_stbd_thruster_pub.publish(thruster_msgs['bow_stbd'])
        vert_port_thruster_pub.publish(thruster_msgs['vert_port'])
        vert_stbd_thruster_pub.publish(thruster_msgs['vert_stbd'])
        # aftPortThrusterPub.publish(thruster_msgs['aft_port'])
        # aftStbdThrusterPub.publish(thruster_msgs['aft_stbd'])
        # aftVertThrusterPub.publish(thruster_msgs['aft_vert'])

        # Collect data for plotting
        elapsed_time = current_count
        plot_time = np.append(plot_time, elapsed_time)
        actual_depth_data = np.append(actual_depth_data, current_depth)
        actual_heading_data = np.append(actual_heading_data, current_heading)
        depth_error_data = np.append(depth_error_data, depth_error)
        heading_error_data = np.append(heading_error_data, heading_error)
        control_signal_data = np.append(control_signal_data, depth_control_signal)
        heading_control_signal_data = np.append(heading_control_signal_data, heading_control_signal)

        rate.sleep()
        current_count += 1
except rospy.ROSInterruptException:  # handles ROS shutdown requests Ctrl+C or rosnode kill etc.
    rospy.loginfo("ROS shutdown request received.")
except Exception as e:  # handles and displays unexpected errors that aren't keyboard interrupts or system exits
    rospy.logerr("Unhandled exception: {}".format(e))
finally:
    # Save and plot final data
    rospy.loginfo("Shutting down, saving data...")
    # Plotting function call
    plot_data(plot_time, actual_depth_data, [desired_depth] * len(plot_time), actual_heading_data,
              [desired_heading] * len(plot_time), depth_error_data, heading_error_data, control_signal_data,
              heading_control_signal_data)

    # Create a filename with ros_rate and PID gains
    filename = "results__ROSrate{}_KpD{}_KiD{}_KdD{}_KpH{}_KiH{}_KdH{}.npy".format(
        desired_rate,
        kp_depth, ki_depth, kd_depth,
        kp_heading, ki_heading, kd_heading
    )
    # Save data to numpy binary file for later analysis
    np.savez(filename, plot_time=plot_time, actual_depth_data=actual_depth_data,
             desired_depth_data=[desired_depth] * len(plot_time),
             actual_heading_data=actual_heading_data,
             desired_heading_data=[desired_heading] * len(plot_time),
             depth_error_data=depth_error_data,
             heading_error_data=heading_error_data)

