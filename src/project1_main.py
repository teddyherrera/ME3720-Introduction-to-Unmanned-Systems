#!/usr/bin/env python
# Every Python ROS Node will have the line 1 declaration at the top. The first line makes sure your script is executed
# as a Python script.
# From ROS wiki
import rospy
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
from tf.transformations import quaternion_from_euler, euler_from_quaternion
import numpy as np
from plot_data import plot_data
from pid_controller import PIDController
from update_heading import update_heading
from set_depth import set_depth
from cte import cte


def wrap_to_180(angle):
    """Wrap an angle in degrees to [-180, 180]."""
    return (angle + 180) % 360 - 180


# Initialize ROS node
# Documentation on how to initialize ROS Node in Python:
# http://wiki.ros.org/rospy/Overview/Initialization%20and%20Shutdown#Initializing_your_ROS_Node
rospy.init_node('Project1_main', anonymous=True)

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
desired_rate = 3.0  # Hz
kp_depth, ki_depth, kd_depth = 250.0, 12.5, 25.0
kp_heading, ki_heading, kd_heading = 1.0, 0.2, 0.8
stability_thrust = 500.0


# Initialize PID Controllers
depth_PID = PIDController(kp_depth, ki_depth, kd_depth)
heading_PID = PIDController(kp_heading, ki_heading, kd_heading)

# Define waypoints
waypoint1 = np.array([25, 25, -10])
waypoint2 = np.array([50, 25, -10])
waypoint3 = np.array([50, 50, -10])
waypoint4 = np.array([25, 50, -10])

# Create a numpy array for all waypoints
WAYPOINTS = np.array([waypoint1, waypoint2, waypoint3, waypoint4, waypoint1])

# Accessing separate arrays for x, y, z coordinates
WAYPOINT_x = WAYPOINTS[:, 0]
WAYPOINT_y = WAYPOINTS[:, 1]
WAYPOINT_z = WAYPOINTS[:, 2]

# build callback function
def fusion_state_callback(msg):
    global body_x, body_y, body_depth, body_roll, body_pitch, body_heading, fusion_state_sub
    # Convert quaternion to Euler angles to get heading...
    quat = np.array([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z,
                    msg.pose.pose.orientation.w])
    roll, pitch, yaw = euler_from_quaternion(quat)
    body_roll = np.rad2deg(roll)
    body_pitch = np.rad2deg(pitch)
    body_heading = np.rad2deg(yaw)
    body_x = msg_pose.pose.pose.position.x
    body_y = msg.pose.pose.pose.position.y
    body_depth = msg.pose.pose.position.z
    fusion_state_sub = np.array([body_x, body_y, body_depth, body_roll, body_pitch, body_heading])


# subscriber function
rospy.Subscriber('/fusion/pose_gt', Odometry, fusion_state_callback, queue_size=desired_rate)

rospy.sleep(1.0)  # needed so that the callback function can pull the initial state before moving on

# PID control setup
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
rho = 5  # look ahead distance
jj = 1
start_time = time.time()
try:
    while not rospy.is_shutdown():
        # Get current depth and heading
        body_x = fusion_state_sub[0]
        body_y = fusion_state_sub[1]
        body_depth = fusion_state_sub[2]
        body_roll = fusion_state_sub[3]
        body_pitch = fusion_state_sub[4]
        body_heading = fusion_state_sub[5]

        # Calculate desired heading and depth
        desired_heading = CTE(body_x, body_y, WAYPOINT_x[jj], WAYPOINT_y[jj], WAYPOINT_X[jj+1], WAYPOINT_y[jj+1], rho)
        desired_depth = WAYPOINT_z[jj]

        # Calculate depth error
        depth_error = depth_PID.update(current_depth, desired_depth, dt)

        if abs(depth_error) >= 0.5:
            update_thrusters_depth = set_depth(depth_control_signal, stability_thrust)


        # Calculate heading error
        heading_error = heading_PID.update_angle(current_heading, desired_heading, dt)


        # Update thrusters
        thruster_msgs = update_thrusters(depth_error, heading_error)
        bow_port_thruster_pub.publish(thruster_msgs['bow_port'])
        bow_stbd_thruster_pub.publish(thruster_msgs['bow_stbd'])
        vert_port_thruster_pub.publish(thruster_msgs['vert_port'])
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
        current_count += dt
        print('current_count: {}'.format(current_count))
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

