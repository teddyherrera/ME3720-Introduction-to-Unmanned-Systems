#!/usr/bin/env python
# Every Python ROS Node will have the line 1 declaration at the top. The first line makes sure your script is executed
# as a Python script.
# From ROS wiki
import rospy
from uuv_gazebo_ros_plugins_msgs.msg import FloatStamped
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import numpy as np
import matplotlib.pyplot as plt
from pid_controller import PIDController
from depth_heading_updater import depth_heading_updater
from cte import cte


# Function to wrap angles to [-180, 180]
def wrap_to_180(angle):
    """Wrap an angle in degrees to [-180, 180]."""
    return (angle + 180) % 360 - 180


# Global variables
desired_rate = 2  # Hz
kp_depth, ki_depth, kd_depth = 250.0, 4, 10.0
kp_heading, ki_heading, kd_heading = 1.0, 0.2, 0.8
stability_thrust = 500.0  # provides bow and aft port/stbd thrusters with a baseline thrust to stabilize rotation
body_x = None
body_y = None
body_depth = None
body_roll = None
body_pitch = None
body_heading = None

# Initialize PID Controllers
depth_PID = PIDController(kp_depth, ki_depth, kd_depth)
heading_PID = PIDController(kp_heading, ki_heading, kd_heading)

# Define waypoints
waypoint1 = np.array([0.0, 0.0, -10.0])
waypoint2 = np.array([25.0, 25.0, -10.0])
waypoint3 = np.array([50.0, 25.0, -15.0])
waypoint4 = np.array([50.0, 50.0, -20.0])
waypoint5 = np.array([25.0, 50.0, -15.0])

# Create a numpy array of waypoints
WAYPOINTS = np.array([waypoint1, waypoint2, waypoint3, waypoint4, waypoint5, waypoint2])

# Accessing separate arrays for x, y, z coordinates
WAYPOINT_x = WAYPOINTS[:, 0]
WAYPOINT_y = WAYPOINTS[:, 1]
WAYPOINT_z = WAYPOINTS[:, 2]

# Initialize ROS node
# Documentation on how to initialize ROS Node in Python:
# http://wiki.ros.org/rospy/Overview/Initialization%20and%20Shutdown#Initializing_your_ROS_Node
rospy.init_node('Project1_main', anonymous=True)

# create the publisher object
# documentation for rospy.Publisher: http://wiki.ros.org/rospy/Overview/Publishers%20and%20Subscribers
bow_port_thruster_pub = rospy.Publisher('/fusion/thrusters/3/input', FloatStamped, queue_size=desired_rate)
bow_stbd_thruster_pub = rospy.Publisher('/fusion/thrusters/0/input', FloatStamped, queue_size=desired_rate)
vert_port_thruster_pub = rospy.Publisher('/fusion/thrusters/4/input', FloatStamped, queue_size=desired_rate)
vert_stbd_thruster_pub = rospy.Publisher('/fusion/thrusters/1/input', FloatStamped, queue_size=desired_rate)
aft_port_thruster_pub = rospy.Publisher('/fusion/thrusters/5/input', FloatStamped, queue_size=desired_rate)
aft_stbd_thruster_pub = rospy.Publisher('/fusion/thrusters/2/input', FloatStamped, queue_size=desired_rate)
aft_vert_thruster_pub = rospy.Publisher('/fusion/thrusters/6/input', FloatStamped, queue_size=desired_rate)


# build callback function
def fusion_state_callback(msg):
    global body_x, body_y, body_depth, body_roll, body_pitch, body_heading
    # Convert quaternion to Euler angles to get heading
    quat = np.array([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z,
                    msg.pose.pose.orientation.w])
    roll, pitch, yaw = euler_from_quaternion(quat)
    body_roll = np.rad2deg(roll)
    body_pitch = np.rad2deg(pitch)
    body_heading = wrap_to_180(np.rad2deg(yaw))
    body_x = msg.pose.pose.position.x
    body_y = msg.pose.pose.position.y
    body_depth = msg.pose.pose.position.z


# subscriber function
rospy.Subscriber('/fusion/pose_gt', Odometry, fusion_state_callback, queue_size=desired_rate)

rospy.sleep(1.0)  # needed so that the callback function can pull the initial state before moving on

# PID control setup
rate = rospy.Rate(desired_rate)  # Hz
dt = 1.0 / desired_rate

# Main control loop
rho = 5  # look ahead distance
jj = 0  # waypoint index
start_time = rospy.get_time()  # used for time normalization
# Initialize empty dictionary to store data for plotting
# Keys represent the data type and values are numpy arrays
data_storage = {
    'plot_time': np.array([], dtype=float),
    'body_x_data': np.array([], dtype=float),
    'body_y_data': np.array([], dtype=float),
    'body_depth_data': np.array([], dtype=float),
    'desired_depth_data': np.array([], dtype=float),
    'body_heading_data': np.array([], dtype=float),
    'desired_heading_data': np.array([], dtype=float),
    'depth_error_data': np.array([], dtype=float),
    'heading_error_data': np.array([], dtype=float)
}
try:
    while not rospy.is_shutdown():

        current_time = rospy.get_time() - start_time  # normalize time

        # Call cross-track error controller (cte) function
        # Pass in body_x, body_y, and the next two x and y waypoints
        # Also pass in rho, the look-ahead distance
        desired_heading = cte(body_x, body_y, WAYPOINT_x[jj], WAYPOINT_y[jj], WAYPOINT_x[jj+1], WAYPOINT_y[jj+1], rho)

        # Set desired depth to the current waypoint z value
        desired_depth = WAYPOINT_z[jj+1]

        # Call depth control function
        # Pass in body_depth, desired_depth, and dt
        # Returns a tuple with where index [0] is control signal and index [1] is error
        depth_PID_output = depth_PID.depth_update(body_depth, desired_depth, dt)
        # call heading control function
        # Returns a tuple with where index [0] is control signal and index [1] is error
        heading_PID_output = heading_PID.heading_update(body_heading, desired_heading, dt)

        # Bailout logic for large errors/turns
        if abs(heading_PID_output[1]) >= 2:
            rpm_adjustment = 0.0  # minimizes vehicles surge movement to prioritize heading control

            # call heading control function
            # Returns a dictionary of thruster messages w/ Float 64 data type
            depth_heading_thruster_msgs = depth_heading_updater(depth_PID_output[0], heading_PID_output[0],
                                                                stability_thrust, rpm_adjustment)

        else:  # when depth error is small, prioritize heading control
            # prioritizes vehicles surge movement when heading error is below bailout threshold
            rpm_adjustment = 30.0

            # call heading control function
            # Returns a dictionary of thruster messages w/ Float 64 data type
            depth_heading_thruster_msgs = depth_heading_updater(depth_PID_output[0], heading_PID_output[0], stability_thrust,
                                                                rpm_adjustment)

        # send heading prioritized methods
        vert_port_thruster_pub.publish(depth_heading_thruster_msgs['vert_port'])
        vert_stbd_thruster_pub.publish(depth_heading_thruster_msgs['vert_stbd'])
        bow_port_thruster_pub.publish(depth_heading_thruster_msgs['bow_port'])
        bow_stbd_thruster_pub.publish(depth_heading_thruster_msgs['bow_stbd'])
        aft_port_thruster_pub.publish(depth_heading_thruster_msgs['aft_port'])
        aft_stbd_thruster_pub.publish(depth_heading_thruster_msgs['aft_stbd'])

        # Evaluate if within desired waypoint radius
        # If so, move to next waypoint
        current_distance_to_next_waypoint = np.sqrt((body_x - WAYPOINT_x[jj+1])**2 + (body_y - WAYPOINT_y[jj+1])**2)
        if current_distance_to_next_waypoint < 2:
            jj += 1

        # Update plotting arrays, appending the current values
        data_storage['plot_time'] = np.append(data_storage['plot_time'], current_time)
        data_storage['body_x_data'] = np.append(data_storage['body_x_data'], body_x)
        data_storage['body_y_data'] = np.append(data_storage['body_y_data'], body_y)
        data_storage['body_depth_data'] = np.append(data_storage['body_depth_data'], body_depth)
        data_storage['desired_depth_data'] = np.append(data_storage['desired_depth_data'], desired_depth)
        data_storage['depth_error_data'] = np.append(data_storage['depth_error_data'], depth_PID_output[1])
        data_storage['body_heading_data'] = np.append(data_storage['body_heading_data'], body_heading)
        data_storage['desired_heading_data'] = np.append(data_storage['desired_heading_data'], desired_heading)
        data_storage['heading_error_data'] = np.append(data_storage['heading_error_data'], heading_PID_output[1])

        rate.sleep()  # delays loop so it runs at desired rate
except rospy.ROSInterruptException:  # handles ROS shutdown requests Ctrl+C or rosnode kill etc.
    rospy.loginfo("ROS shutdown request received.")
except Exception as e:  # handles and displays unexpected errors that aren't keyboard interrupts or system exits
    rospy.logerr("Unhandled exception: {}".format(e))
finally:
    # Save and plot final data
    rospy.loginfo("Shutting down, saving data...")
    
    # Plotting
    fig = plt.figure(figsize=(12, 12))

    # Top half for navigation
    ax_nav = fig.add_subplot(3, 2, (1, 2))
    ax_nav.plot(WAYPOINTS[:, 0], WAYPOINTS[:, 1], 'ro-', label='Waypoints')
    vehicle_pos, = ax_nav.plot(data_storage['body_y_data'], data_storage['body_x_data'], 'b.', markersize=2,
                               label='Vehicle Position')
    ax_nav.set_xlim(0, 60)
    ax_nav.set_ylim(0, 60)
    ax_nav.set_title('Vehicle Navigation')
    ax_nav.set_xlabel('Y Position (m)')
    ax_nav.set_ylabel('X Position (m)')
    ax_nav.legend()

    # Bottom left for Depth and Depth Error
    ax_depth = fig.add_subplot(3, 2, 3)
    line_depth, = ax_depth.plot(data_storage['plot_time'], data_storage['body_depth_data'], 'r-', label='Fusion Depth')
    line_depth_desired, = ax_depth.plot(data_storage['plot_time'], data_storage['desired_depth_data'], 'b--', label='Desired Depth')
    ax_depth.set_title('Depth Tracking')
    ax_depth.set_xlabel('Time (s)')
    ax_depth.set_ylabel('Depth (m)')
    ax_depth.legend()
    # Depth Error Plot
    ax_depth_error = fig.add_subplot(3, 2, 5)
    line_depth_error, = ax_depth_error.plot(data_storage['plot_time'], data_storage['depth_error_data'], 'g-', label='Depth Error')
    ax_depth_error.axhline(0, color='k', linestyle='--', label='Zero Error')  # Zero error line
    ax_depth_error.set_title('Depth Error')
    ax_depth_error.set_xlabel('Time (s)')
    ax_depth_error.set_ylabel('Error (m)')
    ax_depth_error.legend()

    # Bottom right for Heading and Heading Error
    ax_heading = fig.add_subplot(3, 2, 4)
    line_heading, = ax_heading.plot(data_storage['plot_time'], data_storage['body_heading_data'], 'm-', label='Vehicle Heading')
    line_heading_desired, = ax_heading.plot(data_storage['plot_time'], data_storage['desired_heading_data'], 'k--', label='Desired Heading')
    ax_heading.set_title('Heading Tracking')
    ax_heading.set_xlabel('Time (s)')
    ax_heading.set_ylabel('Heading (degrees)')
    ax_heading.legend()
    # Heading Error Plot
    ax_heading_error = fig.add_subplot(3, 2, 6)
    line_heading_error, = ax_heading_error.plot(data_storage['plot_time'], data_storage['heading_error_data'], 'c-', label='Heading Error')
    ax_heading_error.axhline(0, color='r', linestyle='--', label='Zero Error')  # Zero error line
    ax_heading_error.set_title('Heading Error')
    ax_heading_error.set_xlabel('Time (s)')
    ax_heading_error.set_ylabel('Error (degrees)')
    ax_heading_error.legend()

    fig.subplots_adjust(hspace=0.5, wspace=0.4)  # Adjust horizontal and vertical spacing

    plt.show()

    # Create a filename with ros_rate and PID gains
    filename = "results__ROSrate{}_KpD{}_KiD{}_KdD{}_KpH{}_KiH{}_KdH{}.npy".format(
        desired_rate,
        kp_depth, ki_depth, kd_depth,
        kp_heading, ki_heading, kd_heading
    )
    print(data_storage.keys())
    # Save data to numpy binary file for later analysis if needed
    np.savez(filename, plot_time=data_storage['plot_time'],
             body_x_data=data_storage['body_x_data'],
             body_y_data=data_storage['body_y_data'],
             WAYPOINTS=WAYPOINTS,
             body_depth_data=data_storage['body_depth_data'],
             desired_depth_data=data_storage['desired_depth_data'],
             body_heading_data=data_storage['body_heading_data'],
             desired_heading_data=data_storage['desired_heading_data'],
             depth_error_data=data_storage['depth_error_data'],
             heading_error_data=data_storage['heading_error_data'])

