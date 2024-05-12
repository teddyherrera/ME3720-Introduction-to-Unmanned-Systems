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
    body_x = msg.pose.pose.pose.position.x
    body_y = msg.pose.pose.pose.position.y
    body_depth = msg.pose.pose.position.z
    fusion_state_sub = np.array([body_x, body_y, body_depth, body_roll, body_pitch, body_heading])

# subscriber function
rospy.Subscriber('/fusion/pose_gt', Odometry, fusion_state_callback, queue_size=desired_rate)

rospy.sleep(1.0)  # needed so that the callback function can pull the initial state before moving on

# PID control setup
rate = rospy.Rate(desired_rate)  # 4 Hz
dt = 1.0 / desired_rate

# Setup plot
plt.ion()  # Turn on interactive mode
fig = plt.figure(figsize=(10, 8))

# Top half for navigation
ax_nav = fig.add_subplot(3, 2, (1,2))
ax_nav.plot(waypoints[:, 0], waypoints[:, 1], 'ro-', label='Waypoints')
vehicle_pos, = ax_nav.plot([], [], 'bo', label='Vehicle Position')
ax_nav.set_xlim([0, 60])
ax_nav.set_ylim([0, 60])
ax_nav.set_title('Vehicle Navigation')
ax_nav.set_xlabel('Y Position (m)')
ax_nav.set_ylabel('X Position (m)')
ax_nav.legend()

# Bottom left for Depth
ax_depth = fig.add_subplot(3, 2, 3)
line_depth, = ax_depth.plot([], [], 'r-', label='Actual Depth')
line_depth_desired, = ax_depth.plot([], [], 'b--', label='Desired Depth')
ax_depth.set_title('Depth Tracking')
ax_depth.set_xlabel('Time (s)')
ax_depth.set_ylabel('Depth (m)')
ax_depth.legend()

ax_depth_error = fig.add_subplot(3, 2, 4)
line_depth_error, = ax_depth_error.plot([], [], 'g-', label='Depth Error')
ax_depth_error.axhline(0, color='k', linestyle='--', label='Zero Error')  # Zero error line
ax_depth_error.set_title('Depth Error')
ax_depth_error.set_xlabel('Time (s)')
ax_depth_error.set_ylabel('Error (m)')
ax_depth_error.legend()

# Bottom right for Heading
ax_heading = fig.add_subplot(3, 2, 5)
line_heading, = ax_heading.plot([], [], 'm-', label='Actual Heading')
line_heading_desired, = ax_heading.plot([], [], 'k--', label='Desired Heading')
ax_heading.set_title('Heading Tracking')
ax_heading.set_xlabel('Time (s)')
ax_heading.set_ylabel('Heading (degrees)')
ax_heading.legend()

ax_heading_error = fig.add_subplot(3, 2, 6)
line_heading_error, = ax_heading_error.plot([], [], 'c-', label='Heading Error')
ax_heading_error.axhline(0, color='r', linestyle='--', label='Zero Error')  # Zero error line
ax_heading_error.set_title('Heading Error')
ax_heading_error.set_xlabel('Time (s)')
ax_heading_error.set_ylabel('Error (degrees)')
ax_heading_error.legend()

# Main control loop
plot_time = []
rho = 5  # look ahead distance
jj = 1
body_depth_data = []
desired_depth_data = []
depth_error_data = []
body_heading_data = []
desired_heading_data = []
heading_error_data = []
try:
    while not rospy.is_shutdown():

        current_time = rospy.get_time()
        plot_time.append(current_time)

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
        depth_PID_output = depth_PID.update(body_depth, desired_depth, dt)  # [0] - depth control signal [1] - error

        if abs(depth_PID_output[1]) >= 0.5:
            # call depth control function
            update_thrusters_depth = set_depth(depth_PID_output[0], stability_thrust)

            # send messages
            vert_port_thruster_pub.publish(thruster_msgs['vert_port'])
            vert_stbd_thruster_pub.publish(thruster_msgs['vert_stbd'])
            bow_port_thruster_pub.publish(thruster_msgs['bow_port'])
            bow_stbd_thruster_pub.publish(thruster_msgs['bow_stbd'])
            aft_port_thruster_pub.publish(thruster_msgs['aft_port'])
            aft_stbd_thruster_pub.publish(thruster_msgs['aft_stbd'])

        else:
            # call heading control function
            heading_PID_output = update_heading(body_heading, desired_heading, dt)

            # Bailout logic for large errors/turns
            if abs(heading_PID_output[1]) >= 5:
                rpm_adjustment = 0.0   # minimizes vehicles surge movement to prioritize heading control

                # call heading control function
                update_heading(depth_PID_output[0],heading_PID_output[0], stability_thrust, rpm_adjustment)

                # send messages
                vert_port_thruster_pub.publish(thruster_msgs['vert_port'])
                vert_stbd_thruster_pub.publish(thruster_msgs['vert_stbd'])
                bow_port_thruster_pub.publish(thruster_msgs['bow_port'])
                bow_stbd_thruster_pub.publish(thruster_msgs['bow_stbd'])
                aft_port_thruster_pub.publish(thruster_msgs['aft_port'])
                aft_stbd_thruster_pub.publish(thruster_msgs['aft_stbd'])

            else:
                rpm_adjustment = 30.0  # prioritizes vehicles surge movement when heading error is below the bailout threshold

                # call heading control function
                update_heading(depth_PID_output[0], heading_PID_output[0], stability_thrust, rpm_adjustment)

                # send messages
                vert_port_thruster_pub.publish(thruster_msgs['vert_port'])
                vert_stbd_thruster_pub.publish(thruster_msgs['vert_stbd'])
                bow_port_thruster_pub.publish(thruster_msgs['bow_port'])
                bow_stbd_thruster_pub.publish(thruster_msgs['bow_stbd'])
                aft_port_thruster_pub.publish(thruster_msgs['aft_port'])
                aft_stbd_thruster_pub.publish(thruster_msgs['aft_stbd'])

        # Update plotting arrays
        body_depth_data.append(body_depth)
        desired_depth_data.append(desired_depth)
        depth_error_data.append(depth_PID_output[1])
        body_heading_data.append(body_heading)
        desired_heading_data.append(desired_heading)
        heading_error_data.append(heading_PID_output[1])

        # Update plot data
        vehicle_pos.set_data(body_y, body_x)
        line_depth.set_data(plot_time, body_depth_data)
        line_depth_desired.set_data(plot_time, desired_depth_data)
        line_depth_error.set_data(plot_time, depth_error_data)

        line_heading.set_data(plot_time, body_heading_data)
        line_heading_desired.set_data(plot_time, desired_heading_data)
        line_heading_error.set_data(plot_time, heading_error_data)

        # Rescale plot to fit new data
        ax_depth.relim()
        ax_depth.autoscale_view()
        ax_depth_error.relim()
        ax_depth_error.autoscale_view()

        ax_heading.relim()
        ax_heading.autoscale_view()
        ax_heading_error.relim()
        ax_heading_error.autoscale_view()

        plt.draw()  # Redraw the plots
        plt.pause(0.1)  # Pause briefly to allow GUI events to process

        rate.sleep()
except rospy.ROSInterruptException:  # handles ROS shutdown requests Ctrl+C or rosnode kill etc.
    rospy.loginfo("ROS shutdown request received.")
except Exception as e:  # handles and displays unexpected errors that aren't keyboard interrupts or system exits
    rospy.logerr("Unhandled exception: {}".format(e))
finally:
    # Save and plot final data
    rospy.loginfo("Shutting down, saving data...")
    # End plotting
    plt.ioff()  # Turn off interactive mode
    plt.show()  # Keep the window open at the end

    # Create a filename with ros_rate and PID gains
   filename = "results__ROSrate{}_KpD{}_KiD{}_KdD{}_KpH{}_KiH{}_KdH{}.npy".format(
        desired_rate,
        kp_depth, ki_depth, kd_depth,
        kp_heading, ki_heading, kd_heading
    )
    # Save data to numpy binary file for later analysis
   np.savez(filename, plot_time=plot_time, body_depth_data=body_depth_data,
            desired_depth_data=desired_depth_data,
            body_heading_data=body_heading_data,
            desired_heading_data= desired_heading_data,
            depth_error_data=depth_error_data,
            heading_error_data=heading_error_data)

