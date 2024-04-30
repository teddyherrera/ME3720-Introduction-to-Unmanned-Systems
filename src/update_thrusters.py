from std_msgs.msg import Float64


# update_thrusters.py
def update_thrusters(depth_control_signal, heading_control_signal):
    # Set Max/Min RPMs
    max_thruster_cmd = 1200
    min_thruster_cmd = -1200
    # Calculate thruster commands and clamp them
    bow_port_command = max(min(depth_control_signal + heading_control_signal, max_thruster_cmd/2.0),
                           min_thruster_cmd)
    bow_stbd_command = max(min(depth_control_signal - heading_control_signal, max_thruster_cmd/2.0),
                           min_thruster_cmd)
    vert_port_command = max(min(depth_control_signal, max_thruster_cmd), min_thruster_cmd)
    vert_stbd_command = max(min(depth_control_signal, max_thruster_cmd), min_thruster_cmd)

    # Returning a dictionary of ROS messages for each thruster
    return {
        'bow_port': Float64(data=bow_port_command),
        'bow_stbd': Float64(data=bow_stbd_command),
        'vert_port': Float64(data=vert_port_command),
        'vert_stbd': Float64(data=vert_stbd_command)
    }
