from std_msgs.msg import Float64


def set_depth(depth_control_signal, heading_control_signal, stability_thrust):
    """
    Prioritize depth control with some heading control enabled.
    Parameters:
        depth_control_signal: Depth control signal. Used for controlling vehicle's depth.
        heading_control_signal: Heading control signal. Used for controlling vehicle's heading.
        stability_thrust: Stability thrust value. Used so RPMs are in a range to affect vehicle dynamics.
    Output:
        :return: Dictionary containing updated thruster commands.
    """
    # Assign maximum RPM limits
    max_rpm = 1200.0
    # Limit heading control signal between -20 and 20 RPMs
    heading_control_signal = max(min(heading_control_signal, 20.0), -20.0)

    # Apply constraints to thruster commands to be within +/- 1200 RPM
    vert_port_command = max(min(-depth_control_signal, max_rpm), -max_rpm)  # Positive RPMs send down
    vert_stbd_command = max(min(depth_control_signal, max_rpm), -max_rpm)   # Positive RPMs send up

    bow_port_command = -stability_thrust + heading_control_signal # Negative for clockwise due to positive RPMs send CCW
    bow_stbd_command = stability_thrust + heading_control_signal   # Positive sends CCW, need negative for CW
    aft_port_command = -stability_thrust
    aft_stbd_command = stability_thrust


    return {
        'vert_port': Float64(data=vert_port_command),
        'vert_stbd': Float64(data=vert_stbd_command),
        'bow_port': Float64(data=bow_port_command),
        'bow_stbd': Float64(data=bow_stbd_command),
        'aft_port': Float64(data=aft_port_command),
        'aft_stbd': Float64(data=aft_stbd_command)
    }