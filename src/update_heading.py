from std_msgs.msg import Float64

def update_heading(depth_control_signal, heading_control_signal, stability_thrust, rpm_adjustment):

    # Define maximum RPM limits
    max_rpm_depth = 400
    heading_control_signal = max(min(heading_control_signal, 25), -25)

    # Apply constraints to thruster commands to be within +/- 1200 RPM
    vert_port_command = max(min(-depth_control_signal, max_rpm_depth), -max_rpm_depth)  # Positive RPMs send down
    vert_stbd_command = max(min(depth_control_signal, max_rpm_depth), -max_rpm_depth)  # Positive RPMs send up

    bow_port_command = -stability_thrust + heading_control_signal  # Negative for clockwise due to positive RPMs send CCW
    bow_stbd_command = stability_thrust + heading_control_signal   # Positive sends CCW, need negative for CW
    aft_port_command = -stability_thrust + rpm_adjustment
    aft_stbd_command = stability_thrust - rpm_adjustment

    return {
        'vert_port': Float64(data=vert_port_command),
        'vert_stbd': Float64(data=vert_stbd_command),
        'bow_port': Float64(data=bow_port_command),
        'bow_stbd': Float64(data=bow_stbd_command),
        'aft_port': Float64(data=aft_port_command),
        'aft_stbd': Float64(data=aft_stbd_command)
    }
