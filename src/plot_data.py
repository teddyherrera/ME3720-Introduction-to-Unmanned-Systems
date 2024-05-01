import matplotlib.pyplot as plt
import numpy as np


def plot_data(plot_time, actual_depth_data, desired_depth_data, actual_heading_data, desired_heading_data, depth_error_data, heading_error_data, control_signal_data, heading_control_signal_data):
    # Heading Plot
    plt.figure(1, figsize=(10, 12))  # Set the figure size
    # Plot Depth and Depth Error
    ax1 = plt.subplot(311)  # 3 rows, 1 columns, 1st subplot
    ax1.plot(plot_time, actual_depth_data, label='Actual Depth')
    ax1.plot(plot_time, desired_depth_data, 'r--', label='Desired Depth')
    ax1.set_xlabel('Time (s)')
    ax1.set_ylabel('Depth (m)')
    ax1.set_title('Depth Tracking')
    ax1.legend()

    ax2 = plt.subplot(312)  # 2rd subplot
    ax2.plot(plot_time, depth_error_data, label='Depth Error')
    ax2.set_xlabel('Time (s)')
    ax2.set_ylabel('Error (m)')
    ax2.set_title('Depth Error')
    ax2.legend()

    ax3 = plt.subplot(313)  # 3rd subplot
    ax3.plot(plot_time, control_signal_data, label='Depth Control Signal')
    ax3.set_xlabel('Time (s)')
    ax3.set_ylabel('Control Signal')
    ax3.set_title('Depth Control Signal Output')
    ax3.legend()

    plt.tight_layout()
    plt.savefig('depth_plots.png')
    plt.show()

    # Plot Heading and Heading Error
    plt.figure(2, figsize=(10,12))
    ax4 = plt.subplot(311)
    ax4.plot(plot_time, actual_heading_data, label='Actual Heading')
    ax4.plot(plot_time, desired_heading_data, 'r--', label='Desired Heading')
    ax4.set_xlabel('Time (s)')
    ax4.set_ylabel('Heading (Degrees)')
    ax4.set_title('Heading Tracking')
    ax4.legend()

    ax5 = plt.subplot(312)
    ax5.plot(plot_time, heading_error_data, label='Heading Error')
    ax5.set_xlabel('Time (s)')
    ax5.set_ylabel('Error (Degrees)')
    ax5.set_title('Heading Error')
    ax5.legend()

    ax6 = plt.subplot(313)  # 6th subplot
    ax6.plot(plot_time, heading_control_signal_data, label='Heading Control Signal')
    ax6.set_xlabel('Time (s)')
    ax6.set_ylabel('Control Signal')
    ax6.set_title('Heading Control Signal Output')
    ax6.legend()

    plt.tight_layout()
    plt.savefig('heading_plots.png')
    plt.show()
