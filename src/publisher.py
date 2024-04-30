#!usr/bin/env python
import rospy
from std_msgs.msg import Float64


def talker():
    bow_port_thruster_pub = rospy.Publisher('/bow_port_thruster', Float64, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(4)
    while not rospy.is_shutdown():
        test_message = 300.0
        rospy.loginfo(test_message)
        bow_port_thruster_pub.publish(test_message)
        rate.sleep()


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass


