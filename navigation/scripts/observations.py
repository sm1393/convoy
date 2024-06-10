#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist

rospy.init_node("convoy_observation")
velocity_publisher = rospy.Publisher("/volta_" + str(robotID) + "/cmd_vel", Twist, queue_size=10)

if __name__ == '__main__':
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")
