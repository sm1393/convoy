#!/usr/bin/env python3

import rospy

rospy.init_node("convoy_observation")

if __name__ == '__main__':
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")
