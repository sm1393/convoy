#!/usr/bin/env python3

import sys

import numpy as np
import time

import rospy


robotID = int(rospy.myargv(argv=sys.argv)[1])

rospy.init_node('volta_' + str(robotID) + '_intelligent_node')

myPose = np.array([0.0, 0.0])
def myPoseCallback(msg):
    global myPose
    myPose[0] = msg.pose.pose.position.x
    myPose[1] = msg.pose.pose.position.y    

if __name__ == '__main__':
    try:
        # rospy.Subscriber("/volta_" + str(robotID) + "/amcl_pose", PoseWithCovarianceStamped, myPoseCallback)

        time.sleep(1)

        while not rospy.is_shutdown():
            # print("Goal state = ", client.get_state())
            pass

    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")
