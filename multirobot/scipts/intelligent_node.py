#!/usr/bin/env python3

import sys

import numpy as np
import time

import rospy
import actionlib
from geometry_msgs.msg import PoseWithCovarianceStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import Path
from navigation.msg import navigation

myGoal = PoseWithCovarianceStamped()
navControl = navigation()
localPlanCopy = Path()
robotID = int(rospy.myargv(argv=sys.argv)[1])

rospy.init_node('volta_' + str(robotID) + '_intelligent_node')
client = actionlib.SimpleActionClient('/volta_' + str(robotID) + '/move_base',MoveBaseAction)
client.wait_for_server()

myPose = np.array([0.0, 0.0])
def myPoseCallback(msg):
    global myPose
    myPose[0] = msg.pose.pose.position.x
    myPose[1] = msg.pose.pose.position.y

def localPlanCallback(msg):
    global localPlanCopy
    localPlanCopy = msg

def navigationControl():
    global localPlanCopy, myPose
    pointOnLine = np.array([0.0, 0.0])
    minDeviationFromPath = np.inf
    for pose in localPlanCopy.poses:
        pointOnLine[0] = pose.pose.position.x
        pointOnLine[1] = pose.pose.position.y
        deviation = np.linalg.norm(pointOnLine - myPose)
        if deviation < minDeviationFromPath:
            minDeviationFromPath = deviation
        # print(minDeviationFromPath)
        if minDeviationFromPath > 0.25:
            return False
    return True

if __name__ == '__main__':
    try:
        rospy.Subscriber("/volta_" + str(robotID) + "/amcl_pose", PoseWithCovarianceStamped, myPoseCallback)
        rospy.Subscriber("/volta_" + str(robotID) + "/move_base/TebLocalPlannerROS/local_plan", Path, localPlanCallback)
        pub = rospy.Publisher("/volta_" + str(robotID) + "/navigation", navigation, queue_size=10)

        time.sleep(1)

        while not rospy.is_shutdown():
            navControl.flag = navigationControl()
            pub.publish(navControl)

    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")
