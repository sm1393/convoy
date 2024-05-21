#!/usr/bin/env python3

import sys

import numpy as np
import time

import rospy
import actionlib
from geometry_msgs.msg import PoseWithCovarianceStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from navigation.msg import navigation

myGoal = PoseWithCovarianceStamped()
navigationControl = False
isRobotDeviated = False

robotID = int(rospy.myargv(argv=sys.argv)[1])
myLeaderID = int(rospy.myargv(argv=sys.argv)[2])

rospy.init_node('volta_' + str(robotID) + '_navigation_node')
client = actionlib.SimpleActionClient('/volta_' + str(robotID) + '/move_base',MoveBaseAction)
client.wait_for_server()
goal = MoveBaseGoal()
goal.target_pose.header.frame_id = 'map'

leaderPrevPose = np.array([np.inf, np.inf])

leaderPose = np.array([0.0, 0.0])
prevLeaderPose = np.array([0.0, 0.0])
myGoal = PoseWithCovarianceStamped()
def leaderPoseeCallback(msg):
    global leaderPose, myGoal
    myGoal = msg
    leaderPose[0] = msg.pose.pose.position.x
    leaderPose[1] = msg.pose.pose.position.y

myPose = np.array([0.0, 0.0])
def myPoseCallback(msg):
    global myPose, covariance
    myPose[0] = msg.pose.pose.position.x
    myPose[1] = msg.pose.pose.position.y    

def navigationCallback(msg):
    global navigationControl, isRobotDeviated
    navigationControl = msg.flag
    isRobotDeviated = msg.deviated

if __name__ == '__main__':
    try:
        rospy.Subscriber("/volta_" + str(robotID) + "/amcl_pose", PoseWithCovarianceStamped, myPoseCallback)
        rospy.Subscriber("/volta_" + str(myLeaderID) + "/amcl_pose", PoseWithCovarianceStamped, leaderPoseeCallback)
        rospy.Subscriber("/volta_" + str(robotID) + "/navigation", navigation, navigationCallback)

        time.sleep(1)
        print(robotID, "following", myLeaderID)

        leaderPrevPose = np.copy(leaderPose)
        while not rospy.is_shutdown():
            goal.target_pose.header.stamp = rospy.Time.now()
            goal.target_pose.pose.position.x = leaderPose[0]
            goal.target_pose.pose.position.y = leaderPose[1]
            goal.target_pose.pose.position.z = myGoal.pose.pose.position.z
            goal.target_pose.pose.orientation.x = myGoal.pose.pose.orientation.x
            goal.target_pose.pose.orientation.y = myGoal.pose.pose.orientation.y
            goal.target_pose.pose.orientation.z = myGoal.pose.pose.orientation.z
            goal.target_pose.pose.orientation.w = myGoal.pose.pose.orientation.w
            if np.linalg.norm(leaderPose - leaderPrevPose) > 1:
                if np.linalg.norm(leaderPose - myPose) > 1:
                    if client.get_state() == 1 and not navigationControl:
                        client.cancel_all_goals()
                        client.send_goal(goal)
                    else:
                        client.send_goal(goal)
                else:
                    client.stop_tracking_goal()
                    client.cancel_all_goals()
                leaderPrevPose = np.copy(leaderPose)

    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")
