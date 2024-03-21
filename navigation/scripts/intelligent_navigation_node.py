#!/usr/bin/env python3

import sys

import numpy as np
from cv_bridge import CvBridge
import cv2

import rospy
import actionlib
from geometry_msgs.msg import PoseWithCovarianceStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from navigation.msg import navigation

arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_5X5_250)
arucoParams = cv2.aruco.DetectorParameters_create()

myGoal = PoseWithCovarianceStamped()
bridge = CvBridge()

robotID = int(rospy.myargv(argv=sys.argv)[1])

rospy.init_node('volta_' + str(robotID) + '_navigation_node')
client = actionlib.SimpleActionClient('/volta_' + str(robotID) + '/move_base',MoveBaseAction)
client.wait_for_server()
goal = MoveBaseGoal()
goal.target_pose.header.frame_id = 'map'

noOfRobots = 4
myLeaderID = 0
robotsAroundMe = set()
frontCamera = False
leftCamera = False
rightCamera = False

class robotInfo:
    def __init__(self, id):
        self.id = id
        self.pose = np.array([0.0, 0.0])
        rospy.Subscriber("/volta_" + str(self.id) + "/amcl_pose", PoseWithCovarianceStamped, self.poseCallback)

    def poseCallback(self, msg):
        global myGoal, myLeaderID
        if self.id == myLeaderID:
            myGoal = msg
        self.pose[0] = msg.pose.pose.position.x
        self.pose[1] = msg.pose.pose.position.y

leaderPrevPose = np.array([np.inf, np.inf])

flag = True
myLeaderID = np.inf
def navigationCallback(msg):
    global flag, myLeaderID 
    flag = msg.flag
    myLeaderID = msg.leaderID
    rospy.loginfo(msg)

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
    global myPose
    myPose[0] = msg.pose.pose.position.x
    myPose[1] = msg.pose.pose.position.y    

if __name__ == '__main__':
    try:
        myLeaderID == np.inf
        rospy.Subscriber("/volta_" + str(robotID) + "/navigation", navigation, navigationCallback)
        while myLeaderID == np.inf:
            pass
        print("My leader is ", myLeaderID)
        print("Ready to GO !!!")
        rospy.Subscriber("/volta_" + str(robotID) + "/amcl_pose", PoseWithCovarianceStamped, myPoseCallback)
        rospy.Subscriber("/volta_" + str(myLeaderID) + "/amcl_pose", PoseWithCovarianceStamped, leaderPoseeCallback)

        while not rospy.is_shutdown():
            if np.linalg.norm(leaderPose - myPose) > 1.0:
                if np.linalg.norm(leaderPose - leaderPrevPose) > 1.0:
                    goal.target_pose.header.stamp = rospy.Time.now()
                    goal.target_pose.pose.position.x = myGoal.pose.pose.position.x
                    goal.target_pose.pose.position.y = myGoal.pose.pose.position.y
                    goal.target_pose.pose.position.z = myGoal.pose.pose.position.z
                    goal.target_pose.pose.orientation.x = myGoal.pose.pose.orientation.x
                    goal.target_pose.pose.orientation.y = myGoal.pose.pose.orientation.y
                    goal.target_pose.pose.orientation.z = myGoal.pose.pose.orientation.z
                    goal.target_pose.pose.orientation.w = myGoal.pose.pose.orientation.w
                    client.send_goal(goal)
                    leaderPrevPose = np.copy(leaderPose)
            else:
                client.stop_tracking_goal()
                client.cancel_all_goals()

    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")
