#!/usr/bin/env python3

import sys

import numpy as np
from cv_bridge import CvBridge
import cv2
import time

import rospy
import actionlib
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import Image, CompressedImage
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_5X5_250)
arucoParams = cv2.aruco.DetectorParameters_create()

myGoal = PoseWithCovarianceStamped()
bridge = CvBridge()

robotID = int(rospy.myargv(argv=sys.argv)[1])

rospy.init_node('volta_' + str(robotID) + '_node')
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

def frontCameraCallback(msg):
    global frontCamera, robotsAroundMe
    image = bridge.imgmsg_to_cv2(msg, "bgr8")
    corners, ids, rejected = cv2.aruco.detectMarkers(image, arucoDict, parameters=arucoParams)
    if type(ids) != type(None):
        for id in ids:
            robotsAroundMe.add(id[0])
    frontCamera = True

def leftCameraCallback(msg):
    global leftCamera, robotsAroundMe
    image = bridge.imgmsg_to_cv2(msg, "bgr8")
    corners, ids, rejected = cv2.aruco.detectMarkers(image, arucoDict, parameters=arucoParams)
    if type(ids) != type(None):
        for id in ids:
            robotsAroundMe.add(id[0])
    leftCamera = True

def rightCameraCallback(msg):
    global rightCamera, robotsAroundMe
    image = bridge.imgmsg_to_cv2(msg, "bgr8")
    corners, ids, rejected = cv2.aruco.detectMarkers(image, arucoDict, parameters=arucoParams)
    if type(ids) != type(None):
        for id in ids:
            robotsAroundMe.add(id[0])
    rightCamera = True

def selectMyLeader():
    global robotsAroundMe, frontCamera, leftCamera, rightCamera
    while not frontCamera and leftCamera and rightCamera:
        pass
    frontCamera = False
    leftCamera = False
    rightCamera = False

    minDistance = np.inf
    myLeaderID = np.nan
    for robot in robotsAroundMe:
        if robot == 0:
            return 0
        distance = np.linalg.norm(robotsInfo[robot].pose - robotsInfo[robotID].pose)
        if distance < minDistance:
            minDistance = distance
            myLeaderID = robot
    robotsAroundMe = set({})
    return myLeaderID

myLeaderPrevPose = np.array([np.inf, np.inf])

if __name__ == '__main__':
    try:
        rospy.Subscriber("/volta_" + str(robotID) + "/camera_front/camera_front", Image, frontCameraCallback)
        rospy.Subscriber("/volta_" + str(robotID) + "/camera_left/camera_left", Image, leftCameraCallback)
        rospy.Subscriber("/volta_" + str(robotID) + "/camera_right/camera_right", Image, rightCameraCallback)

        robotsInfo = []
        for id in range(noOfRobots):
            robotsInfo.append(robotInfo(id))

        time.sleep(1)
        myLeaderID = selectMyLeader()
        print("myLeaderID = ", myLeaderID)
        print("Ready to GO !!!")

        while not rospy.is_shutdown():
            if frontCamera and leftCamera and rightCamera:
                print("Robots around me: ", robotsAroundMe)
                frontCamera = False
                leftCamera = False
                rightCamera = False
                robotsAroundMe = set({})
            if np.linalg.norm(robotsInfo[myLeaderID].pose - robotsInfo[robotID].pose) > 1:
                if np.linalg.norm(robotsInfo[myLeaderID].pose - myLeaderPrevPose) > 1:
                    goal.target_pose.header.stamp = rospy.Time.now()
                    goal.target_pose.pose.position.x = myGoal.pose.pose.position.x
                    goal.target_pose.pose.position.y = myGoal.pose.pose.position.y
                    goal.target_pose.pose.position.z = myGoal.pose.pose.position.z
                    goal.target_pose.pose.orientation.x = myGoal.pose.pose.orientation.x
                    goal.target_pose.pose.orientation.y = myGoal.pose.pose.orientation.y
                    goal.target_pose.pose.orientation.z = myGoal.pose.pose.orientation.z
                    goal.target_pose.pose.orientation.w = myGoal.pose.pose.orientation.w
                    client.send_goal(goal)
                    myLeaderPrevPose = np.copy(robotsInfo[myLeaderID].pose)
            else:
                client.stop_tracking_goal()
                client.cancel_all_goals()

    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")
