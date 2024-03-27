#!/usr/bin/env python3

import sys

import numpy as np
from cv_bridge import CvBridge
import cv2
import time
import imutils

import rospy
import actionlib
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import Image
from navigation.msg import navigation
import multirobot.msg

arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_5X5_250)
arucoParams = cv2.aruco.DetectorParameters_create()

bridge = CvBridge()

robotID = int(rospy.myargv(argv=sys.argv)[1])

rospy.init_node('volta_' + str(robotID) + '_camera_node')

noOfRobots = 4
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
        self.pose[0] = msg.pose.pose.position.x
        self.pose[1] = msg.pose.pose.position.y

def frontCameraCallback(msg):
    global frontCamera, robotsAroundMe
    image = bridge.imgmsg_to_cv2(msg, "bgr8")
    corners, ids, rejected = cv2.aruco.detectMarkers(image, arucoDict, parameters=arucoParams)
    image = imutils.resize(image, width=1000)
    if type(ids) != type(None):
        for id in ids:
            robotsAroundMe.add(id[0])
    frontCamera = True

def leftCameraCallback(msg):
    global leftCamera, robotsAroundMe
    image = bridge.imgmsg_to_cv2(msg, "bgr8")
    corners, ids, rejected = cv2.aruco.detectMarkers(image, arucoDict, parameters=arucoParams)
    image = imutils.resize(image, width=1000)
    if type(ids) != type(None):
        for id in ids:
            robotsAroundMe.add(id[0])
    leftCamera = True

def rightCameraCallback(msg):
    global rightCamera, robotsAroundMe
    image = bridge.imgmsg_to_cv2(msg, "bgr8")
    corners, ids, rejected = cv2.aruco.detectMarkers(image, arucoDict, parameters=arucoParams)
    image = imutils.resize(image, width=1000)
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

def convoy_client(robotID):
    client = actionlib.SimpleActionClient('convoy_config_'+str(robotID), multirobot.msg.InitiateConvoyAction)
    client.wait_for_server()
    goal = multirobot.msg.InitiateConvoyGoal(myID=robotID)
    client.send_goal(goal)
    client.wait_for_result()
    return client.get_result()

if __name__ == '__main__':
    try:
        pub = rospy.Publisher("/volta_" + str(robotID) + "/navigation", navigation, queue_size=10)
        navigationMsg = navigation()
        rospy.Subscriber("/volta_" + str(robotID) + "/camera_front/camera_front", Image, frontCameraCallback)
        rospy.Subscriber("/volta_" + str(robotID) + "/camera_left/camera_left", Image, leftCameraCallback)
        rospy.Subscriber("/volta_" + str(robotID) + "/camera_right/camera_right", Image, rightCameraCallback)

        robotsInfo = []
        for id in range(noOfRobots):
            robotsInfo.append(robotInfo(id))

        time.sleep(1)
        navigationMsg.leaderID = selectMyLeader()
        navigationMsg.flag = False

        print(robotID, "following", navigationMsg.leaderID)

        result = convoy_client(robotID)
        if not result:
            print("Mission Abort!")
            exit()

        while not rospy.is_shutdown():
            if frontCamera and leftCamera and rightCamera:
                for id in robotsAroundMe:
                    distance = np.linalg.norm(robotsInfo[id].pose - robotsInfo[robotID].pose)
                    if distance < 1.5:
                        navigationMsg.flag = False
                        break
                pub.publish(navigationMsg)
                navigationMsg.flag = True
                frontCamera = False
                leftCamera = False
                rightCamera = False
                robotsAroundMe = set({})

    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")
