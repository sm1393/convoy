#!/usr/bin/env python3

import sys

import numpy as np
from cv_bridge import CvBridge
import cv2
import time
import imutils

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import Image
from navigation.msg import navigation

class robotInfo:
    def __init__(self, id):
        self.id = id
        self.pose = np.array([0.0, 0.0])
        rospy.Subscriber("/volta_" + str(self.id) + "/amcl_pose", PoseWithCovarianceStamped, self.poseCallback)

    def poseCallback(self, msg):
        self.pose[0] = msg.pose.pose.position.x
        self.pose[1] = msg.pose.pose.position.y

class Camera:
    def __init__(self, noOfRobots):
        self.robotsAroundMe = set()
        self.frontCamera = False
        self.leftCamera = False
        self.rightCamera = False
        self.noOfRobots = noOfRobots
        
        self.arucoDict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_250)
        self.arucoParams = cv2.aruco.DetectorParameters()
        self.bridge = CvBridge()

    def frontCameraCallback(self, msg):
        image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        corners, ids, rejected = cv2.aruco.detectMarkers(image, self.arucoDict, parameters=self.arucoParams)
        if type(ids) != type(None):
            for id in ids:
                if id in range(self.noOfRobots):
                    self.robotsAroundMe.add(id[0])
        self.frontCamera = True

    def leftCameraCallback(self, msg):
        image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        corners, ids, rejected = cv2.aruco.detectMarkers(image, self.arucoDict, parameters=self.arucoParams)
        if type(ids) != type(None):
            for id in ids:
                if id in range(self.noOfRobots):
                    self.robotsAroundMe.add(id[0])
        self.leftCamera = True

    def rightCameraCallback(self, msg):
        image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        corners, ids, rejected = cv2.aruco.detectMarkers(image, self.arucoDict, parameters=self.arucoParams)
        if type(ids) != type(None):
            for id in ids:
                if id in range(self.noOfRobots):
                    self.robotsAroundMe.add(id[0])
        self.rightCamera = True

    def selectMyLeader(self, robotsInfo):
        while not self.frontCamera and self.leftCamera and self.rightCamera:
            pass
        self.frontCamera = False
        self.leftCamera = False
        self.rightCamera = False

        minDistance = np.inf
        myLeaderID = np.nan
        for robot in self.robotsAroundMe:
            if robot == 0:
                return 0
            distance = np.linalg.norm(robotsInfo[robot].pose - robotsInfo[robotID].pose)
            if distance < minDistance:
                minDistance = distance
                myLeaderID = robot
        self.robotsAroundMe = set({})
        return myLeaderID

if __name__ == '__main__':
    try:
        robotID = int(rospy.myargv(argv=sys.argv)[1])
        rospy.init_node('volta_' + str(robotID) + '_camera_node')

        noOfRobots = 4
        camera = Camera(noOfRobots)

        rospy.Subscriber("/volta_" + str(robotID) + "/camera_front_ir/camera_front/color/image_raw", Image, camera.frontCameraCallback)
        rospy.Subscriber("/volta_" + str(robotID) + "/camera_left_ir/camera_left/color/image_raw", Image, camera.leftCameraCallback)
        rospy.Subscriber("/volta_" + str(robotID) + "/camera_right_ir/camera_right/color/image_raw", Image, camera.rightCameraCallback)

        robotsInfo = []
        for id in range(noOfRobots):
            robotsInfo.append(robotInfo(id))

        time.sleep(1)

        while not rospy.is_shutdown():
            if camera.frontCamera and camera.leftCamera and camera.rightCamera:
                robotsAroundMeCopy = camera.robotsAroundMe.copy()
                print(robotsAroundMeCopy)
                camera.frontCamera = False
                camera.leftCamera = False
                camera.rightCamera = False
                robotsAroundMeCopy = set({})
                robotsAroundMe = set({})

    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")
