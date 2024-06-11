#!/usr/bin/env python3

import sys

import numpy as np
from cv_bridge import CvBridge
import cv2
import time
import imutils

import rospy
from sensor_msgs.msg import Image

class Camera:
    def __init__(self, noOfRobots):
        self.robotsAroundMe = set()
        self.frontCamera = False
        self.leftCamera = False
        self.rightCamera = False
        self.noOfRobots = noOfRobots
        
        self.arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_5X5_250)
        self.arucoParams = cv2.aruco.DetectorParameters_create()
        self.bridge = CvBridge()

    def frontCameraCallback(self, msg):
        image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        corners, ids, rejected = cv2.aruco.detectMarkers(image, self.arucoDict, parameters=self.arucoParams)
        image = imutils.resize(image, width=1000)
        if type(ids) != type(None):
            for id in ids:
                if id in range(self.noOfRobots):
                    self.robotsAroundMe.add(id[0])
        self.frontCamera = True

    def leftCameraCallback(self, msg):
        image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        corners, ids, rejected = cv2.aruco.detectMarkers(image, self.arucoDict, parameters=self.arucoParams)
        image = imutils.resize(image, width=1000)
        if type(ids) != type(None):
            for id in ids:
                if id in range(self.noOfRobots):
                    self.robotsAroundMe.add(id[0])
        self.leftCamera = True

    def rightCameraCallback(self, msg):
        image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        corners, ids, rejected = cv2.aruco.detectMarkers(image, self.arucoDict, parameters=self.arucoParams)
        image = imutils.resize(image, width=1000)
        if type(ids) != type(None):
            for id in ids:
                if id in range(self.noOfRobots):
                    self.robotsAroundMe.add(id[0])
        self.rightCamera = True

if __name__ == '__main__':
    try:
        robotID = int(rospy.myargv(argv=sys.argv)[1])
        rospy.init_node('volta_' + str(robotID) + '_camera_node')

        noOfRobots = 4
        camera = Camera(noOfRobots)

        rospy.Subscriber("/volta_" + str(robotID) + "/camera_front/camera_front", Image, camera.frontCameraCallback)
        rospy.Subscriber("/volta_" + str(robotID) + "/camera_left/camera_left", Image, camera.leftCameraCallback)
        rospy.Subscriber("/volta_" + str(robotID) + "/camera_right/camera_right", Image, camera.rightCameraCallback)

        time.sleep(1)

        while not rospy.is_shutdown():
            if camera.frontCamera and camera.leftCamera and camera.rightCamera:
                robotsAroundMeCopy = camera.robotsAroundMe.copy()
                print(robotsAroundMeCopy)
                collisionPossible = False
                camera.frontCamera = False
                camera.leftCamera = False
                camera.rightCamera = False
                robotsAroundMe = set({})
                robotsAroundMeCopy = set({})

    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")
