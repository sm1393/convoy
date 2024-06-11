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

if __name__ == '__main__':
    try:
        robotID = int(rospy.myargv(argv=sys.argv)[1])
        rospy.init_node('volta_' + str(robotID) + '_camera_node')

        noOfRobots = 4
        camera = Camera(noOfRobots)

        rospy.Subscriber("/volta_" + str(robotID) + "/camera_front_ir/camera_front/color/image_raw", Image, camera.frontCameraCallback)
        time.sleep(1)

        while not rospy.is_shutdown():
            if camera.frontCamera:
                robotsAroundMeCopy = camera.robotsAroundMe.copy()
                print("robotsAroundMeCopy", robotsAroundMeCopy)
                camera.frontCamera = False
                robotsAroundMe = set({})
                robotsAroundMeCopy = set({})

    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")
