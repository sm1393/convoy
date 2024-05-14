#!/usr/bin/env python3

import sys

import numpy as np
from cv_bridge import CvBridge
import cv2

import rospy
from sensor_msgs.msg import Image

frontmin = 0
frontmax = 0
leftmin = 0
leftmax = 0
rightmin = 0
rightmax = 0

class Camera:
    def __init__(self, noOfRobots):
        self.robotsAroundMe = set()
        self.bridge = CvBridge()

    def frontCameraCallback(self, msg):
        global frontmin, frontmax
        image = self.bridge.imgmsg_to_cv2(msg, "32FC1")
        frontmin = np.nanmin(image)
        frontmax = np.nanmax(image)
        cv2.imshow("image front", image)
        cv2.waitKey(1)

    def leftCameraCallback(self, msg):
        global leftmin, leftmax
        image = self.bridge.imgmsg_to_cv2(msg, "32FC1")
        leftmin = np.nanmin(image)
        leftmax = np.nanmax(image)
        # cv2.imshow("image left", image)
        # cv2.waitKey(1)

    def rightCameraCallback(self, msg):
        global rightmin, rightmax
        image = self.bridge.imgmsg_to_cv2(msg, "32FC1")
        rightmin = np.nanmin(image)
        rightmax = np.nanmax(image)
        # cv2.imshow("image right", image)
        # cv2.waitKey(1)

if __name__ == '__main__':
    try:
        robotID = int(rospy.myargv(argv=sys.argv)[1])
        rospy.init_node('volta_' + str(robotID) + '_camera_node')

        noOfRobots = 2
        camera = Camera(noOfRobots)

        rospy.Subscriber("/volta_" + str(robotID) + "/camera_front_ir/camera_front/depth/image_raw", Image, camera.frontCameraCallback)
        rospy.Subscriber("/volta_" + str(robotID) + "/camera_left_ir/camera_left/depth/image_raw", Image, camera.leftCameraCallback)
        rospy.Subscriber("/volta_" + str(robotID) + "/camera_right_ir/camera_right/depth/image_raw", Image, camera.rightCameraCallback)

        while not rospy.is_shutdown():
            print(frontmin, frontmax, leftmin, leftmax, rightmin, rightmax)
            pass

    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")
