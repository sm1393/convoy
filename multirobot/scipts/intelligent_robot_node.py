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
import multirobot.srv

arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_5X5_250)
arucoParams = cv2.aruco.DetectorParameters_create()

bridge = CvBridge()
navigationMsg = navigation()

robotID = int(rospy.myargv(argv=sys.argv)[1])

rospy.init_node('volta_' + str(robotID) + '_camera_node')

noOfRobots = 4
robotsAroundMe = set()
robotsAroundMeCopied = set()
frontCamera = False
leftCamera = False
rightCamera = False

underDiscussion = False

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
            if id in range(0,10):
                robotsAroundMe.add(id[0])
    frontCamera = True

def leftCameraCallback(msg):
    global leftCamera, robotsAroundMe
    image = bridge.imgmsg_to_cv2(msg, "bgr8")
    corners, ids, rejected = cv2.aruco.detectMarkers(image, arucoDict, parameters=arucoParams)
    image = imutils.resize(image, width=1000)
    if type(ids) != type(None):
        for id in ids:
            if id in range(0,10):
                robotsAroundMe.add(id[0])
    leftCamera = True

def rightCameraCallback(msg):
    global rightCamera, robotsAroundMe
    image = bridge.imgmsg_to_cv2(msg, "bgr8")
    corners, ids, rejected = cv2.aruco.detectMarkers(image, arucoDict, parameters=arucoParams)
    image = imutils.resize(image, width=1000)
    if type(ids) != type(None):
        for id in ids:
            if id in range(0,10):
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

def lockCheckCallback(req):
    global navigationMsg
    while underDiscussion:
        print("Waiting for previous lock to resolve")
        pass
    if req.leaderID == navigationMsg.leaderID:
        if np.linalg.norm(robotsInfo[navigationMsg.leaderID].pose - robotsInfo[robotID].pose) < req.distance_from_leader:
            return multirobot.srv.lock_checkResponse(req.leaderID == navigationMsg.leaderID, True)
        else:
            navigationMsg.leaderID = req.myID
            return multirobot.srv.lock_checkResponse(req.leaderID == navigationMsg.leaderID, False)
    else:
        # Yet to be decided
        return multirobot.srv.lock_checkResponse(req.leaderID == navigationMsg.leaderID, False)

if __name__ == '__main__':
    try:
        pub = rospy.Publisher("/volta_" + str(robotID) + "/navigation", navigation, queue_size=10)
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

        rospy.Service('lock_check_' + str(robotID), multirobot.srv.lock_check, lockCheckCallback)

        collisionPossible = False
        while not rospy.is_shutdown():
            if frontCamera and leftCamera and rightCamera:
                robotsAroundMeCopied = robotsAroundMe.copy()
                print("List out of index:", robotsAroundMeCopied, ", ", robotsAroundMe)
                for id in robotsAroundMeCopied:
                    if id != navigationMsg.leaderID:
                        distance = np.linalg.norm(robotsInfo[id].pose - robotsInfo[robotID].pose)
                        if distance < 1.5:
                            navigationMsg.flag = False
                            rospy.wait_for_service('lock_check_' + str(id))
                            lock_check = rospy.ServiceProxy('lock_check_' + str(id), multirobot.srv.lock_check)
                            lock_check_return = lock_check(robotID, navigationMsg.leaderID, np.linalg.norm(robotsInfo[navigationMsg.leaderID].pose - robotsInfo[robotID].pose))
                            print(navigationMsg.flag, "Asking", id, "->", lock_check_return.same_leaderID_confirmation, lock_check_return.update_your_leader)
                            if lock_check_return.same_leaderID_confirmation:
                                if lock_check_return.update_your_leader:
                                    navigationMsg.leaderID = id
                            collisionPossible = True
                        if not collisionPossible:
                            navigationMsg.flag = True
                collisionPossible = False
                frontCamera = False
                leftCamera = False
                rightCamera = False
                robotsAroundMe = set({})
                robotsAroundMeCopied = set({})
            pub.publish(navigationMsg)
            navigationMsg.flag = True

    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")
