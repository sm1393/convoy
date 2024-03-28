#!/usr/bin/env python3

import sys

import numpy as np
from cv_bridge import CvBridge
import cv2
from cv2.aruco import detectMarkers
import time
from imutils import resize

import rospy
import actionlib

from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import Image

from navigation.msg import navigation
import multirobot.msg
import multirobot.srv

noOfRobots = 4
robotID = int(rospy.myargv(argv=sys.argv)[1])

class Camera:
    def __init__(self, noOfRobots):
        self.frontCamera = False
        self.leftCamera = False
        self.rightCamera = False
        self.robotsAroundMe = set()
        self.noOfRobots = noOfRobots
        self.bridge = CvBridge()
        self.arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_5X5_250)
        self.arucoParams = cv2.aruco.DetectorParameters_create()

    def addRobotsToList(self, ids):
        if type(ids) != type(None):
            for id in ids:
                if id in range(0, self.noOfRobots):
                    self.robotsAroundMe.add(id[0])

    def frontCameraCallback(self, msg):
        image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        corners, ids, rejected = detectMarkers(image, self.arucoDict, parameters = self.arucoParams)
        image = resize(image, width=1000)
        self.addRobotsToList(self, ids)
        self.frontCamera = True

    def leftCameraCallback(self, msg):
        image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        corners, ids, rejected = detectMarkers(image, self.arucoDict, parameters = self.arucoParams)
        image = resize(image, width=1000)
        self.addRobotsToList(self, ids)
        self.leftCamera = True

    def rightCameraCallback(self, msg):
        image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        corners, ids, rejected = detectMarkers(image, self.arucoDict, parameters = self.arucoParams)
        image = resize(image, width=1000)
        self.addRobotsToList(self, ids)
        rightCamera = True
    
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

class robotInfo:
    def __init__(self, id):
        self.id = id
        self.pose = np.array([0.0, 0.0])
        rospy.Subscriber("/volta_" + str(self.id) + "/amcl_pose", PoseWithCovarianceStamped, self.poseCallback)

    def poseCallback(self, msg):
        self.pose[0] = msg.pose.pose.position.x
        self.pose[1] = msg.pose.pose.position.y

class InterCommunication:
    def __init__(self, robotID, noOfRobots):
        self.robotID = robotID
        self.noOfRobots = noOfRobots
        self.navigationMsg = navigation()
        self.underDiscussion = False
        
    def convoy_client(self, robotID):
        client = actionlib.SimpleActionClient('convoy_config_'+str(self.robotID), multirobot.msg.InitiateConvoyAction)
        client.wait_for_server()
        goal = multirobot.msg.InitiateConvoyGoal(myID=self.robotID)
        client.send_goal(goal)
        client.wait_for_result()
        return client.get_result()

    def lockCheckCallback(self, req):
        while self.underDiscussion:
            print("Waiting for previous lock to resolve")
            pass
        if req.leaderID == self.navigationMsg.leaderID:
            if np.linalg.norm(robotsInfo[self.navigationMsg.leaderID].pose - robotsInfo[self.robotID].pose) < req.distance_from_leader:
                return multirobot.srv.lock_checkResponse(req.leaderID == self.navigationMsg.leaderID, True)
            else:
                self.navigationMsg.leaderID = req.myID
                return multirobot.srv.lock_checkResponse(req.leaderID == self.navigationMsg.leaderID, False)
        else:
            # Yet to be decided
            return multirobot.srv.lock_checkResponse(req.leaderID == self.navigationMsg.leaderID, False)


rospy.init_node('volta_' + str(robotID) + '_camera_node')

if __name__ == '__main__':
    try:
        camera = Camera(noOfRobots)
        com = InterCommunication(robotID, noOfRobots)
        robotsInfo = []
        for id in range(noOfRobots):
            robotsInfo.append(robotInfo(id))

        pub = rospy.Publisher("/volta_" + str(robotID) + "/navigation", navigation, queue_size=10)

        rospy.Subscriber("/volta_" + str(robotID) + "/camera_front/camera_front", Image, camera.frontCameraCallback)
        rospy.Subscriber("/volta_" + str(robotID) + "/camera_left/camera_left", Image, camera.leftCameraCallback)
        rospy.Subscriber("/volta_" + str(robotID) + "/camera_right/camera_right", Image, camera.rightCameraCallback)

        rospy.Service('lock_check_' + str(robotID), multirobot.srv.lock_check, com.lockCheckCallback)

        time.sleep(1)

        com.navigationMsg.leaderID = camera.selectMyLeader()
        com.navigationMsg.flag = False

        print(robotID, "following", com.navigationMsg.leaderID, "| Waiting for permission ...")
        result = com.convoy_client(robotID)
        if not result:
            print("Mission Abort!")
            exit()
        print("Permission Granted !")

        collisionPossible = False
        while not rospy.is_shutdown():
            if camera.frontCamera and camera.leftCamera and camera.rightCamera:
                for id in camera.robotsAroundMe:
                    if id != com.navigationMsg.leaderID:
                        distance = np.linalg.norm(robotsInfo[id].pose - robotsInfo[robotID].pose)
                        if distance < 1.5:
                            com.navigationMsg.flag = False
                            rospy.wait_for_service('lock_check_' + str(id))
                            lock_check = rospy.ServiceProxy('lock_check_' + str(id), multirobot.srv.lock_check)
                            lock_check_return = lock_check(robotID, com.navigationMsg.leaderID, np.linalg.norm(robotsInfo[com.navigationMsg.leaderID].pose - robotsInfo[robotID].pose))
                            print(com.navigationMsg.flag, "Asking", id, "->", lock_check_return.same_leaderID_confirmation, lock_check_return.update_your_leader)
                            if lock_check_return.same_leaderID_confirmation:
                                if lock_check_return.update_your_leader:
                                    com.navigationMsg.leaderID = id
                            collisionPossible = True
                        if not collisionPossible:
                            com.navigationMsg.flag = True
                collisionPossible = False
                camera.frontCamera = False
                camera.leftCamera = False
                camera.rightCamera = False
                camera.robotsAroundMe = set({})
            pub.publish(com.navigationMsg)
            com.navigationMsg.flag = True

    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")
