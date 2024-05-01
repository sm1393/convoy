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

class Communication:
    def __init__(self, robotID):
        self.robotID = robotID
        self.myFollowers = set()
        self.navigationMsg = navigation()
        self.underDiscussion = False

    def convoy_client(self, robotID):
        client = actionlib.SimpleActionClient('convoy_config_'+str(robotID), multirobot.msg.InitiateConvoyAction)
        client.wait_for_server()
        goal = multirobot.msg.InitiateConvoyGoal(myID=robotID)
        client.send_goal(goal)
        client.wait_for_result()
        return client.get_result()

    def lockCheckCallback(self, req):
        self.underDiscussion = True
        print("Robot", req.myID, " with leader ", req.leaderID, " asking for lock check")
        if req.leaderID == self.navigationMsg.leaderID:
            self.navigationMsg.flag = False
            pub.publish(self.navigationMsg)
            if np.linalg.norm(robotsInfo[self.navigationMsg.leaderID].pose - robotsInfo[robotID].pose) < req.distance_from_leader:
                self.underDiscussion = False
                return multirobot.srv.lock_checkResponse(req.leaderID == self.navigationMsg.leaderID, True)
            else:
                self.underDiscussion = False
                self.navigationMsg.leaderID = req.myID
                return multirobot.srv.lock_checkResponse(req.leaderID == self.navigationMsg.leaderID, False)
        else:
            # Yet to be decided
            self.underDiscussion = False
            return multirobot.srv.lock_checkResponse(req.leaderID == self.navigationMsg.leaderID, False)
        
    def updateFollowerListCallback(self, req):
        if req.action == 1:
            self.myFollowers.add(req.followerID)
            return multirobot.srv.leader_confirmationResponse(True)
        else:
            self.myFollowers.remove(req.followerID)
            return multirobot.srv.leader_confirmationResponse(True)

if __name__ == '__main__':
    try:
        robotID = int(rospy.myargv(argv=sys.argv)[1])
        rospy.init_node('volta_' + str(robotID) + '_camera_node')

        noOfRobots = 4
        camera = Camera(noOfRobots)
        com = Communication(robotID)

        pub = rospy.Publisher("/volta_" + str(robotID) + "/navigation", navigation, queue_size=10)

        rospy.Subscriber("/volta_" + str(robotID) + "/camera_front/camera_front", Image, camera.frontCameraCallback)
        rospy.Subscriber("/volta_" + str(robotID) + "/camera_left/camera_left", Image, camera.leftCameraCallback)
        rospy.Subscriber("/volta_" + str(robotID) + "/camera_right/camera_right", Image, camera.rightCameraCallback)

        rospy.Service('lock_check_' + str(robotID), multirobot.srv.lock_check, com.lockCheckCallback)
        rospy.Service('leader_confirmation_' + str(robotID), multirobot.srv.leader_confirmation, com.updateFollowerListCallback)

        robotsInfo = []
        for id in range(noOfRobots):
            robotsInfo.append(robotInfo(id))

        time.sleep(1)
        com.navigationMsg.leaderID = camera.selectMyLeader(robotsInfo)
        if com.navigationMsg.leaderID != 0:
            rospy.wait_for_service('leader_confirmation_' + str(com.navigationMsg.leaderID))
            leader_confirmation = rospy.ServiceProxy('leader_confirmation_' + str(com.navigationMsg.leaderID), multirobot.srv.leader_confirmation)
            leader_confirmation_return = leader_confirmation(True, robotID)
        com.navigationMsg.flag = False

        print("Request Accepted: ", robotID, "following", com.navigationMsg.leaderID)

        result = com.convoy_client(robotID)
        if not result:
            print("Mission Abort!")
            exit()

        while not rospy.is_shutdown():
            if camera.frontCamera and camera.leftCamera and camera.rightCamera:
                robotsAroundMeCopy = camera.robotsAroundMe.copy()
                for id in robotsAroundMeCopy:
                    if id != com.navigationMsg.leaderID and id not in com.myFollowers:
                        distance = np.linalg.norm(robotsInfo[id].pose - robotsInfo[robotID].pose)
                        if distance < 2.5:
                            com.navigationMsg.flag = False
                            pub.publish(com.navigationMsg)
                            if not com.underDiscussion:
                                com.underDiscussion = True
                                rospy.wait_for_service('lock_check_' + str(id))
                                print("Asking", id, " for discussion about leader ", com.navigationMsg.leaderID)
                                lock_check = rospy.ServiceProxy('lock_check_' + str(id), multirobot.srv.lock_check)
                                lock_check_return = lock_check(robotID, com.navigationMsg.leaderID, np.linalg.norm(robotsInfo[com.navigationMsg.leaderID].pose - robotsInfo[robotID].pose))

                                if lock_check_return.same_leaderID_confirmation:
                                    if lock_check_return.update_your_leader:

                                        rospy.wait_for_service('leader_confirmation_' + str(com.navigationMsg.leaderID))
                                        leader_confirmation = rospy.ServiceProxy('leader_confirmation_' + str(com.navigationMsg.leaderID), multirobot.srv.leader_confirmation)
                                        leader_confirmation_return = leader_confirmation(False, robotID)

                                        com.navigationMsg.leaderID = id
                                        rospy.wait_for_service('leader_confirmation_' + str(com.navigationMsg.leaderID))
                                        leader_confirmation = rospy.ServiceProxy('leader_confirmation_' + str(com.navigationMsg.leaderID), multirobot.srv.leader_confirmation)
                                        leader_confirmation_return = leader_confirmation(True, robotID)
                                com.underDiscussion = False

                collisionPossible = False
                camera.frontCamera = False
                camera.leftCamera = False
                camera.rightCamera = False
                robotsAroundMeCopy = set({})
            if com.underDiscussion:
                com.navigationMsg.flag = False
            else:
                com.navigationMsg.flag = True
            pub.publish(com.navigationMsg)

    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")
