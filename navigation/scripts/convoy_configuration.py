#!/usr/bin/env python3

import sys

import numpy as np
import time
import rospy
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import PoseWithCovarianceStamped
from navigation.srv import convoyConfig,convoyConfigResponse
followerGoal = PoseWithCovarianceStamped()

noOfRobots = 4
chainFormat = np.ones((noOfRobots,))*np.nan
rospy.init_node('way_point_decider')

class robotInfo:
    def __init__(self, id):
        self.id = id
        self.robotPos = np.array([0.0, 0.0])
        rospy.Subscriber("volta_" + str(id) + "/amcl_pose", PoseWithCovarianceStamped, self.robotPosCallback)

    def robotPosCallback(self, msg):
        self.robotPos[0] = msg.pose.pose.position.x
        self.robotPos[1] = msg.pose.pose.position.y
        # self.robotPos[2] = euler_from_quaternion((msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w))[2]

def assign_follower_ID(req):
    print("Follower ID received is %d. Its leader is %d"%(req.followerID, chainFormat[req.followerID]))
    return convoyConfigResponse(int(chainFormat[req.followerID]))

def getConvoyConfig(robotPosArray):
    global noOfRobots, chainFormat
    currentRobot = 0
    for i in range(noOfRobots):
        for j in range(i+1, noOfRobots):
            print("Distance between", i, "and", j, "is", np.linalg.norm(robotPosArray[i] - robotPosArray[j]))
    print("---------------------------------------------------------------------------------------------")
    for i in range(noOfRobots-1):
        minDistance = np.inf
        nearestRobotID = np.nan
        for j in range(1, noOfRobots):
            if j not in chainFormat and j != currentRobot:
                distance = np.linalg.norm(robotPosArray[currentRobot] - robotPosArray[j])
                if distance < minDistance:
                    minDistance = distance
                    nearestRobotID = j
        chainFormat[nearestRobotID] = currentRobot
        currentRobot = nearestRobotID
    return chainFormat

if __name__ == '__main__':
    try:
        robotPosCallbacks = []
        for i in range(noOfRobots):
            robotPosCallbacks.append(robotInfo(i))
        time.sleep(2)
        robotInitialPosArray = []
        for i in range(noOfRobots):
            robotInitialPosArray.append(robotPosCallbacks[i].robotPos)
        chainFormat = getConvoyConfig(robotInitialPosArray)
        print("Final Chain Formation: ", chainFormat)

        s = rospy.Service('convoy_config', convoyConfig, assign_follower_ID)
        print("Ready to assign role.")
        rospy.spin()

    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")

