#!/usr/bin/env python3

import sys
import numpy  as np
import matplotlib.pyplot as plt
from math import sin, cos, pi

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Path

scanInfoCollected = False
localGoal = [1.0, 1.0]
robotPose = [1.0, 1.0]
noOfPoints = 0
angleMin = 0
angleMax = 0

def poseCallback(msg):
    global robotPose
    robotPose[0] = msg.pose.pose.position.x
    robotPose[1] = msg.pose.pose.position.y

def laserScanCallback(msg):
    global scanInfoCollected, noOfPoints, angleMin, angleMax
    if not scanInfoCollected:
        scanInfoCollected = True
        angleMin = msg.angle_min
        angleMax = msg.angle_max
        noOfPoints = len(msg.ranges)
    X = []
    Y = []
    for i in range(noOfPoints):
        if msg.ranges[i] != np.inf:
            angle = angleMin + (angleMax - angleMin) * i / noOfPoints
            X.append(msg.ranges[i] * cos(angle))
            Y.append(msg.ranges[i] * sin(angle))

def localGoalCallback(msg):
    global localGoal, robotPose
    print("###################################################3")
    print(robotPose, msg.poses[0], msg.poses[-1])
    # localGoal.target_pose.pose.position.x = msg.poses.pose.position.x
    # localGoal.target_pose.pose.position.y = msg.poses.pose.position.y
        
if __name__ == '__main__':
    try:
        robotID = int(rospy.myargv(argv=sys.argv)[1])
        rospy.init_node('volta_' + str(robotID) + '_laserscan_check')
        rospy.Subscriber("/volta_" + str(robotID) + "/amcl_pose", PoseWithCovarianceStamped, poseCallback)
        rospy.Subscriber("/volta_" + str(robotID) + "/move_base/DWAPlannerROS/local_plan", Path, localGoalCallback)
        rospy.Subscriber("/volta_" + str(robotID) + "/scan_filtered", LaserScan, laserScanCallback)
        rospy.spin()

    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")
