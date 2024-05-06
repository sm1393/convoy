#!/usr/bin/env python3

import sys
import numpy  as np
import matplotlib.pyplot as plt
from math import sin, cos, pi

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Path

scanCollected = False
localGoal = [1.0, 1.0]
robotPose = [1.0, 1.0]

def poseCallback(msg):
    robotPose[0] = msg.pose.pose.position.x
    robotPose[1] = msg.pose.pose.position.y

def laserScanCallback(msg):
    global scanCollected
    if not scanCollected:
        scanCollected = True
        scan = msg.ranges
        X = []
        Y = []
        for i in range(len(msg.ranges)):
            if i > len(scan)/4 and i < 3*len(scan)/4:
                if scan[i] != np.inf:
                    X.append(scan[i] * cos(2*pi * i / len(scan)))
                    Y.append(scan[i] * sin(2*pi * i / len(scan)))   
                else:
                    X.append(0)
                    Y.append(0)   
        plt.scatter(X, Y)
        plt.savefig("plt")

def localGoalCallback(msg):
    localGoal.target_pose.pose.position.x = msg.poses.pose.position.x
    localGoal.target_pose.pose.position.y = msg.poses.pose.position.y
        
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
