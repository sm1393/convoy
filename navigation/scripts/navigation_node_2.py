#!/usr/bin/env python3

import sys

import numpy as np
import time

import rospy
import actionlib
from geometry_msgs.msg import PoseWithCovarianceStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import Path
from geometry_msgs.msg import Twist

myGoal = PoseWithCovarianceStamped()
localPlanCopy = Path()
navigationControl = False
isRobotDeviated = False

robotID = int(rospy.myargv(argv=sys.argv)[1])
myLeaderID = int(rospy.myargv(argv=sys.argv)[2])

rospy.init_node('volta_' + str(robotID) + '_navigation_node')
client = actionlib.SimpleActionClient('/volta_' + str(robotID) + '/move_base',MoveBaseAction)
client.wait_for_server()
goal = MoveBaseGoal()
goal.target_pose.header.frame_id = 'map'

leaderPrevPose = np.array([np.inf, np.inf])

leaderPose = np.array([0.0, 0.0])
prevLeaderPose = np.array([0.0, 0.0])
myGoal = PoseWithCovarianceStamped()
def leaderPoseeCallback(msg):
    global leaderPose, myGoal
    myGoal = msg
    leaderPose[0] = msg.pose.pose.position.x
    leaderPose[1] = msg.pose.pose.position.y

myPose = np.array([0.0, 0.0])
def myPoseCallback(msg):
    global myPose, covariance
    myPose[0] = msg.pose.pose.position.x
    myPose[1] = msg.pose.pose.position.y    

def localPlanCallback(msg):
    global localPlanCopy
    localPlanCopy = msg

def navigationControl():
    global localPlanCopy, myPose
    pointOnLine = np.array([0.0, 0.0])
    minDeviationFromPath = np.inf
    for pose in localPlanCopy.poses:
        pointOnLine[0] = pose.pose.position.x
        pointOnLine[1] = pose.pose.position.y
        deviation = np.linalg.norm(pointOnLine - myPose)
        if deviation < minDeviationFromPath:
            minDeviationFromPath = deviation
        if minDeviationFromPath > 0.5:
            return True
    return False

if __name__ == '__main__':
    try:
        rospy.Subscriber("/volta_" + str(robotID) + "/amcl_pose", PoseWithCovarianceStamped, myPoseCallback)
        rospy.Subscriber("/volta_" + str(myLeaderID) + "/amcl_pose", PoseWithCovarianceStamped, leaderPoseeCallback)
        rospy.Subscriber("/volta_" + str(robotID) + "/move_base/TebLocalPlannerROS/local_plan", Path, localPlanCallback)
        velocity_publisher = rospy.Publisher("/volta_" + str(robotID) + "/cmd_vel", Twist, queue_size=10)
        vel_msg = Twist()
        vel_msg.linear.x = 0
        vel_msg.linear.x = 0
        vel_msg.linear.y = 0
        vel_msg.linear.z = 0
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        vel_msg.angular.z = 0

        robotState = np.inf

        time.sleep(1)
        print(robotID, "following", myLeaderID)

        leaderPrevPose = np.copy(leaderPose)
        while not rospy.is_shutdown():
            goal.target_pose.header.stamp = rospy.Time.now()
            goal.target_pose.pose.position.x = leaderPose[0]
            goal.target_pose.pose.position.y = leaderPose[1]
            goal.target_pose.pose.position.z = myGoal.pose.pose.position.z
            goal.target_pose.pose.orientation.x = myGoal.pose.pose.orientation.x
            goal.target_pose.pose.orientation.y = myGoal.pose.pose.orientation.y
            goal.target_pose.pose.orientation.z = myGoal.pose.pose.orientation.z
            goal.target_pose.pose.orientation.w = myGoal.pose.pose.orientation.w
            if np.linalg.norm(leaderPose - myPose) < 4:
                if robotState != 0:
                    robotState = 0
                client.stop_tracking_goal()
                client.cancel_all_goals()
                continue
            if np.linalg.norm(leaderPose - leaderPrevPose) > 1:
                if robotState != 1:
                    robotState = 1
                client.send_goal(goal)
                leaderPrevPose = np.copy(leaderPose)
            if client.get_state() == 1 and navigationControl():
                if robotState != 2:
                    robotState = 2
                    velocity_publisher.publish(vel_msg)
                continue
            elif client.get_state() == 4:
                if robotState != 3:
                    robotState = 3
                client.send_goal(goal)
                continue
            if robotState != 4:
                robotState = 4

    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")

