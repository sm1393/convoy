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

# robotID = int(rospy.myargv(argv=sys.argv)[1])

robotID = 0

rospy.init_node('volta_' + str(robotID) + '_navigation_node')
client = actionlib.SimpleActionClient('/volta_' + str(robotID) + '/move_base',MoveBaseAction)
client.wait_for_server()
goal = MoveBaseGoal()
goal.target_pose.header.frame_id = 'map'

myPose = np.array([0.0, 0.0])
def myPoseCallback(msg):
    global myPose
    myPose[0] = msg.pose.pose.position.x
    myPose[1] = msg.pose.pose.position.y    

followersList = [1]
followerPose = [np.array([0.0, 0.0]), np.array([0.0, 0.0]), np.array([0.0, 0.0]), np.array([0.0, 0.0]), np.array([0.0, 0.0]), np.array([0.0, 0.0]), np.array([0.0, 0.0])]
def followerPose_1_Callback(msg):
    global followerPose
    followerPose[1][0] = msg.pose.pose.position.x
    followerPose[1][1] = msg.pose.pose.position.y
def followerPose_2_Callback(msg):
    global followerPose
    followerPose[2][0] = msg.pose.pose.position.x
    followerPose[2][1] = msg.pose.pose.position.y
def followerPose_3_Callback(msg):
    global followerPose
    followerPose[3][0] = msg.pose.pose.position.x
    followerPose[3][1] = msg.pose.pose.position.y
def followerPose_4_Callback(msg):
    global followerPose
    followerPose[4][0] = msg.pose.pose.position.x
    followerPose[4][1] = msg.pose.pose.position.y
def followerPose_5_Callback(msg):
    global followerPose
    followerPose[5][0] = msg.pose.pose.position.x
    followerPose[5][1] = msg.pose.pose.position.y
def followerPose_6_Callback(msg):
    global followerPose
    followerPose[6][0] = msg.pose.pose.position.x
    followerPose[6][1] = msg.pose.pose.position.y

if __name__ == '__main__':
    try:
        rospy.Subscriber("/volta_" + str(robotID) + "/amcl_pose", PoseWithCovarianceStamped, myPoseCallback)

        rospy.Subscriber("/volta_1/amcl_pose", PoseWithCovarianceStamped, followerPose_1_Callback)
        # rospy.Subscriber("/volta_2/amcl_pose", PoseWithCovarianceStamped, followerPose_2_Callback)
        # rospy.Subscriber("/volta_3/amcl_pose", PoseWithCovarianceStamped, followerPose_3_Callback)
        # rospy.Subscriber("/volta_4/amcl_pose", PoseWithCovarianceStamped, followerPose_4_Callback)
        # rospy.Subscriber("/volta_5/amcl_pose", PoseWithCovarianceStamped, followerPose_5_Callback)
        # rospy.Subscriber("/volta_6/amcl_pose", PoseWithCovarianceStamped, followerPose_6_Callback)

        time.sleep(1)

        goalArray = [[39.5, 0.0], [45.0, -4.0], [15.0, -5.0], [0.0, 0.0]]
        counter = 0

        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = goalArray[counter][0]
        goal.target_pose.pose.position.y = goalArray[counter][1]
        goal.target_pose.pose.position.z = 0
        goal.target_pose.pose.orientation.x = 0
        goal.target_pose.pose.orientation.y = 0
        goal.target_pose.pose.orientation.z = 0
        goal.target_pose.pose.orientation.w = 1
        client.send_goal(goal)
        counter += 1
        followerLeftBack = False
        while not rospy.is_shutdown():
            # print(np.linalg.norm(np.array(goalArray[counter]) - myPose), counter, myPose, goalArray[counter-1])
            if np.linalg.norm(np.array(goalArray[counter-1]) - myPose) < 2:
                if counter > 4:
                    client.stop_tracking_goal()
                    client.cancel_all_goals()
                    exit()
                goal.target_pose.header.stamp = rospy.Time.now()
                goal.target_pose.pose.position.x = goalArray[counter][0]
                goal.target_pose.pose.position.y = goalArray[counter][1]
                goal.target_pose.pose.position.z = 0
                goal.target_pose.pose.orientation.x = 0
                goal.target_pose.pose.orientation.y = 0
                goal.target_pose.pose.orientation.z = 0
                goal.target_pose.pose.orientation.w = 1
                # client.send_goal(goal)
                print("Sent")
                counter += 1
            for i in followersList:
                print(np.linalg.norm(followerPose[i] - myPose))
                if np.linalg.norm(followerPose[i] - myPose) > 8:
                    followerLeftBack = True
                    client.stop_tracking_goal()
                    client.cancel_all_goals()
                    break
                else:
                    followerLeftBack = False
                    client.send_goal(goal)


    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")

