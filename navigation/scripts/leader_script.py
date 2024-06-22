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

robotID = int(rospy.myargv(argv=sys.argv)[1])

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

if __name__ == '__main__':
    try:
        rospy.Subscriber("/volta_" + str(robotID) + "/amcl_pose", PoseWithCovarianceStamped, myPoseCallback)

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
        
        while not rospy.is_shutdown():
            print(np.linalg.norm(np.array(goalArray[counter]) - myPose), counter, myPose, goalArray[counter-1])
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
                client.send_goal(goal)
                print("Sent")
                counter += 1

    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")

