#!/usr/bin/env python3

import sys

import numpy as np
import rospy
import actionlib
from geometry_msgs.msg import PoseWithCovarianceStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

followerGoal = PoseWithCovarianceStamped()
leaderPos = np.array([0.0, 0.0])
prevLeaderPos = np.array([0.0, 0.0])
followerPos = np.array([0.0, 0.0])

myargv = rospy.myargv(argv=sys.argv)
LeaderTopic = "/" + myargv[1] + "/amcl_pose"
followerTopic = "/" + myargv[2] + "/amcl_pose"

rospy.init_node('movebase_action_client')
client = actionlib.SimpleActionClient('/' + myargv[2] + '/move_base',MoveBaseAction)
client.wait_for_server()
goal = MoveBaseGoal()
goal.target_pose.header.frame_id = 'map'

def poseLeader(msg):
    global leaderPos, followerGoal
    followerGoal = msg
    leaderPos[0] = msg.pose.pose.position.x
    leaderPos[1] = msg.pose.pose.position.y

def poseFollower(msg):
    global followerPos
    followerPos[0] = msg.pose.pose.position.x
    followerPos[1] = msg.pose.pose.position.y

if __name__ == '__main__':
    try:
        rospy.Subscriber(LeaderTopic, PoseWithCovarianceStamped, poseLeader)
        rospy.Subscriber(followerTopic, PoseWithCovarianceStamped, poseFollower)
        while not rospy.is_shutdown():
            if np.linalg.norm(leaderPos-followerPos) > 1:
                if np.linalg.norm(leaderPos-prevLeaderPos) > 1:
                    goal.target_pose.header.stamp = rospy.Time.now()
                    goal.target_pose.pose.position.x = followerGoal.pose.pose.position.x
                    goal.target_pose.pose.position.y = followerGoal.pose.pose.position.y
                    goal.target_pose.pose.position.z = followerGoal.pose.pose.position.z
                    goal.target_pose.pose.orientation.x = followerGoal.pose.pose.orientation.x
                    goal.target_pose.pose.orientation.y = followerGoal.pose.pose.orientation.y
                    goal.target_pose.pose.orientation.z = followerGoal.pose.pose.orientation.z
                    goal.target_pose.pose.orientation.w = followerGoal.pose.pose.orientation.w
                    client.send_goal(goal)
                    prevLeaderPos = np.copy(leaderPos)
            else:
                client.stop_tracking_goal()
                client.cancel_all_goals()
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")
