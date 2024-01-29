#!/usr/bin/env python3

import numpy as np
import rospy
import actionlib
from geometry_msgs.msg import PoseWithCovarianceStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

followerGoal = PoseWithCovarianceStamped()
leaderPos = np.array([0.0, 0.0])
prevLeaderPos = np.array([0.0, 0.0])
followerPos = np.array([0.0, 0.0])

rospy.init_node('movebase_action_client')
client = actionlib.SimpleActionClient('/volta_1/move_base',MoveBaseAction)
print("Searching")
client.wait_for_server()
print("found")
print()
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
        rospy.Subscriber('/volta_0/amcl_pose', PoseWithCovarianceStamped, poseLeader)
        rospy.Subscriber('/volta_1/amcl_pose', PoseWithCovarianceStamped, poseFollower)
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
