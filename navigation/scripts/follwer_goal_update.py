#!/usr/bin/env python3

import numpy as np
import rospy
import actionlib
from geometry_msgs.msg import PoseWithCovarianceStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

leaderPos = np.array([0.0, 0.0])
followerPos = np.array([0.0, 0.0])

rospy.init_node('movebase_action_client')
client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
client.wait_for_server()
goal = MoveBaseGoal()
goal.target_pose.header.frame_id = 'map'


def poseLeader(msg):
    global leaderPos
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
            if np.linalg.norm(leaderPos-followerPos) > 3:
                if client.get_state() != 0 and client.get_state() != 1:
                    print(client.get_state())
                    goal.target_pose.header.stamp = rospy.Time.now()
                    goal.target_pose.pose.position.x = leaderPos[0]
                    goal.target_pose.pose.position.y = leaderPos[1]
                    goal.target_pose.pose.orientation.w = 1.0
                    client.send_goal(goal)
                    # while client.get_state() != 3:
                    #     print(client.get_state())
            else:
                client.cancel_all_goals()
            print(client.get_state(), np.linalg.norm(leaderPos-followerPos))
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")
