#!/usr/bin/env python3

import numpy as np
import rospy
import actionlib
from nav_msgs.msg import Odometry
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseStamped

leaderPos = np.array([0.0, 0.0])
followerPos = np.array([0.0, 0.0])

def odometryLeader(msg):
    global leaderPos
    leaderPos[0] = msg.pose.pose.position.x
    leaderPos[1] = msg.pose.pose.position.y

def odometryFollower(msg):
    global followerPos
    followerPos[0] = msg.pose.pose.position.x
    followerPos[1] = msg.pose.pose.position.y

rospy.init_node('movebase_action_client')
rospy.Subscriber('/volta_0/odom', Odometry, odometryLeader)
rospy.Subscriber('/volta_1/odom', Odometry, odometryFollower)
goalPub = rospy.Publisher("/volta_1/move_base_simple/goal", PoseStamped, queue_size=1)
client = actionlib.SimpleActionClient('move_base_1',MoveBaseAction)
goal = PoseStamped()
goal.header.frame_id = 'map'

if __name__ == '__main__':
    try:
        while not rospy.is_shutdown():
            if np.linalg.norm(leaderPos-followerPos) > 2:
                print("goal bhejo")
            else:
                print("goal mat bhejo")
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")
