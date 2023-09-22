#!/usr/bin/env python3

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import Odometry
import math 
import numpy as np
import tf

rTrSafeDistance = 2
from nav_msgs.msg import Odometry

class Queue:
    def __init__(self):
        self.items = []

    def isEmpty(self):
        return self.items == []

    def enqueue(self, data):
        self.items.append(data)

    def dequeue(self):
        return self.items.pop(0)

    def length(self):
        return len(self.items)

def odometryLeader(msg):
    yaw = tf.transformations.euler_from_quaternion((msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w))[2]
    leaderQueue.enqueue([msg.pose.pose.position.x, msg.pose.pose.position.y, yaw])

def odometryFollower(msg):
    global followerPos
    yaw = tf.transformations.euler_from_quaternion((msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w))[2]
    followerPos = [msg.pose.pose.position.x, msg.pose.pose.position.y, yaw]

def transformWorldToRobot(x, y, theta, pointInWorld):
    T = np.array([[math.cos(theta),     math.sin(theta),    -x*math.cos(theta) - y*math.sin(theta)]
                ,[-math.sin(theta),     math.cos(theta),     x*math.sin(theta) - y*math.cos(theta)]
                ,[  0,   0,   1]])
    pointInRobotFrame = T @ np.array([pointInWorld[0], pointInWorld[1], 1])
    return pointInRobotFrame

def movebase_client(target, cancelGoal):
    client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
    client.wait_for_server()
    goal = MoveBaseGoal()
    if not cancelGoal:
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = target[0]
        goal.target_pose.pose.position.y = target[1]
        goal.target_pose.pose.orientation.w = 1.0
        client.send_goal(goal)
    else:
        client.cancel_all_goals()
    wait = client.wait_for_result()
    if not wait:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
    else:
        return client.get_result()

if __name__ == '__main__':
    leaderQueue = Queue()
    followerPos = [0, 0, 0]
    oldTarget = [0, 0, 0]
    rospy.init_node('leader_follower', anonymous=False)
    rospy.Subscriber('/volta_0/odom', Odometry, odometryLeader)
    rospy.Subscriber('/volta_1/odom', Odometry, odometryFollower)
    while not rospy.is_shutdown():
        try:
            if not(leaderQueue.isEmpty()):
                target = leaderQueue.dequeue()
                pointInRobotFrame = transformWorldToRobot(followerPos[0], followerPos[1], followerPos[2], [target[0], target[1], 1])
                print(np.linalg.norm(pointInRobotFrame[:2]), target)
                input()
                if np.linalg.norm(pointInRobotFrame[:2]) > rTrSafeDistance:
                    if oldTarget != target:
                        result = movebase_client(target, cancelGoal=False)
                        oldTarget = target
                    else:
                        result = movebase_client(target, cancelGoal=True)
        except rospy.ROSInterruptException:
            rospy.loginfo("Navigation test finished.")
            continue