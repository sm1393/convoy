#!/usr/bin/env python3
# import roslib; roslib.load_manifest('Phoebe')
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import tf
import math
import numpy as np

linVelkP = 2
angVelkP = 2
rTrSafeDistance = 1

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

def followTarget(robotPos, target):
    pointInRobotFrame = transformWorldToRobot(robotPos[0], robotPos[1], robotPos[2], [target[0], target[1], 1])
    weight = np.linalg.norm(pointInRobotFrame[:2]) - rTrSafeDistance
    linVel = linVelkP * pointInRobotFrame[0] * weight
    angVel = angVelkP * pointInRobotFrame[1] * weight
    return linVel, angVel

def follow(linVel, angVel):
    twistMsg.linear.x = linVel
    twistMsg.linear.y = 0
    twistMsg.linear.z = 0
    twistMsg.angular.x = 0
    twistMsg.angular.y = 0
    twistMsg.angular.z = angVel
    followerTwist.publish(twistMsg)

if __name__ == "__main__":
    leaderQueue = Queue()
    followerPos = [0,0,0]
    rospy.init_node('leader_follower', anonymous=False)
    rospy.Subscriber('/volta_0/odom', Odometry, odometryLeader)
    rospy.Subscriber('/volta_1/odom', Odometry, odometryFollower)
    followerTwist = rospy.Publisher('/volta_1/cmd_vel', Twist, queue_size=1)
    twistMsg = Twist()
    while not rospy.is_shutdown():
        try:
            if not(leaderQueue.isEmpty()):
                linVel, angVel = followTarget(followerPos, leaderQueue.dequeue())
                follow(linVel, angVel)
        except Exception as E:
            # print("Exception", E)
            continue