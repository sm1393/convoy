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

finalDeviation = np.inf
def navigationControl():
    global localPlanCopy, myPose, finalDeviation
    pointList = [np.array([pose.pose.position.x, pose.pose.position.y]) for pose in localPlanCopy.poses]
    if len(pointList) == 0:
        return False
    minDeviationFromPath = np.inf
    minDeviationPoint = 0
    for i in range(len(pointList)):
        deviation = np.linalg.norm(pointList[i] - myPose)
        if deviation < minDeviationFromPath:
            minDeviationFromPath = deviation
            minDeviationPoint = i
    if minDeviationPoint == 0:
        minDeviationPoint_1 = 1
        return False
    elif minDeviationPoint == len(pointList)-1:
        minDeviationPoint_1 = len(pointList)-2
    elif np.linalg.norm(np.array(pointList[minDeviationPoint-1]) - np.array(myPose)) > np.linalg.norm(np.array(pointList[minDeviationPoint+1]) - np.array(myPose)):
        minDeviationPoint_1 = minDeviationPoint+1
    else:
        minDeviationPoint_1 = minDeviationPoint-1
    finalDeviation = np.linalg.norm(np.cross(pointList[minDeviationPoint_1]-pointList[minDeviationPoint], pointList[minDeviationPoint]-myPose))/np.linalg.norm(pointList[minDeviationPoint_1]-pointList[minDeviationPoint])
    if finalDeviation > 0.25:
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

        time.sleep(1)
        print(robotID, "following", myLeaderID)

        leaderHasMoved = False
        closeToLeader = False
        deviatedFromPath = False
        OnPath = False

        goalCancelled = False
        goalSent = False

        counter = 0
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

            if np.linalg.norm(leaderPose - myPose) < 2:
                closeToLeader = True
            else:
                closeToLeader = False

            if np.linalg.norm(leaderPose - leaderPrevPose) > 2:
                leaderHasMoved = True
            else:
                leaderHasMoved = False

            if navigationControl():
                if deviatedFromPath:
                    stillDeviated = True
                else:
                    stillDeviated = False
                    deviatedFromPath = True
            else:
                deviatedFromPath = False
                stillDeviated = False

            if closeToLeader or deviatedFromPath:
            # if closeToLeader:
                if not goalCancelled:
                    if not stillDeviated:
                        print(counter, closeToLeader, leaderHasMoved, deviatedFromPath, stillDeviated, "C")
                        velocity_publisher.publish(vel_msg)
                        client.stop_tracking_goal()
                        client.cancel_all_goals()
                        goalCancelled = True
                        goalSent = False
                        counter += 1

            if not closeToLeader:
                if not goalSent or leaderHasMoved:
                    print(counter, closeToLeader, leaderHasMoved, deviatedFromPath, stillDeviated, "S")
                    client.send_goal(goal)
                    leaderPrevPose = np.copy(leaderPose)
                    goalSent = True
                    goalCancelled = False
                    counter += 1

    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")
