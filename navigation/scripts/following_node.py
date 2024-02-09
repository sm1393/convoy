#!/usr/bin/env python3

import sys

import numpy as np
from cv_bridge import CvBridge
import cv2
import imutils

import rospy
import actionlib
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import Image, CompressedImage
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

ARUCO_DICT = {
	"DICT_4X4_50": cv2.aruco.DICT_4X4_50,
	"DICT_4X4_100": cv2.aruco.DICT_4X4_100,
	"DICT_4X4_250": cv2.aruco.DICT_4X4_250,
	"DICT_4X4_1000": cv2.aruco.DICT_4X4_1000,
	"DICT_5X5_50": cv2.aruco.DICT_5X5_50,
	"DICT_5X5_100": cv2.aruco.DICT_5X5_100,
	"DICT_5X5_250": cv2.aruco.DICT_5X5_250,
	"DICT_5X5_1000": cv2.aruco.DICT_5X5_1000,
	"DICT_6X6_50": cv2.aruco.DICT_6X6_50,
	"DICT_6X6_100": cv2.aruco.DICT_6X6_100,
	"DICT_6X6_250": cv2.aruco.DICT_6X6_250,
	"DICT_6X6_1000": cv2.aruco.DICT_6X6_1000,
	"DICT_7X7_50": cv2.aruco.DICT_7X7_50,
	"DICT_7X7_100": cv2.aruco.DICT_7X7_100,
	"DICT_7X7_250": cv2.aruco.DICT_7X7_250,
	"DICT_7X7_1000": cv2.aruco.DICT_7X7_1000,
	"DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL,
	"DICT_APRILTAG_16h5": cv2.aruco.DICT_APRILTAG_16h5,
	"DICT_APRILTAG_25h9": cv2.aruco.DICT_APRILTAG_25h9,
	"DICT_APRILTAG_36h10": cv2.aruco.DICT_APRILTAG_36h10,
	"DICT_APRILTAG_36h11": cv2.aruco.DICT_APRILTAG_36h11
}

aruco_type = "DICT_5X5_250"
arucoDict = cv2.aruco.Dictionary_get(ARUCO_DICT[aruco_type])
arucoParams = cv2.aruco.DetectorParameters_create()

followerGoal = PoseWithCovarianceStamped()
image = Image()
compressedImage = CompressedImage()

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

def aruco_display(corners, ids, rejected, image):
	if len(corners) > 0:
		ids = ids.flatten()
		for (markerCorner, markerID) in zip(corners, ids):
			corners = markerCorner.reshape((4, 2))
			(topLeft, topRight, bottomRight, bottomLeft) = corners
			topRight = (int(topRight[0]), int(topRight[1]))
			bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
			bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
			topLeft = (int(topLeft[0]), int(topLeft[1]))
			cv2.line(image, topLeft, topRight, (0, 255, 0), 2)
			cv2.line(image, topRight, bottomRight, (0, 255, 0), 2)
			cv2.line(image, bottomRight, bottomLeft, (0, 255, 0), 2)
			cv2.line(image, bottomLeft, topLeft, (0, 255, 0), 2)
			cX = int((topLeft[0] + bottomRight[0]) / 2.0)
			cY = int((topLeft[1] + bottomRight[1]) / 2.0)
			cv2.circle(image, (cX, cY), 4, (0, 0, 255), -1)
			cv2.putText(image, str(markerID),(topLeft[0], topLeft[1] - 10), cv2.FONT_HERSHEY_SIMPLEX,
				0.5, (0, 255, 0), 2)
			print("[Inference] ArUco marker ID: {}".format(markerID))
	return image

def poseLeader(msg):
    global leaderPos, followerGoal
    followerGoal = msg
    leaderPos[0] = msg.pose.pose.position.x
    leaderPos[1] = msg.pose.pose.position.y

def poseFollower(msg):
    global followerPos
    followerPos[0] = msg.pose.pose.position.x
    followerPos[1] = msg.pose.pose.position.y

def cameraFeed(msg):
    bridge = CvBridge()
    image = bridge.imgmsg_to_cv2(msg, "bgr8")
    image = imutils.resize(image, width=1000)
    corners, ids, rejected = cv2.aruco.detectMarkers(image, arucoDict, parameters=arucoParams)
    detected_markers = aruco_display(corners, ids, rejected, image)
    cv2.imshow('detected_markers', detected_markers)
    cv2.waitKey(1)

if __name__ == '__main__':
    try:
        rospy.Subscriber(LeaderTopic, PoseWithCovarianceStamped, poseLeader)
        rospy.Subscriber(followerTopic, PoseWithCovarianceStamped, poseFollower)
        rospy.Subscriber("/" + myargv[2] + "/camera/camera", Image, cameraFeed)
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
        cv2.destroyAllWindows() 
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")
