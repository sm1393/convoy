#!/usr/bin/env python3

import sys

import numpy as np
from cv_bridge import CvBridge, CvBridgeError
import cv2
import time
import imutils

import rospy
import actionlib
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import Image
from navigation.msg import navigation
import multirobot.msg
import multirobot.srv

from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TransformStamped
from tf import transformations
import tf
from ultralytics import YOLO
bridge = CvBridge()

model = YOLO('train46.pt') #replace with downloaded weights file
# Camera parameters (update these with actual camera parameters)
camera_matrix = np.array([[970.13975699,   0.        , 661.05696322],
                                   [  0.        , 965.0683426 , 324.24867006],
                                   [  0.        ,    0.       ,   1.        ]]) #for camera 219, for other cameras refer to homography.txt
dist_coeffs = np.array([-0.44779831, 0.21493212, 0.0086979, -0.00269077, 0.00281984]) #for camera 219, for other cameras refer to homography.txt
br = tf.TransformBroadcaster()

# Function to estimate pose using PnP
def estimate_pose(keypoints_2d, keypoints_3d):
    # Convert keypoints to numpy arrays
    keypoints_2d = np.array(keypoints_2d)
    keypoints_3d = np.array(keypoints_3d)
    # Perform PnP
    _, rotation_vector, translation_vector, _ = cv2.solvePnPRansac(keypoints_3d, keypoints_2d, camera_matrix, dist_coeffs)
    # Convert rotation vector to rotation matrix
    rotation_matrix, _ = cv2.Rodrigues(rotation_vector)
    return translation_vector, rotation_matrix

# ROS callback function for receiving camera images
def camera_callback(frame):
    try:
        # Perform object detection
        results = model(frame)
        # Extract keypoints from YOLO detections
        keypoints = results[0].keypoints.xy.cpu().numpy()
        if keypoints.shape[1] == 0:
            print("none")
            return
        print(keypoints[0])
        for kp in keypoints[0]:
            x, y = int(kp[0]), int(kp[1])
            cv2.circle(frame, (x, y), 5, (0, 255, 0), -1)
        # Show the frame with keypoints
        # cv2.imshow("KeyPoints", frame)

        # # Prepare 3D keypoints
        keypoints_3d = np.array([[0.01206,0.235,-0.21776],[0.00005,0.0813,-0.26776],[0.00005, 0.171, 0.19024],[0.16505, 0.2642, 0.10624],[0.16506, 0.2655, -0.16966],[-0.16495, 0.2641, 0.10524],[-0.16495, 0.2633, -0.17016],[-0.06495, 0.17695, -0.30763],
        [-0.01994,0.17695,-0.27316],[0.06006, 0.18075, -0.28716],[0.06505, 0.13695, -0.27976],[0.00005, 0.037, -0.26736],[-0.11994, 0.19595, -0.27163],[-0.11994, 0.15995, -0.27063],[0.11805, 0.19595, -0.27063],
        [0.11805, 0.15795, -0.27163],[0.17996,0.2377,-0.16876],[0.18006, 0.1211, -0.16876],[0.18075,0.238,0.10694],[0.18026,0.1226,0.10524],[0.03766,0.2073,0.14424],[0.03756,0.207,0.22124],[-0.03694,0.2073,0.14424],
        [-0.03654,0.2073,0.22164],[0.13015,0.1208, 0.22314],[-0.12935,0.1206,0.22254],[0.17955,0.1211,0.17304],[-0.17975,0.1206,0.17334],[-0.16495,0.2671,-0.05846],[0.06505, 0.13695, -0.27976]]) #these are obtained from Unity 3D game simulator

        # # Estimate pose using PnP
        # print(keypoints, end=" | ")
        translation, rotation = estimate_pose([keypoints[0]], keypoints_3d)
        # print(translation, " | ", rotation)
        # print("#############################################################")
        # Publish pose
        pose_msg = PoseStamped()
        pose_msg.header.stamp = rospy.Time.now()
        pose_msg.pose.position.x = translation[0]
        pose_msg.pose.position.y = translation[1]
        pose_msg.pose.position.z = translation[2]
        rotation = np.vstack((rotation, np.array([0,0,0])))
        rotation = np.hstack((rotation, np.array([0,0,0,1]).reshape(-1,1)))
        # print(rotation)   
        quaternion = transformations.quaternion_from_matrix(rotation)
        pose_msg.pose.orientation.x = quaternion[0]
        pose_msg.pose.orientation.y = quaternion[1]
        pose_msg.pose.orientation.z = quaternion[2]
        pose_msg.pose.orientation.w = quaternion[3]
        pose_pub.publish(pose_msg)

        static_transformStamped = TransformStamped()
        static_transformStamped.header.stamp = rospy.Time.now()
        static_transformStamped.header.frame_id = "camera"
        static_transformStamped.child_frame_id = "volta"
        static_transformStamped.transform.translation.x = translation[0]
        static_transformStamped.transform.translation.y = translation[1]
        static_transformStamped.transform.translation.z = translation[2]
        static_transformStamped.transform.rotation.x = quaternion[0]
        static_transformStamped.transform.rotation.y = quaternion[1]
        static_transformStamped.transform.rotation.z = quaternion[2]
        static_transformStamped.transform.rotation.w = quaternion[3]
        br.sendTransform(translation, quaternion, rospy.Time.now(), "volta_0_detected", "volta_1/camera_front")
    except CvBridgeError as e:
        print(e)

class robotInfo:
    def __init__(self, id):
        self.id = id
        self.pose = np.array([0.0, 0.0])
        rospy.Subscriber("/volta_" + str(self.id) + "/amcl_pose", PoseWithCovarianceStamped, self.poseCallback)

    def poseCallback(self, msg):
        self.pose[0] = msg.pose.pose.position.x
        self.pose[1] = msg.pose.pose.position.y

class Camera:
    def __init__(self, noOfRobots):
        self.robotsAroundMe = set()
        self.frontCamera = False
        self.leftCamera = False
        self.rightCamera = False
        self.noOfRobots = noOfRobots
        
        self.arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_5X5_250)
        self.arucoParams = cv2.aruco.DetectorParameters_create()
        self.bridge = CvBridge()

    def frontCameraCallback(self, msg):
        image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        camera_callback(image)
        # cv2.imshow("image", image)
        # if cv2.waitKey(1) & 0xFF == ord('q'):
        #     cv2.destroyAllWindows()
        #     exit()

    def leftCameraCallback(self, msg):
        image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

    def rightCameraCallback(self, msg):
        image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

    def selectMyLeader(self, robotsInfo):
        while not self.frontCamera and self.leftCamera and self.rightCamera:
            pass
        self.frontCamera = False
        self.leftCamera = False
        self.rightCamera = False

        minDistance = np.inf
        myLeaderID = np.nan
        for robot in self.robotsAroundMe:
            if robot == 0:
                return 0
            distance = np.linalg.norm(robotsInfo[robot].pose - robotsInfo[robotID].pose)
            if distance < minDistance:
                minDistance = distance
                myLeaderID = robot
        self.robotsAroundMe = set({})
        return myLeaderID

if __name__ == '__main__':
    try:
        robotID = int(rospy.myargv(argv=sys.argv)[1])
        rospy.init_node('volta_' + str(robotID) + '_camera_node')

        noOfRobots = 2
        camera = Camera(noOfRobots)

        pub = rospy.Publisher("/volta_" + str(robotID) + "/navigation", navigation, queue_size=10)
        pose_pub = rospy.Publisher('/pose_estimation', PoseStamped, queue_size=10)

        rospy.Subscriber("/volta_" + str(robotID) + "/camera_front/camera_front", Image, camera.frontCameraCallback)
        rospy.Subscriber("/volta_" + str(robotID) + "/camera_left/camera_left", Image, camera.leftCameraCallback)
        rospy.Subscriber("/volta_" + str(robotID) + "/camera_right/camera_right", Image, camera.rightCameraCallback)

        robotsInfo = []
        for id in range(noOfRobots):
            robotsInfo.append(robotInfo(id))

        while not rospy.is_shutdown():
            pass
        #     if cv2.waitKey(1) & 0xFF == ord('q'):
        #         break
        # cv2.destroyAllWindows()

    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")
