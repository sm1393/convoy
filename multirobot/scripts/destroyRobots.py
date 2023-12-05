#!/usr/bin/python3
import rospy
from gazebo_msgs.srv import DeleteModel

modelList = ["volta_0", "volta_1"]

if __name__ == "__main__":
    try:
        delete_model = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
        for model in modelList:
            delete_model(model)
    except rospy.ServiceException as e:
        print("Delete Model service call failed: {0}".format(e))