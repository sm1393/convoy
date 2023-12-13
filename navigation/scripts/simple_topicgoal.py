#!/usr/bin/env python3

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseStamped

rospy.init_node('movebase_action_client')
goalPub = rospy.Publisher("/volta_1/move_base_simple/goal", PoseStamped, queue_size=1)
client = actionlib.SimpleActionClient('move_base_1',MoveBaseAction)
goal = PoseStamped()
goal.header.frame_id = 'map'

if __name__ == '__main__':
    try:
        goal.header.stamp = rospy.Time.now()
        goal.pose.position.x = 0.0
        goal.pose.position.y = 0.0
        goal.pose.orientation.w = 1.0
        print(goalPub.name)
        while goalPub.get_num_connections() < 1:
            pass
        goalPub.publish(goal)
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")
