#!/usr/bin/env python3

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseActionGoal, MoveBaseGoal
from geometry_msgs.msg import PoseStamped

rospy.init_node('movebase_action_client')
# goalPub = rospy.Publisher("/volta_1/move_base_simple/goal", MoveBaseActionGoal, queue_size=1)
goalPub = rospy.Publisher('/move_base/goal', MoveBaseActionGoal, queue_size = -1)

goal = MoveBaseGoal()
goal.target_pose.header.frame_id = 'map'

finalGoal = MoveBaseActionGoal()
finalGoal.header = goal.target_pose.header
finalGoal.goal_id.id = str('1')


if __name__ == '__main__':
    try:
        if not rospy.is_shutdown():
            goal.target_pose.header.stamp = rospy.Time.now()
            goal.target_pose.pose.position.x = 2.0
            goal.target_pose.pose.position.y = 2.0
            goal.target_pose.pose.orientation.w = 1.0
            finalGoal.goal_id.stamp = rospy.Time.now()
            finalGoal.goal.target_pose = goal
            goalPub.publish(finalGoal)
            print(finalGoal)
            # print(goalPub.name)
            # while goalPub.get_num_connections() < 1:
            #     pass
            # goalPub.publish(goal)
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")
