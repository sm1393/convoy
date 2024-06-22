#! /usr/bin/env python3

import rospy
import actionlib
import multirobot.msg

robotsReady = 0
noOfRobots = 7

class ConvoyAction(object):
    _feedback = multirobot.msg.InitiateConvoyFeedback()
    _result = multirobot.msg.InitiateConvoyResult()

    def __init__(self, name):
        self._action_name = "convoy_config_" + str(name)
        self._as = actionlib.SimpleActionServer(self._action_name, multirobot.msg.InitiateConvoyAction, execute_cb = self.execute_cb, auto_start=False)
        self._as.start()

    def execute_cb(self, goal):
        global robotsReady, noOfRobots
        success = False
        robotsReady += 1
        self._feedback.wait = 1
        rospy.loginfo('%s: Executing, waiting for all %r ' % (self._action_name, self._feedback.wait))
        while not success:
            if robotsReady == noOfRobots-1:
                self._feedback.wait = 0
                success = True
            else:
                self._as.publish_feedback(self._feedback)
        if success:
            self._result.command = 1
            rospy.loginfo('%s: Succeeded' % self._action_name)
            self._as.set_succeeded(self._result)

if __name__ == '__main__':
    rospy.init_node('convoy')
    for i in range(noOfRobots):
        ConvoyAction(i)
    rospy.spin()
