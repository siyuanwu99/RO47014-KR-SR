#!/usr/bin/env python

import rospy
import actionlib
from tiago_custom_msgs.msg import PickAction
from tiago_custom_msgs.msg import PickGoal

class PickClient(object):
    def __init__(self, side="right"):
        # Launch Client and wait for server
        self.client = actionlib.SimpleActionClient('/pick_server_' + side, PickAction)
        rospy.loginfo('Waiting for server...')
        self.client.wait_for_server()

    def run(self, aruco_id, object_type="cube_top"):
        goal = PickGoal()
        goal.aruco_id = aruco_id
        goal.object_type = object_type

        rospy.loginfo('Sending picking goal...')
        self.client.send_goal(goal)
        self.client.wait_for_result()

        rospy.loginfo('Picking finished')
