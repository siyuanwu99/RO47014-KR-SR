#!/usr/bin/env python

import rospy
import actionlib
from tiago_custom_msgs.msg import PlaceAction
from tiago_custom_msgs.msg import PlaceGoal
from geometry_msgs.msg import PointStamped

class PlaceClient(object):
    def __init__(self, side="right"):
        # Launch Client and wait for server
        self.client = actionlib.SimpleActionClient('/place_server_' + side, PlaceAction)
        rospy.loginfo('Waiting for server...')
        self.client.wait_for_server()

    def run(self, position=PointStamped(), aruco_id=0):
        goal = PlaceGoal()
        goal.aruco_id = aruco_id
        goal.position = position

        rospy.loginfo('Sending placing goal...')
        self.client.send_goal(goal)
        self.client.wait_for_result()

        rospy.loginfo('Placing finished')
