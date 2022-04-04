#!/usr/bin/env python

import rospy
import time
import geometry_msgs.msg
from move_base import MoveBase

rospy.init_node("example_task")
move_base = MoveBase()
pose_goal = geometry_msgs.msg.Pose()

pose_goal.position.x = -1.12
pose_goal.position.y = 1.20
pose_goal.position.z = 0.0

pose_goal.orientation.x = 0.0
pose_goal.orientation.y = 0.0
pose_goal.orientation.z = 0.707
pose_goal.orientation.w = 0.707

move_base.run(pose_goal)

