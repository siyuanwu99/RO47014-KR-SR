#!/usr/bin/env python

import rospy
import geometry_msgs.msg

from place_client import PlaceClient

rospy.init_node("place_task")

# place_client = PlaceClient("right")
place_client = PlaceClient("left")
point_goal = geometry_msgs.msg.PointStamped()

# point_goal.header.frame_id = "base_footprint"
# point_goal.point.x = 0.8
# point_goal.point.y = -0.3
# point_goal.point.z = 0.93

point_goal.header.frame_id = "map"
point_goal.point.x = 1.93
point_goal.point.y = 2.5
point_goal.point.z = 1.3

place_client.run(position=point_goal)
# place_client.run(aruco_id=1)
