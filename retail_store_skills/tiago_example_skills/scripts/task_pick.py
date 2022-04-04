#!/usr/bin/env python

import rospy

from pick_client import PickClient

rospy.init_node("place_task")

pick_client = PickClient("right")
id = 16
pick_client.run(aruco_id=id, object_type="hagelslag")
print("Picked object with id: " + str(id))
print("Waiting for object to be picked")
    
# pick_client = PickClient("left")
# id = 1
# pick_client.run(aruco_id=id, object_type="hagelslag")
# print("Picked object with id: " + str(id))
# print("Waiting for object to be picked")
    
