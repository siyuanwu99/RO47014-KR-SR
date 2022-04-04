#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Pose
from visualization_msgs.msg import Marker
from tf.transformations import quaternion_from_euler
from math import pi
from grocery_store_utils.srv import *


class GroceryStoreGraspPoseServer:
  def __init__(self):
    rospy.init_node("grocery_store_grasp_pose_server")

    rospy.Service('get_grasp_pose', getGraspPose, self.get_grasp_pose_cb)

    self.pub = rospy.Publisher("/grocery_store_grasp_pose_server/pose_goal", Marker, queue_size=10)

  def get_grasp_pose_cb(self, req):
    print(req)
    resp = getGraspPoseResponse()
    resp.frame_id = "aruco_marker_{}".format(req.object_id)

    if req.object_type == "cube_top":
      grasp_pose = Pose()
      grasp_pose.position.x = 0.0
      grasp_pose.position.y = -0.22
      grasp_pose.position.z = -0.01
      orientation = quaternion_from_euler(-pi/2, 0, pi/2)
      grasp_pose.orientation.x = orientation[0]
      grasp_pose.orientation.y = orientation[1]
      grasp_pose.orientation.z = orientation[2]
      grasp_pose.orientation.w = orientation[3]

      pre_grasp_pose = grasp_pose
      post_grasp_pose = grasp_pose

    elif req.object_type == "cube_side":
      grasp_pose = Pose()
      grasp_pose.position.x = 0.00
      grasp_pose.position.y = 0.00
      grasp_pose.position.z = 0.200
      orientation = quaternion_from_euler(0, pi/2, 0)
      grasp_pose.orientation.x = orientation[0]
      grasp_pose.orientation.y = orientation[1]
      grasp_pose.orientation.z = orientation[2]
      grasp_pose.orientation.w = orientation[3]

      pre_grasp_pose = grasp_pose
      post_grasp_pose = grasp_pose

    elif req.object_type == "hagelslag":
      grasp_pose = Pose()
      grasp_pose.position.x = 0.00
      grasp_pose.position.y = 0.00
      grasp_pose.position.z = 0.180
      orientation = quaternion_from_euler(0, pi/2, 0)
      grasp_pose.orientation.x = orientation[0]
      grasp_pose.orientation.y = orientation[1]
      grasp_pose.orientation.z = orientation[2]
      grasp_pose.orientation.w = orientation[3]

      pre_grasp_pose = grasp_pose
      post_grasp_pose = grasp_pose

    elif req.object_type == "carton":
      grasp_pose = Pose()
      grasp_pose.position.x = -0.01
      grasp_pose.position.y = -0.02
      grasp_pose.position.z = 0.200
      orientation = quaternion_from_euler(0, pi/2, 0)
      grasp_pose.orientation.x = orientation[0]
      grasp_pose.orientation.y = orientation[1]
      grasp_pose.orientation.z = orientation[2]
      grasp_pose.orientation.w = orientation[3]

      pre_grasp_pose = Pose()
      pre_grasp_pose.position.x = -0.01
      pre_grasp_pose.position.y = -0.02
      pre_grasp_pose.position.z = 0.300
      orientation = quaternion_from_euler(0, pi/2, 0)
      pre_grasp_pose.orientation.x = orientation[0]
      pre_grasp_pose.orientation.y = orientation[1]
      pre_grasp_pose.orientation.z = orientation[2]
      pre_grasp_pose.orientation.w = orientation[3]

      post_grasp_pose = Pose()
      post_grasp_pose.position.x = 0
      post_grasp_pose.position.y = 0.00
      post_grasp_pose.position.z = 0.350
      orientation = quaternion_from_euler(0, pi/2, 0)
      post_grasp_pose.orientation.x = orientation[0]
      post_grasp_pose.orientation.y = orientation[1]
      post_grasp_pose.orientation.z = orientation[2]
      post_grasp_pose.orientation.w = orientation[3]

    elif req.object_type == "can":
      grasp_pose = Pose()
      grasp_pose.position.x = 0
      grasp_pose.position.y = -0.00
      grasp_pose.position.z = 0.170
      orientation = quaternion_from_euler(0, pi/2, 0)
      grasp_pose.orientation.x = orientation[0]
      grasp_pose.orientation.y = orientation[1]
      grasp_pose.orientation.z = orientation[2]
      grasp_pose.orientation.w = orientation[3]

      pre_grasp_pose = Pose()
      pre_grasp_pose.position.x = 0
      pre_grasp_pose.position.y = -0.00
      pre_grasp_pose.position.z = 0.270
      orientation = quaternion_from_euler(0, pi/2, 0)
      pre_grasp_pose.orientation.x = orientation[0]
      pre_grasp_pose.orientation.y = orientation[1]
      pre_grasp_pose.orientation.z = orientation[2]
      pre_grasp_pose.orientation.w = orientation[3]

      post_grasp_pose = Pose()
      post_grasp_pose.position.x = 0
      post_grasp_pose.position.y = 0.05
      post_grasp_pose.position.z = 0.270
      orientation = quaternion_from_euler(0, pi/2, 0)
      post_grasp_pose.orientation.x = orientation[0]
      post_grasp_pose.orientation.y = orientation[1]
      post_grasp_pose.orientation.z = orientation[2]
      post_grasp_pose.orientation.w = orientation[3]

    elif req.object_type == "thee":
      grasp_pose = Pose()
      grasp_pose.position.x = -0.01
      grasp_pose.position.y = -0.02
      grasp_pose.position.z = 0.200
      orientation = quaternion_from_euler(0, pi/2, 0)
      grasp_pose.orientation.x = orientation[0]
      grasp_pose.orientation.y = orientation[1]
      grasp_pose.orientation.z = orientation[2]
      grasp_pose.orientation.w = orientation[3]

      pre_grasp_pose = Pose()
      pre_grasp_pose.position.x = -0.01
      pre_grasp_pose.position.y = -0.02
      pre_grasp_pose.position.z = 0.300
      orientation = quaternion_from_euler(0, pi/2, 0)
      pre_grasp_pose.orientation.x = orientation[0]
      pre_grasp_pose.orientation.y = orientation[1]
      pre_grasp_pose.orientation.z = orientation[2]
      pre_grasp_pose.orientation.w = orientation[3]

      post_grasp_pose = Pose()
      post_grasp_pose.position.x = 0
      post_grasp_pose.position.y = 0.05
      post_grasp_pose.position.z = 0.300
      orientation = quaternion_from_euler(0, pi/2, 0)
      post_grasp_pose.orientation.x = orientation[0]
      post_grasp_pose.orientation.y = orientation[1]
      post_grasp_pose.orientation.z = orientation[2]
      post_grasp_pose.orientation.w = orientation[3]

    elif req.object_type == "basket":
      grasp_pose = Pose()
      grasp_pose.position.x = -0.18
      grasp_pose.position.y = 0.03
      grasp_pose.position.z = -0.15
      orientation = quaternion_from_euler(-pi/2, 0, 0)
      grasp_pose.orientation.x = orientation[0]
      grasp_pose.orientation.y = orientation[1]
      grasp_pose.orientation.z = orientation[2]
      grasp_pose.orientation.w = orientation[3]

      pre_grasp_pose = Pose()
      pre_grasp_pose.position.x = -0.25
      pre_grasp_pose.position.y = 0.03
      pre_grasp_pose.position.z = -0.15
      orientation = quaternion_from_euler(-pi/2, 0, 0)
      pre_grasp_pose.orientation.x = orientation[0]
      pre_grasp_pose.orientation.y = orientation[1]
      pre_grasp_pose.orientation.z = orientation[2]
      pre_grasp_pose.orientation.w = orientation[3]

      post_grasp_pose = Pose()
      post_grasp_pose.position.x = -0.18
      post_grasp_pose.position.y = 0.13
      post_grasp_pose.position.z = -0.10
      orientation = quaternion_from_euler(-pi/2, 0, 0)
      post_grasp_pose.orientation.x = orientation[0]
      post_grasp_pose.orientation.y = orientation[1]
      post_grasp_pose.orientation.z = orientation[2]
      post_grasp_pose.orientation.w = orientation[3]
    else:
      raise NotImplementedError("The type {}, is not a known object type within the grasp server")

    resp.grasp_pose = grasp_pose
    resp.pre_grasp_pose = pre_grasp_pose
    resp.post_grasp_pose = post_grasp_pose
    resp.flag = 1
    self.pub_grasp_pose(resp.grasp_pose, resp.frame_id)
    print(resp)
    return resp

  def pub_grasp_pose(self, grasp_pose, frame_id):
    # get grasp pose
    grasp_marker = Marker()
    grasp_marker.header.frame_id = frame_id
    grasp_marker.type = Marker.MESH_RESOURCE
    grasp_marker.mesh_resource = "package://retail_store_simulation/models/gripper.dae"
    grasp_marker.action = Marker.ADD
    grasp_marker.pose = grasp_pose
    grasp_marker.scale.x = 1.0
    grasp_marker.scale.y = 1.0
    grasp_marker.scale.z = 1.0
    grasp_marker.color.a = 1.0
    grasp_marker.color.r = 1.0
    grasp_marker.color.g = 0.0
    grasp_marker.color.b = 0.0
    self.pub.publish(grasp_marker)

if __name__ == "__main__":
  GroceryStoreGraspPoseServer()
  rospy.spin()
