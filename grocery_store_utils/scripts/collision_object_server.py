#!/usr/bin/env python
import rospy
import sys
import rospy
import rospkg
import moveit_commander

from moveit_msgs.msg import CollisionObject
from geometry_msgs.msg import PoseStamped, Pose, TransformStamped
from shape_msgs.msg import SolidPrimitive
from aruco_msgs.msg import MarkerArray

from tf.transformations import quaternion_from_euler, euler_from_quaternion
import tf
from math import pi
from grocery_store_utils.srv import *
from std_srvs.srv import Empty


class GroceryStoreCollisionObjectServer:
  def __init__(self):
    rospy.init_node("grocery_store_collision_object_server")

    rospack = rospkg.RosPack()
    self.model_path = rospack.get_path('retail_store_simulation')+"/models/"

    self.ignore_orientation = False # Aruco orientation can be noisy, if you want to align the orientation with 'base_footprint' set this flag to true

    # Available services
    rospy.Service('add_collision_object', addCollObjByAruco, self.add_coll_obj_cb) # Only ByAruco is supported currently
    rospy.Service('remove_collision_object', removeCollObj, self.remove_coll_obj_cb)

    # rospy.wait_for_service('/clear_octomap') #this will stop your code until the clear octomap service starts running
    # self.clear_octomap = rospy.ServiceProxy('/clear_octomap', Empty)

    # Initialize moveit scene 
    moveit_commander.roscpp_initialize(sys.argv)
    self.planning_scene_interface = moveit_commander.PlanningSceneInterface()

    self.tl = tf.TransformListener()
    self.br = tf.TransformBroadcaster()

  def remove_coll_obj_cb(self, req):
    # TODO: Make this function gripper side / frame independent
    resp = removeCollObjResponse()
    self.planning_scene_interface.remove_attached_object("gripper_right_link")
    self.planning_scene_interface.remove_world_object(req.object_id)
    resp.flag = 1
    return resp
    
  def add_coll_obj_cb(self, req):
    resp = addCollObjByArucoResponse()
    resp.flag = 1

    aruco_frame_id = "aruco_marker_{}".format(req.object_id)
    aruco_frame_msg = TransformStamped()
    aruco_frame_msg.header.frame_id = "map"
    aruco_frame_msg.child_frame_id = aruco_frame_id
    aruco_frame_msg.transform.translation = req.aruco_pose.position

    if not self.ignore_orientation:
        aruco_frame_msg.transform.rotation = req.aruco_pose.orientation
    else:
        # Using base_footprint frame to calculate orientation of object, because aruco marker detected orientations are bad
        (_, rot) = self.tl.lookupTransform('map', 'base_footprint', rospy.Time(0))
        (r, p, y) = euler_from_quaternion(rot)
        orientation = quaternion_from_euler(r-pi/2, p+pi, y+pi/2)
        aruco_frame_msg.transform.rotation.x = orientation[0]
        aruco_frame_msg.transform.rotation.y = orientation[1]
        aruco_frame_msg.transform.rotation.z = orientation[2]
        aruco_frame_msg.transform.rotation.w = orientation[3]

    self.br.sendTransformMessage(aruco_frame_msg)

    rospy.sleep(0.2)

    flag = self.register_collision_object(aruco_frame_id, req.object_type, req.object_id)
    if not flag:
      print("object type {} not known! Please enter a valid object type in request".format(req.object_type))

    return resp

  def register_collision_object(self, aruco_frame_id, object_type, object_id):
    ## Add your collision object below
    if object_type == "cube_top":
      cube_pose = Pose()
      cube_pose.position.x = 0.0
      cube_pose.position.y = 0.0
      cube_pose.position.z = -0.035
      cube_pose.orientation.w = 1.0

      # create object
      coll_obj = CollisionObject()
      coll_obj.header.frame_id = aruco_frame_id
      coll_obj.operation = coll_obj.ADD
      box = SolidPrimitive()
      box.type = box.BOX
      box.dimensions = [0.07, 0.07, 0.10]
      coll_obj.id = object_id
      coll_obj.primitives.append(box)
      coll_obj.primitive_poses.append(cube_pose)

      self.planning_scene_interface.add_object(coll_obj)

      # add short table section below cube
      coll_obj = CollisionObject()
      coll_obj.header.frame_id = aruco_frame_id
      coll_obj.operation = coll_obj.ADD
      box = SolidPrimitive()
      box.type = box.BOX
      box.dimensions = [0.50, 0.50, 0.02]
      coll_obj.id = "table"
      coll_obj.primitives.append(box)

      table_pose = cube_pose
      table_pose.position.z = -0.100
      coll_obj.primitive_poses.append(table_pose)

      self.planning_scene_interface.add_object(coll_obj)

    if object_type == "cube_side":
      cube_pose = Pose()
      cube_pose.position.x = 0.0
      cube_pose.position.y = -0.025
      cube_pose.position.z = -0.03
      cube_pose.orientation.w = 1.0

      # create object
      coll_obj = CollisionObject()
      coll_obj.header.frame_id = aruco_frame_id
      coll_obj.operation = coll_obj.ADD
      box = SolidPrimitive()
      box.type = box.BOX
      box.dimensions = [0.07, 0.09, 0.07]
      coll_obj.id = object_id
      coll_obj.primitives.append(box)
      coll_obj.primitive_poses.append(cube_pose)

      self.planning_scene_interface.add_object(coll_obj)

      # add short table section below cube
      coll_obj = CollisionObject()
      coll_obj.header.frame_id = aruco_frame_id
      coll_obj.operation = coll_obj.ADD
      box = SolidPrimitive()
      box.type = box.BOX
      box.dimensions = [0.50, 0.02, 0.50]
      coll_obj.id = "table"
      coll_obj.primitives.append(box)

      table_pose = cube_pose
      table_pose.position.y = -0.083
      coll_obj.primitive_poses.append(table_pose)

      self.planning_scene_interface.add_object(coll_obj)

    if object_type == "hagelslag":
      cube_pose = Pose()
      cube_pose.position.x = 0.0
      cube_pose.position.y = -0.025
      cube_pose.position.z = -0.025
      cube_pose.orientation.w = 1.0

      # create object
      coll_obj = CollisionObject()
      coll_obj.header.frame_id = aruco_frame_id
      coll_obj.operation = coll_obj.ADD
      box = SolidPrimitive()
      box.type = box.BOX
      box.dimensions = [0.07, 0.16, 0.07]
      coll_obj.id = object_id
      coll_obj.primitives.append(box)
      coll_obj.primitive_poses.append(cube_pose)

      self.planning_scene_interface.add_object(coll_obj)

      # add short table section below cube
      coll_obj = CollisionObject()
      coll_obj.header.frame_id = aruco_frame_id
      coll_obj.operation = coll_obj.ADD
      box = SolidPrimitive()
      box.type = box.BOX
      box.dimensions = [0.50, 0.02, 0.50]
      coll_obj.id = "table"
      coll_obj.primitives.append(box)

      table_pose = cube_pose
      table_pose.position.y = -0.153
      coll_obj.primitive_poses.append(table_pose)

      self.planning_scene_interface.add_object(coll_obj)


    if object_type == "carton":
      carton_pose = Pose()
      carton_pose.position.x = -0.01
      carton_pose.position.y = -0.05
      carton_pose.position.z = -0.030
      carton_pose.orientation.w = 1.0

      # create object
      coll_obj = CollisionObject()
      coll_obj.header.frame_id = aruco_frame_id
      coll_obj.operation = coll_obj.ADD
      box = SolidPrimitive()
      box.type = box.BOX
      box.dimensions = [0.06, 0.22, 0.06]
      coll_obj.id = object_id
      coll_obj.primitives.append(box)
      coll_obj.primitive_poses.append(carton_pose)

      self.planning_scene_interface.add_object(coll_obj)

    elif object_type == "can":
      can_pose = Pose()
      can_pose.position.x = 0
      can_pose.position.y = 0
      can_pose.position.z = -0.030
      orientation = quaternion_from_euler(pi/2, 0, 0)
      can_pose.orientation.x = orientation[0]
      can_pose.orientation.y = orientation[1]
      can_pose.orientation.z = orientation[2]
      can_pose.orientation.w = orientation[3]

      # create object
      obj_pose_stamped = PoseStamped()
      obj_pose_stamped.header.frame_id = aruco_frame_id
      obj_pose_stamped.pose = can_pose

      self.planning_scene_interface.add_cylinder(object_id, obj_pose_stamped, height=0.12, radius=0.03)

    elif object_type == "thee":
      thee_pose = Pose()
      thee_pose.position.x = 0
      thee_pose.position.y = -0.020
      thee_pose.position.z = -0.030
      thee_pose.orientation.w = 1.0

      # create object
      obj_pose_stamped = PoseStamped()
      obj_pose_stamped.header.frame_id = aruco_frame_id
      obj_pose_stamped.pose = thee_pose

      self.planning_scene_interface.add_box(object_id, obj_pose_stamped, size=(0.05, 0.12, 0.07))

    elif object_type == "basket":
      basket_pose = Pose()
      basket_pose.position.x = 0
      basket_pose.position.y = -0.20
      basket_pose.position.z = -0.15
      basket_pose.orientation.w = 1.0
      orientation = quaternion_from_euler(0, -pi/2, -pi/2)
      basket_pose.orientation.x = orientation[0]
      basket_pose.orientation.y = orientation[1]
      basket_pose.orientation.z = orientation[2]
      basket_pose.orientation.w = orientation[3]

      # create object
      obj_pose_stamped = PoseStamped()
      obj_pose_stamped.header.frame_id = aruco_frame_id
      obj_pose_stamped.pose = basket_pose
      # obj_pose_stamped = self.tl.transformPose("map", obj_pose_stamped)

      self.planning_scene_interface.add_mesh(object_id, obj_pose_stamped, self.model_path+"/AH_store/basket_with_aruco/meshes/basket_with_aruco.stl", size=(1, 1, 1))

    else:
      return True

    return False


if __name__ == "__main__":
  n = GroceryStoreCollisionObjectServer()
  # rate = rospy.Rate(10)

  # while not rospy.is_shutdown():
  #   # Manual publishing of frames to tf
  #   obj_names = n.planning_scene_interface.get_known_object_names()
  #   objs = n.planning_scene_interface.get_object_poses(obj_names)

  #   for obj_name in obj_names:
  #     obj = objs[obj_name]
  #     obj_frame_msg = TransformStamped()
  #     obj_frame_msg.header.frame_id = "map"
  #     obj_frame_msg.child_frame_id = obj_name
  #     obj_frame_msg.transform.translation = obj.position
  #     obj_frame_msg.transform.rotation = obj.orientation
  #     n.br.sendTransformMessage(obj_frame_msg)

  #   rate.sleep()

  rospy.spin()
