#!/usr/bin/env python

import rospy
import actionlib
from tiago_custom_msgs.msg import PlaceAction
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from tf import TransformListener
from itertools import cycle
from copy import copy, deepcopy
from math import pi

# For arm control
import moveit_commander
from geometry_msgs.msg import PoseStamped, Point
from grocery_store_utils.srv import *

# For aruco
from aruco_msgs.msg import MarkerArray
from visualization_msgs.msg import Marker as VisMarker

# For gripper control
from gripper_control import GripperControl
# from gazebo_ros_link_attacher.srv import Attach, AttachRequest, AttachResponse

# For head control
from look_to_point import LookToPoint

# To tuck the arm
from play_motion_msgs.msg import PlayMotionAction, PlayMotionGoal
from sensor_msgs.msg import JointState


class PlaceActionClass(object):
    def __init__(self, side="right"):
        # Initialize action server
        self._action_name = "place_server_" + side
        self._side = side
        self._as = actionlib.SimpleActionServer(self._action_name, PlaceAction, execute_cb=self.as_cb, auto_start = False)
        self._as.start()

        # Settings
        self._aruco_marker = None
        self._aruco_id = -1
        self._tl = TransformListener()

        # Initialize grocery store collision object server clients
        rospy.wait_for_service('add_collision_object', timeout=2)
        rospy.wait_for_service('remove_collision_object', timeout=2)
        rospy.wait_for_service('get_grasp_pose', timeout=2)
        self._add_co = rospy.ServiceProxy('add_collision_object', addCollObjByAruco)
        self._remove_co = rospy.ServiceProxy('remove_collision_object', removeCollObj)
        self._get_grasp_pose = rospy.ServiceProxy('get_grasp_pose', getGraspPose)

        # Initialize publishers/subscribers
        self._aruco_sub = rospy.Subscriber("aruco_marker_publisher/markers", MarkerArray, self.aruco_cb)
        self._vis_pub = rospy.Publisher("/grocery_store_grasp_pose_server/pose_goal", VisMarker, queue_size=0)

        # Moveit interfaces
        self._scene = moveit_commander.PlanningSceneInterface()
        self._group = moveit_commander.MoveGroupCommander("arm_{}_torso".format(self._side))
        self._robot = moveit_commander.RobotCommander()
        self._group.set_max_velocity_scaling_factor(0.6) # Increase group velocity

    def aruco_cb(self, msg):
        for marker in msg.markers:
            if marker.id == self._aruco_id and self._aruco_marker == None:
                #self.publish_vis_marker(marker.pose.pose, marker.id)
                self._aruco_marker = marker

    def publish_vis_marker(self, pose, id):
        # Method to display a marker in RViz
        marker = VisMarker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time()
        marker.ns = "marker_vis"
        marker.id = id
        marker.type = VisMarker.MESH_RESOURCE
        marker.mesh_resource = "package://retail_store_simulation/models/gripper.dae"
        marker.action = VisMarker.ADD
        marker.pose = pose
        marker.scale.x = 1.0
        marker.scale.y = 1.0
        marker.scale.z = 1.0
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 1.0
        self._vis_pub.publish(marker)

    def look_for_place_pose(self, aruco_id):
        self._aruco_id = aruco_id
        self._aruco_marker = None

        # 1. Look around for aruco marker place pose
        points = [
            Point(x=1.0, y=0.0, z=1.1),
            Point(x=1.0, y=0.0, z=0.0),
            Point(x=1.0, y=-0.7, z=0.0),
            Point(x=1.0, y=0.7, z=0.0),
            Point(x=1.0, y=0.0, z=0.6)
        ]

        head_control = LookToPoint()
        for point in cycle(points):
            head_control.run(point)
            if self._aruco_marker is not None: break
        # After finding the aruco marker look slightly below it to capture table in octomap. 
        aruco_pose = copy(self._aruco_marker.pose.pose.position)
        aruco_pose.z -= 0.3
        head_control.run(aruco_pose, frame_id='map')

        # 2. Create collision object for shelf

        # 3. Calculate and Visualize place pose
        arm_goal = PoseStamped()
        arm_goal.pose = copy(self._aruco_marker.pose.pose)
        arm_goal.header.frame_id = "map"
        arm_goal.pose.position.x += 0
        arm_goal.pose.position.y += 0.15
        arm_goal.pose.position.z += 0.15 
        orientation = quaternion_from_euler(pi/2, 0, -pi/2)
        arm_goal.pose.orientation.x = orientation[0]
        arm_goal.pose.orientation.y = orientation[1]
        arm_goal.pose.orientation.z = orientation[2]
        arm_goal.pose.orientation.w = orientation[3]
        return arm_goal

    def get_place_pose_from_goal(self, goal, use_tiago_orientation=True):
        #temp
        # goal.position.x = 0.3
        # goal.position.y = -2.0
        # goal.position.z = 1.50

        if use_tiago_orientation:
            # Lookup orientation of Tiago 
            (_, rot) = self._tl.lookupTransform('map', 'base_footprint', rospy.Time(0))
            (r, p, y) = euler_from_quaternion(rot)
            orientation = quaternion_from_euler(r+pi/2, p, y)
        else:
            orientation = quaternion_from_euler(pi/2, 0, -pi/2)

        arm_goal = PoseStamped()
        point_stamped = self._tl.transformPoint("map", goal.position)
        arm_goal.pose.position = point_stamped.point
        arm_goal.pose.orientation.x = orientation[0]
        arm_goal.pose.orientation.y = orientation[1]
        arm_goal.pose.orientation.z = orientation[2]
        arm_goal.pose.orientation.w = orientation[3]
        arm_goal.header.frame_id = "map"
    
        base_goal = self._tl.transformPose("base_footprint", arm_goal)
        print("base_goal: {}".format(base_goal))
        return arm_goal

    def as_cb(self, goal):
        print(goal)
            
        # If an aruco id is passed in the goal search for aruco else use goal.pose
        if goal.aruco_id != 0:
            arm_goal = self.look_for_place_pose(goal.aruco_id)
        else:
            arm_goal = self.get_place_pose_from_goal(goal)

        self.publish_vis_marker(arm_goal.pose, goal.aruco_id)

        # Look around to fill out Octomap
        points = [
            Point(x=1.0, y=0.0, z=1.1),
            Point(x=1.0, y=0.0, z=0.0),
            Point(x=1.0, y=-0.7, z=0.0),
            Point(x=1.0, y=0.7, z=0.0),
            Point(x=1.0, y=0.0, z=0.6)
        ]

        head_control = LookToPoint()
        for point in points:
            head_control.run(point)
            
        # Move arm to pre-place pose
        rospy.sleep(0.3)
        arm_goal_local = self._tl.transformPose("base_footprint", arm_goal)
        if self._side == "right":
            arm_goal_local.pose.position.y -= 0.2
        elif self._side == "left":
            arm_goal_local.pose.position.y += 0.2
        else:
            print("Wrong side")

        pre_goal_local = deepcopy(arm_goal_local)
        pre_goal_local.pose.position.x -= 0.15
        self._group.set_pose_target(pre_goal_local)
        succeeded = self._group.go(wait=True)

        # 4. Move TIAGo's arm to the pose that you want to place the object (mind the height)
        rospy.sleep(0.3)
        self._group.set_pose_target(arm_goal_local)
        succeeded = self._group.go(wait=True)
        if not succeeded:
            print("failed to place")
            self._as.set_aborted()
            return

        # 5. Open the gripper
        rospy.sleep(0.5)
        gripper = GripperControl(self._side)
        gripper.run('open')
        eef_link = self._group.get_end_effector_link()
        # remove cube as collision object
        self._scene.remove_attached_object(link=eef_link)
        self._scene.remove_world_object()
        
        # return pre-place pose
        self._group.set_pose_target(pre_goal_local)
        succeeded = self._group.go(wait=True)

        # 6. Move the arm back to a neutral position
        rospy.sleep(1.0)
        client = actionlib.SimpleActionClient("play_motion", PlayMotionAction)
        client.wait_for_server()
        rospy.loginfo("...connected.")
        rospy.wait_for_message("joint_states", JointState)
        rospy.sleep(1.0)
        rospy.loginfo("Tuck arm...")
        goal = PlayMotionGoal()
        goal.motion_name = 'home_'+self._side
        goal.skip_planning = False
        client.send_goal(goal)
        client.wait_for_result(rospy.Duration(20.0))
        rospy.loginfo("Arm tucked.")

        # return head to center
        head_control = LookToPoint()
        head_control.run(Point(x=1.0, y=0.0, z=1.1))

        self._as.set_succeeded()

if __name__ == "__main__":
    rospy.init_node("place_server")
    rospy.loginfo("place servers started")

    PlaceActionClass(side="right")
    PlaceActionClass(side="left")

    rospy.spin()

