#!/usr/bin/env python

import rospy
import actionlib
from tiago_custom_msgs.msg import PickAction
from itertools import cycle
from copy import copy

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



class PickActionClass(object):
    def __init__(self, side="right"):
        # Initialize action server
        self._action_name = "pick_server_" + side
        self._side = side
        self._as = actionlib.SimpleActionServer(self._action_name, PickAction, execute_cb=self.as_cb, auto_start = False)
        self._as.start()

        # Initialize grocery store collision object server clients
        rospy.wait_for_service('add_collision_object', timeout=2)
        rospy.wait_for_service('remove_collision_object', timeout=2)
        rospy.wait_for_service('get_grasp_pose', timeout=2)
        self._add_co = rospy.ServiceProxy('add_collision_object', addCollObjByAruco)
        self._remove_co = rospy.ServiceProxy('remove_collision_object', removeCollObj)
        self._get_grasp_pose = rospy.ServiceProxy('get_grasp_pose', getGraspPose)

        # Initialize publishers/subscribers
        self._aruco_sub = rospy.Subscriber("aruco_marker_publisher/markers", MarkerArray, self.aruco_cb)
        self._vis_pub = rospy.Publisher("visualization_marker", VisMarker, queue_size=0)
        
        # Moveit interfaces
        self._scene = moveit_commander.PlanningSceneInterface()
        self._group = moveit_commander.MoveGroupCommander("arm_{}_torso".format(self._side))
        self._robot = moveit_commander.RobotCommander()
        self._group.set_max_velocity_scaling_factor(0.6) # Increase group velocity

        # Aruco stuff
        self._aruco_marker = None
        self._aruco_id = -1
        
    def publish_vis_marker(self, pose, id):
        # Method to display a marker in RViz
        marker = VisMarker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time()
        marker.ns = "marker_vis"
        marker.id = id
        marker.type = VisMarker.ARROW
        marker.action = VisMarker.ADD
        marker.pose = pose
        marker.scale.x = 0.1
        marker.scale.y = 0.01
        marker.scale.z = 0.01
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        self._vis_pub.publish(marker)

    def aruco_cb(self, msg):
        for marker in msg.markers:
            if marker.id == self._aruco_id and self._aruco_marker == None:
                #self.publish_vis_marker(marker.pose.pose, marker.id)
                self._aruco_marker = marker

    def as_cb(self, goal):
        print(goal)
        self._aruco_id = goal.aruco_id
        self._aruco_marker = None

        # Prerequisites (open gripper and detached and remove objects)
        gripper = GripperControl(self._side)
        gripper.run('open')
        eef_link = self._group.get_end_effector_link()
        self._scene.remove_attached_object(link=eef_link)
        self._scene.remove_world_object()

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
        rospy.loginfo("Aruco marker pose: {}".format(aruco_pose))

        # 2. Add cube as collision object in Moveit
        # Note: this step uses the collision_object_server
        req = addCollObjByArucoRequest()
        req.object_id = str(self._aruco_id)
        req.object_type = goal.object_type
        req.aruco_pose = self._aruco_marker.pose.pose
        # TODO: catch service errors
        self._add_co(req)
        rospy.sleep(1)

        # 3. Get grasp position 
        # Note: this step uses the grasp_pose_server
        req = getGraspPoseRequest()
        req.object_id = str(self._aruco_id)
        req.object_type = goal.object_type
        res = self._get_grasp_pose(req)
        
        # 4.0 Move to pre-grasp pose
        pre_goal = PoseStamped()
        pre_goal.pose = res.pre_grasp_pose
        pre_goal.pose.position.z += 0.15
        pre_goal.header.frame_id = res.frame_id
        self._group.set_pose_target(pre_goal)
        succeeded = self._group.go(wait=True)
        if not succeeded:
            print("failed to get pre-grasp pose")
            self._as.set_succeeded(succeeded)
            return
        rospy.loginfo("approaching pre-grasp pose")
        

        # 4.1 Go to grasp position
        rospy.sleep(0.1)
        goal = PoseStamped()
        goal.pose = res.grasp_pose
        goal.pose.position.y -= 0.025
        goal.header.frame_id = res.frame_id
        self._group.set_pose_target(goal)
        succeeded = self._group.go(wait=True)
        if not succeeded:
            print("failed to grasp")
            self._as.set_succeeded(succeeded)
            return

        # 5. close gripper and attach collision object
        gripper.run('close')
        rospy.loginfo("closed gripper")
        grasping_group = "gripper_{}".format(self._side)
        touch_links = self._robot.get_link_names(group=grasping_group)
        eef_link = self._group.get_end_effector_link()
        self._scene.attach_box(eef_link, str(self._aruco_id), touch_links=touch_links)

        # 6. Move to post-grasp pose
        rospy.sleep(0.1)
        self._group.set_pose_target(pre_goal)
        succeeded = self._group.go(wait=True)

        # 6. return arm to tuck
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
        
        # 6.5 remove cube as collision object in Moveit
        # Note: this step uses the collision_object_server
        req = removeCollObjResponse()
        req.object_id = str(self._aruco_id)
        req.object_type = goal.object_type
        req.aruco_pose = self._aruco_marker.pose.pose
        # TODO: catch service errors
        self._remove_co(req)
        rospy.loginfo("Removed collision object.")
        rospy.sleep(0.5)

        # 7. return head to center and remove table
        head_control.run(Point(x=1.0, y=0.0, z=1.1))
        self._scene.remove_world_object("table")

        self._as.set_aborted()

if __name__ == "__main__":
    rospy.init_node("pick_server")
    rospy.loginfo("pick servers started")

    PickActionClass(side="right")
    PickActionClass(side="left")

    rospy.spin()
