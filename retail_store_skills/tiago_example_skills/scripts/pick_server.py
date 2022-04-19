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
        rospy.loginfo("The initialization of PickActionClass in pick_server.py starts")  # At the beginning of the second terminal!
        rospy.loginfo("The side is: %s", side) 
        # print("This is ", self._side) # This is not ok
        # execute_cb is ExecuteCallback
        self._as = actionlib.SimpleActionServer(self._action_name, PickAction, execute_cb=self.as_cb, auto_start = False)  # no duplicate
        self._as.start()
        rospy.loginfo("The initialization of PickActionClass continues")

        # Initialize grocery store collision object server clients
        rospy.wait_for_service('add_collision_object', timeout=2)
        rospy.wait_for_service('remove_collision_object', timeout=2)
        rospy.wait_for_service('get_grasp_pose', timeout=2)
        self._add_co = rospy.ServiceProxy('add_collision_object', addCollObjByAruco)  # no duplicate
        self._remove_co = rospy.ServiceProxy('remove_collision_object', removeCollObj) # no duplicate
        self._get_grasp_pose_l = rospy.ServiceProxy('get_grasp_pose', getGraspPose)
        self._get_grasp_pose_r = rospy.ServiceProxy('get_grasp_pose', getGraspPose)

        # Initialize publishers/subscribers
        self._aruco_sub = rospy.Subscriber("aruco_marker_publisher/markers", MarkerArray, self.aruco_cb) #?
        self._vis_pub = rospy.Publisher("visualization_marker", VisMarker, queue_size=0)
        
        # Moveit interfaces
        self._scene = moveit_commander.PlanningSceneInterface()  # no duplicate
        self._group_l = moveit_commander.MoveGroupCommander("arm_{}_torso".format("left"))
        self._group_r = moveit_commander.MoveGroupCommander("arm_{}_torso".format("right"))
        self._robot = moveit_commander.RobotCommander()
        self._group_l.set_max_velocity_scaling_factor(0.6) # Increase group velocity
        self._group_r.set_max_velocity_scaling_factor(0.6) # Increase group velocity

        # Aruco stuff
        self._aruco_marker_l = None
        self._aruco_id_l = -1
        self._aruco_marker_r = None
        self._aruco_id_r = -1

        rospy.loginfo("The initialization of PickActionClass ends") # final part of initialization of second terminal
        
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
            if marker.id == self._aruco_id_l and self._aruco_marker_l == None:
                #self.publish_vis_marker(marker.pose.pose, marker.id)
                self._aruco_marker_l = marker
            if marker.id == self._aruco_id_r and self._aruco_marker_r == None:
                #self.publish_vis_marker(marker.pose.pose, marker.id)
                self._aruco_marker_r = marker

    def as_cb(self, goal):
        # STEP 2:: Once received the goals, use two groups to plan the arm
        rospy.loginfo("pick server starts processing the goal!")
        print(goal)
        self._aruco_id_l = goal.aruco_id / 100
        self._aruco_marker_l = None
        self._aruco_id_r = goal.aruco_id % 100
        self._aruco_marker_r = None

        # Prerequisites (open gripper and detached and remove objects)
        gripper_l = GripperControl("left")
        gripper_l.run('open')
        eef_link_l = self._group_l.get_end_effector_link()
        self._scene.remove_attached_object(link=eef_link_l)
        # self._scene.remove_world_object() ##  !!
        gripper_r = GripperControl("right")
        gripper_r.run('open')
        eef_link_r = self._group_r.get_end_effector_link()
        self._scene.remove_attached_object(link=eef_link_r)
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
            if (self._aruco_marker_l is not None) and (self._aruco_marker_r is not None): break
        # After finding the aruco marker look slightly below it to capture table in octomap. 
        # rospy.loginfo("I am printing now:")
        # print(self._aruco_marker_l)
        aruco_pose_l = copy(self._aruco_marker_l.pose.pose.position)  #!!! Nontype aruco_marker
        aruco_pose_l.z -= 0.3
        head_control.run(aruco_pose_l, frame_id='map')
        rospy.loginfo("Aruco marker pose: {}".format(aruco_pose_l))
        aruco_pose_r = copy(self._aruco_marker_r.pose.pose.position)
        aruco_pose_r.z -= 0.3
        head_control.run(aruco_pose_r, frame_id='map')
        rospy.loginfo("Aruco marker pose: {}".format(aruco_pose_r))

        # 2. Add cube as collision object in Moveit
        # Note: this step uses the collision_object_server
        req_l = addCollObjByArucoRequest()
        req_l.object_id = str(self._aruco_id_l)
        req_l.object_type = goal.object_type
        req_l.aruco_pose = self._aruco_marker_l.pose.pose
        # TODO: catch service errors
        self._add_co(req_l)
        rospy.sleep(0.5) ##!!
        req_r = addCollObjByArucoRequest()
        req_r.object_id = str(self._aruco_id_r)
        req_r.object_type = goal.object_type
        req_r.aruco_pose = self._aruco_marker_r.pose.pose
        # TODO: catch service errors
        self._add_co(req_r)
        rospy.sleep(0.5)

        # 3. Get grasp position 
        # Note: this step uses the grasp_pose_server
        req_l = getGraspPoseRequest()
        req_l.object_id = str(self._aruco_id_l)
        req_l.object_type = goal.object_type
        res_l = self._get_grasp_pose_l(req_l) ###!!
        req_r = getGraspPoseRequest()
        req_r.object_id = str(self._aruco_id_r)
        req_r.object_type = goal.object_type
        res_r = self._get_grasp_pose_r(req_r) ##!!
        
        # 4.0 Move to pre-grasp pose
        pre_goal_l = PoseStamped()
        pre_goal_l.pose = res_l.pre_grasp_pose
        pre_goal_l.pose.position.z += 0.12  # 0.15
        pre_goal_l.header.frame_id = res_l.frame_id
        pre_goal_r = PoseStamped()
        pre_goal_r.pose = res_r.pre_grasp_pose
        pre_goal_r.pose.position.z += 0.12 # 0.15
        pre_goal_r.header.frame_id = res_r.frame_id
        self._group_l.set_pose_target(pre_goal_l)
        self._group_r.set_pose_target(pre_goal_r)
        succeeded_l = self._group_l.go(wait=True)
        succeeded_r = self._group_r.go(wait=True)
        if not succeeded_l:
            print("left gripper failed to get pre-grasp pose")
            self._as.set_succeeded(succeeded_l)
            return
        rospy.loginfo("left gripper approaching pre-grasp pose")
        if not succeeded_r:
            print("right gripper failed to get pre-grasp pose")
            self._as.set_succeeded(succeeded_r)
            return
        rospy.loginfo("right gripper approaching pre-grasp pose")
        

        # 4.1 Go to grasp position
        rospy.sleep(0.1)
        goal_l = PoseStamped()
        goal_l.pose = res_l.grasp_pose
        goal_l.pose.position.y -= 0.025
        goal_l.header.frame_id = res_l.frame_id
        rospy.sleep(0.1)
        goal_r = PoseStamped()
        goal_r.pose = res_r.grasp_pose
        goal_r.pose.position.y -= 0.025
        goal_r.header.frame_id = res_r.frame_id
        self._group_l.set_pose_target(goal_l)
        self._group_r.set_pose_target(goal_r)
        succeeded_l = self._group_l.go(wait=True)
        succeeded_r = self._group_r.go(wait=True)
        if not succeeded_l:
            print("left gripper failed to grasp")
            self._as.set_succeeded(succeeded_l)
            return  ##
        if not succeeded_r:
            print("right gripper failed to grasp")
            self._as.set_succeeded(succeeded_r)
            return

        # 5. close gripper and attach collision object
        gripper_l.run('close')
        rospy.loginfo("closed left gripper")
        grasping_group_l = "gripper_{}".format("left")
        touch_links_l = self._robot.get_link_names(group=grasping_group_l)  # should _robot duplicate? Maybe No?
        eef_link_l = self._group_l.get_end_effector_link()
        self._scene.attach_box(eef_link_l, str(self._aruco_id_l), touch_links=touch_links_l) ##
        gripper_r.run('close')
        rospy.loginfo("closed right gripper")
        grasping_group_r = "gripper_{}".format("right")
        touch_links_r = self._robot.get_link_names(group=grasping_group_r)
        eef_link_r = self._group_r.get_end_effector_link()
        self._scene.attach_box(eef_link_r, str(self._aruco_id_r), touch_links=touch_links_r) # should scene duplicate? Maybe no

        # 6. Move to post-grasp pose
        rospy.sleep(0.1)
        self._group_l.set_pose_target(pre_goal_l)
        succeeded = self._group_l.go(wait=True)
        rospy.sleep(0.1)
        self._group_r.set_pose_target(pre_goal_r)
        succeeded = self._group_r.go(wait=True)

        # 6. return arm to tuck
        client = actionlib.SimpleActionClient("play_motion", PlayMotionAction) # should client duplicate? Maybe no
        client.wait_for_server()
        rospy.loginfo("...connected.")
        rospy.wait_for_message("joint_states", JointState)
        rospy.sleep(1.0) ##
        rospy.loginfo("Tuck left arm...")
        goal_l = PlayMotionGoal()
        goal_l.motion_name = 'home_'+'left'
        goal_l.skip_planning = False
        client.send_goal(goal_l)
        client.wait_for_result(rospy.Duration(20.0))
        rospy.loginfo("Left arm tucked.") ##
        rospy.loginfo("Tuck right arm...")
        goal_r = PlayMotionGoal()
        goal_r.motion_name = 'home_'+'right'
        goal_r.skip_planning = False
        client.send_goal(goal_r)
        client.wait_for_result(rospy.Duration(20.0))
        rospy.loginfo("Right arm tucked.")
        
        # 6.5 remove cube as collision object in Moveit
        # Note: this step uses the collision_object_server
        req_l= removeCollObjResponse()
        req_l.object_id = str(self._aruco_id_l)
        req_l.object_type = goal.object_type
        req_l.aruco_pose = self._aruco_marker_l.pose.pose
        # TODO: catch service errors
        self._remove_co(req_l)
        rospy.loginfo("Removed collision object.")
        # rospy.sleep(0.5) ##
        req_r = removeCollObjResponse()
        req_r.object_id = str(self._aruco_id_r)
        req_r.object_type = goal.object_type
        req_r.aruco_pose = self._aruco_marker_r.pose.pose
        # TODO: catch service errors
        self._remove_co(req_r)
        rospy.loginfo("Removed collision object.")
        rospy.sleep(0.5)

        # 7. return head to center and remove table
        head_control.run(Point(x=1.0, y=0.0, z=1.1))
        self._scene.remove_world_object("table")

        self._as.set_aborted()

if __name__ == "__main__":
    rospy.init_node("pick_server")
    rospy.loginfo("pick servers started")

    rospy.loginfo("PickActionClass right starts")
    PickActionClass(side="right")
    rospy.loginfo("PickActionClass left starts")
    PickActionClass(side="left") # These two classes are necessary

    rospy.spin()
