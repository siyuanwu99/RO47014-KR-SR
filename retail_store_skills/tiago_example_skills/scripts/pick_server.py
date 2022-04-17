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

import multiprocessing
import threading
import time
# from pathos.pools import ProcessPool


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
        self._robot_l = moveit_commander.RobotCommander()
        self._robot_r = moveit_commander.RobotCommander()
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
        self._aruco_id_l = goal.aruco_id / 100
        self._aruco_marker_l = None
        self._aruco_id_r = goal.aruco_id % 100
        self._aruco_marker_r = None

        # # Prerequisites (open gripper and detached and remove objects)
        # self.gripper_l = GripperControl("left")
        # self.gripper_l.run('open')
        # eef_link_l = self._group_l.get_end_effector_link()
        # self._scene.remove_attached_object(link=eef_link_l)
        # # self._scene.remove_world_object() ##  !!
        # self.gripper_r = GripperControl("right")
        # self.gripper_r.run('open')
        # eef_link_r = self._group_r.get_end_effector_link()
        # self._scene.remove_attached_object(link=eef_link_r)
        # self._scene.remove_world_object()

        # # 1. Look around for aruco marker place pose
        # points = [
        #     Point(x=1.0, y=0.0, z=1.1),
        #     Point(x=1.0, y=0.0, z=0.0),
        #     Point(x=1.0, y=-0.7, z=0.0),
        #     Point(x=1.0, y=0.7, z=0.0),
        #     Point(x=1.0, y=0.0, z=0.6)
        # ]

        # head_control = LookToPoint()
        # for point in cycle(points):
        #     head_control.run(point)
        #     if (self._aruco_marker_l is not None) and (self._aruco_marker_r is not None): break
        # # After finding the aruco marker look slightly below it to capture table in octomap. 
        # # rospy.loginfo("I am printing now:")
        # # print(self._aruco_marker_l)
        # aruco_pose_l = copy(self._aruco_marker_l.pose.pose.position)  #!!! Nontype aruco_marker
        # aruco_pose_l.z -= 0.3
        # head_control.run(aruco_pose_l, frame_id='map')
        # rospy.loginfo("Aruco marker pose: {}".format(aruco_pose_l))
        # aruco_pose_r = copy(self._aruco_marker_r.pose.pose.position)
        # aruco_pose_r.z -= 0.3
        # head_control.run(aruco_pose_r, frame_id='map')
        # rospy.loginfo("Aruco marker pose: {}".format(aruco_pose_r))

        # # 2. Add cube as collision object in Moveit
        # # Note: this step uses the collision_object_server
        # req_l = addCollObjByArucoRequest()
        # req_l.object_id = str(self._aruco_id_l)
        # req_l.object_type = goal.object_type
        # req_l.aruco_pose = self._aruco_marker_l.pose.pose
        # # TODO: catch service errors
        # self._add_co(req_l)
        # rospy.sleep(0.5) ##!!
        # req_r = addCollObjByArucoRequest()
        # req_r.object_id = str(self._aruco_id_r)
        # req_r.object_type = goal.object_type
        # req_r.aruco_pose = self._aruco_marker_r.pose.pose
        # # TODO: catch service errors
        # self._add_co(req_r)
        # rospy.sleep(0.5)

        # # 3. Get grasp position 
        # # Note: this step uses the grasp_pose_server
        # req_l = getGraspPoseRequest()
        # req_l.object_id = str(self._aruco_id_l)
        # req_l.object_type = goal.object_type
        # self.res_l = self._get_grasp_pose_l(req_l) ###!!
        # req_r = getGraspPoseRequest()
        # req_r.object_id = str(self._aruco_id_r)
        # req_r.object_type = goal.object_type
        # self.res_r = self._get_grasp_pose_r(req_r) ##!!

        # Start moving two grippers simultanuously # multiprocessing.Process
        tic = time.time()
        # # version 3
        # lists = ["left", "right"]
        # pool = ProcessPool(nodes=2)
        # pool.map(self.sleepy_man(self, ), lists) # ONLY the second one is processed

        # version 2 not working https://stackoverflow.com/questions/70968015/cant-use-pool-map-for-a-class-method-in-ros-python
        lists = ["left", "right"]
        self.num = 0
        pool = multiprocessing.Pool(processes=2)
        pool.map(self.sleepy_man(self, ), lists) # ONLY the second one is processed
        pool.close()
        pool.join()
        rospy.loginfo('The answer is %d', self.num) # answer is 1

        # # version 1 # multiprocessing.Process Files are run on CPU
        # move_gripper_l = threading.Thread(target=self.sleepy_man("left"))
        # move_gripper_l.start()

        # move_gripper_r = threading.Thread(target=self.sleepy_man("right"))
        # move_gripper_r.start()

        # move_gripper_l.join()
        # move_gripper_r.join()

        toc = time.time()
        rospy.loginfo('Done in %f seconds', toc-tic)

        #  # 7. return head to center and remove table
        # head_control.run(Point(x=1.0, y=0.0, z=1.1))
        # self._scene.remove_world_object("table")

        self._as.set_aborted()
    
    def sleepy_man(self, side):
        self.num += 1
        rospy.loginfo('%s starting to sleep ', side)
        time.sleep(3)
        rospy.loginfo('%s done sleeping', side)

    def move_gripper(self, side):
        # Go to pre-grasp position
        rospy.loginfo("Nihaoo")
        pre_goal = PoseStamped()
        if side == "left":
            rospy.loginfo("Hahaha") 
            pre_goal.pose = self.res_l.pre_grasp_pose
            pre_goal.pose.position.z += 0.12  # 0.15
            pre_goal.header.frame_id = self.res_l.frame_id
            self._group_l.set_pose_target(pre_goal)
            succeeded = self._group_l.go(wait=True)
        else:
            rospy.loginfo("Hallo")  # 1
            pre_goal.pose = self.res_r.pre_grasp_pose
            pre_goal.pose.position.z += 0.12  # 0.15
            pre_goal.header.frame_id = self.res_r.frame_id
            self._group_r.set_pose_target(pre_goal)
            succeeded = self._group_r.go(wait=True)
        if not succeeded:
            print(side, "gripper failed to get pre-grasp pose")
            self._as.set_succeeded(succeeded)
            return
        rospy.loginfo("%s gripper approaching pre-grasp pose", side)

        # Go to grasp position
        rospy.sleep(0.1)
        goal = PoseStamped() 
        if side == "left":
            goal.pose = self.res_l.grasp_pose
            goal.pose.position.y -= 0.025 
            goal.header.frame_id = self.res_l.frame_id
            self._group_l.set_pose_target(goal)
            succeeded = self._group_l.go(wait=True)
        else:
            goal.pose = self.res_r.grasp_pose
            goal.pose.position.y -= 0.025 
            goal.header.frame_id = self.res_r.frame_id
            self._group_r.set_pose_target(goal)
            succeeded = self._group_r.go(wait=True)
        if not succeeded:
            print(str(side) + "gripper failed to grasp")
            self._as.set_succeeded(succeeded)
            return
        rospy.loginfo("%s gripper grasped", side)

        # close gripper and attach collision object
        if side == "left":
            self.gripper_l.run('close')
            rospy.loginfo("closed left gripper")
            grasping_group_l = "gripper_{}".format("left")
            touch_links_l = self._robot_l.get_link_names(group=grasping_group_l)  # should _robot duplicate? Maybe No?
            eef_link_l = self._group_l.get_end_effector_link()
            self._scene.attach_box(eef_link_l, str(self._aruco_id_l), touch_links=touch_links_l) ##
        else:
            self.gripper_r.run('close')
            rospy.loginfo("closed right gripper")
            grasping_group_r = "gripper_{}".format("right")
            touch_links_r = self._robot_r.get_link_names(group=grasping_group_r)
            eef_link_r = self._group_r.get_end_effector_link()
            self._scene.attach_box(eef_link_r, str(self._aruco_id_r), touch_links=touch_links_r) # should scene duplicate? Maybe no

        #  Move to post-grasp pose
        rospy.sleep(0.1)
        if side == "left":
            self._group_l.set_pose_target(pre_goal)
            succeeded = self._group_l.go(wait=True)
        else:
            self._group_r.set_pose_target(pre_goal)
            succeeded = self._group_r.go(wait=True)

        # return arm to tuck
        client = actionlib.SimpleActionClient("play_motion", PlayMotionAction) # should client duplicate? Maybe no
        client.wait_for_server()
        rospy.loginfo("...connected.")
        rospy.wait_for_message("joint_states", JointState)
        rospy.sleep(1.0) ##
        rospy.loginfo("Tuck %s arm...", side)
        goal = PlayMotionGoal()
        goal.motion_name = 'home_' + str(side)
        goal.skip_planning = False
        client.send_goal(goal)
        client.wait_for_result(rospy.Duration(20.0))
        rospy.loginfo("%s arm tucked.", side) ##

        # remove cube as collision object in Moveit
        # Note: this step uses the collision_object_server
        # req= removeCollObjResponse() # has no attribute object_id
        # if side == "left":
        #     req.object_id = str(self._aruco_id_l)
        #     req.aruco_pose = self._aruco_marker_l.pose.pose
        # else:
        #     req.object_id = str(self._aruco_id_r)
        #     req.aruco_pose = self._aruco_marker_r.pose.pose
        # req.object_type = goal.object_type
        # # TODO: catch service errors
        # self._remove_co(req)
        # rospy.loginfo("Removed collision object.")
        # rospy.sleep(0.5) #

if __name__ == "__main__":
    rospy.init_node("pick_server")
    rospy.loginfo("pick servers started")

    rospy.loginfo("PickActionClass right starts")
    PickActionClass(side="right")
    rospy.loginfo("PickActionClass left starts")
    PickActionClass(side="left") # These two classes are necessary

    rospy.spin()
