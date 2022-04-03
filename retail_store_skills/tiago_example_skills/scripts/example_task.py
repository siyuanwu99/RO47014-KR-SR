#!/usr/bin/env python

import rospy
import time

# For gripper control
from gripper_control import GripperControl
from gazebo_ros_link_attacher.srv import Attach, AttachRequest, AttachResponse

# For arm control
import geometry_msgs.msg
import std_srvs.srv
from plan_arm_ik import PlanArmIk

# For head control
from look_to_point import LookToPoint

# For navigation
from move_base import MoveBase

# For aruco
from aruco_msgs.msg import Marker
from aruco_msgs.msg import MarkerArray

from visualization_msgs.msg import Marker as VisMarker


# aruco_pose = geometry_msgs.msg.Pose()
aruco_pose = None
def aruco_cb(data):
    global aruco_pose
    for marker in data.markers:
        if marker.id == 238:
            aruco_pose = marker.pose.pose
            

rospy.init_node("example_task")
rospy.Subscriber('aruco_marker_publisher/markers', MarkerArray, aruco_cb)
armgoal_marker_pub = rospy.Publisher("armgoal_marker", VisMarker, queue_size=0)

# Skills to test. Test all skills to perform a (hard-coded ;)) task!
test_grippers = True
test_arms = False
test_head = False
test_navigation = False
perform_task = False

# Test opening and closing of both grippers
if test_grippers:
    # gripper_left = GripperControl('left')
    gripper_right = GripperControl('right')

    # start = time.time()
    # gripper_left.run('close')
    # rospy.logwarn('Closing gripper took %s seconds', time.time() - start)

    # start = time.time()
    # gripper_right.run('close')
    # rospy.logwarn('Closing gripper took %s seconds', time.time() - start)
    # rospy.sleep(2)

    start = time.time()
    gripper_right.run('open')
    # rospy.logwarn('Opening gripper took %s seconds', time.time() - start)

    # start = time.time()
    # gripper_left.run('open')
    # rospy.logwarn('Opening gripper took %s seconds', time.time() - start)

# Test arm movement
if test_arms:
    arm_right = PlanArmIk('right', armgoal_marker_pub)

    ## Create goal pose for arm
    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.orientation.x = 0.0
    pose_goal.orientation.y = 0.0
    pose_goal.orientation.z = 0.707
    pose_goal.orientation.w = 0.707
    pose_goal.position.x = 0.59
    pose_goal.position.y = -0.073
    pose_goal.position.z = 0.38

    ## Send goal to arm
    arm_right.run(pose_goal)

# Test head control
if test_head:
    head_control = LookToPoint()

    # Look to a point in space
    point = geometry_msgs.msg.Point()
    point.x = 1.0
    point.y = 0.0
    point.z = 1.0
    head_control.run(point)

    # Look at target frame
    #point = "/arm_right_7_link"
    #head_control.run(point)

# Test navigation
if test_navigation:
    move_base = MoveBase()
    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.position.x = 1.5
    pose_goal.position.y = -3.0

    # Left: {z: 0.707, w: 0.707}
    pose_goal.orientation.z = -0.707
    pose_goal.orientation.w = 0.707

    move_base.run(pose_goal)


if perform_task:
    # Create service proxy to attach and detach objects to grippers
    attach_srv = rospy.ServiceProxy('/link_attacher_node/attach', Attach)
    attach_srv.wait_for_service()
    detach_srv = rospy.ServiceProxy('link_attacher_node/detach', Attach)
    detach_srv.wait_for_service()


    # Move to table
    move_base = MoveBase()
    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.position.x = 1.0
    pose_goal.position.y = 1.1

    # Left: {z: 0.707, w: 0.707}
    pose_goal.orientation.z = 0.707
    pose_goal.orientation.w = 0.707

    move_base.run(pose_goal)

    # Clear octomap to generate a fresh one (ensures proper alignment with table and products)
    rospy.wait_for_service('clear_octomap')
    clear_octomap = rospy.ServiceProxy('clear_octomap', std_srvs.srv.Empty)
    clear_octomap()

    # Look around (To build a full octomap for obstacle avoidance with arm)
    head_control = LookToPoint()

    point = geometry_msgs.msg.Point()
    point.x = 1.0
    point.y = 0.0
    point.z = 1.1
    head_control.run(point)

    point.z = 0.0
    head_control.run(point)

    point.y = -0.7
    head_control.run(point)
    rospy.sleep(2)

    point.y = 0.7
    head_control.run(point)

    point.x = 1.0
    point.y = 0.0
    point.z = 0.6
    head_control.run(point)

    # Wait until aruco marker is found
    while not aruco_pose:
        rospy.loginfo('Aruco marker not found!')
        rospy.sleep(2)

    # Move gripper to product
    arm_right = PlanArmIk('right')

    ## Create goal pose for arm
    arm_goal = aruco_pose
    arm_goal.position.z += 0.10
    arm_goal.position.x -= 0.13 ### gripper tool link is not between the grippers, but in the wrist, about 13 cm back)

    success = arm_right.run(arm_goal)
    # Shift goal a bit to ensure that gripper fits around octomap of product
    if not success:
        arm_goal.position.y += 0.01
        success = arm_right.run(arm_goal)
        if not success:
            arm_goal.position.y -= 0.03
            success = arm_right.run(arm_goal)
            if not success:
                rospy.logerr('Failed to bring arm to object, aborting...')
                quit()

    # Close gripper
    gripper_right = GripperControl('right')
    gripper_right.run('close')

    # Link object to gripper to prevent slipping
    rospy.loginfo("Attaching Product and gripper")
    req = AttachRequest()
    req.model_name_1 = "AH_hagelslag_puur_small"
    req.link_name_1 = "link_0"
    req.model_name_2 = "tiago_dual"
    req.link_name_2 = "arm_right_7_link"

    attached = attach_srv.call(req)
    rospy.loginfo('Object attached: %s', attached)

    # Raise arm up
    arm_goal.position.z = 1.3
    arm_right.run(arm_goal)

    # Look up
    point.x = 1.0
    point.y = 0.0
    point.z = 1.1
    head_control.run(point)


    # Tuck arm
    arm_goal.position.x = 0.1
    arm_goal.position.y = -0.2
    arm_goal.position.z = 0.8
    arm_goal.orientation.x = 0.707
    arm_goal.orientation.y = 0.0
    arm_goal.orientation.z = 0.0
    arm_goal.orientation.w = 0.707

    arm_right.run(arm_goal)

    # Clear octomap again to remove ground so that move_base can navigate again
    rospy.wait_for_service('clear_octomap')
    clear_octomap = rospy.ServiceProxy('clear_octomap', std_srvs.srv.Empty)
    clear_octomap()

    # Move to shelf
    move_base = MoveBase()
    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.position.x = -0.40
    pose_goal.position.y = 0.0

    pose_goal.orientation.z = 1.0
    pose_goal.orientation.w = 0.0

    move_base.run(pose_goal)

    # Look around
    head_control = LookToPoint()

    point = geometry_msgs.msg.Point()
    point.x = 1.0
    point.y = 0.0
    point.z = 1.0
    head_control.run(point)

    point.y = -1.0
    head_control.run(point)
    rospy.sleep(1)

    point.y = 1.0
    head_control.run(point)
    rospy.sleep(1)

    point.x = 1.0
    point.y = 0.0
    point.z = 0.6
    head_control.run(point)

    # Move arm to shelf
    # Reset aruco pose
    aruco_pose = None
    while not aruco_pose:
        rospy.loginfo('Aruco marker not found!')
        rospy.sleep(2)

    arm_goal = geometry_msgs.msg.Pose()

    arm_goal = aruco_pose
    arm_goal.position.z += 0.20
    arm_goal.position.x -= 0.12

    rospy.loginfo(arm_goal)
    ## Send goal to arm
    arm_right.run(arm_goal)
    
    arm_goal.position.z -= 0.10
    arm_right.run(arm_goal)

    # Detach object to gripper to prevent slipping
    rospy.loginfo("Detaching Product and gripper")
    req = AttachRequest()
    req.model_name_1 = "AH_hagelslag_puur_small"
    req.link_name_1 = "link_0"
    req.model_name_2 = "tiago_dual"
    req.link_name_2 = "arm_right_7_link"

    detached = detach_srv.call(req)
    rospy.loginfo(detached)

    # Open gripper 
    gripper_right.run('open')