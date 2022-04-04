cmake_minimum_required(VERSION 3.0.2)
project(group13)

# add_compile_options(-std=c++11)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  actionlib
  std_msgs
  std_srvs
  diagnostic_msgs
  rosplan_dispatch_msgs
  rosplan_knowledge_msgs
  # rosplan_dependencies
  roslib
  rostest
  moveit_core 
  moveit_ros_planning 
  moveit_ros_planning_interface 
  tiago_custom_msgs
)

find_package(Eigen3 REQUIRED)
catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs moveit_ros_planning  moveit_ros_planning_interface moveit_core trajectory_msgs
#  INCLUDE_DIRS include
#  LIBRARIES retail_store_planning
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
  tiago_custom_msgs
)

include_directories(
include
${catkin_INCLUDE_DIRS}
${EIGEN3_INCLUDE_DIRS}
)

## Declare action interface for MoveBase interface
add_executable(moveBaseInterface src/RPMoveBase.cpp src/RPActionInterface.cpp)
add_dependencies(moveBaseInterface ${catkin_EXPORTED_TARGETS})
target_link_libraries(moveBaseInterface ${catkin_LIBRARIES})

## Declare action interface for Pick interface
add_executable(pickInterface src/RPPick.cpp src/RPActionInterface.cpp)
add_dependencies(pickInterface ${catkin_EXPORTED_TARGETS})
target_link_libraries(pickInterface ${catkin_LIBRARIES})

## Declare action interface for Place interface
add_executable(placeInterface src/RPPlace.cpp src/RPActionInterface.cpp)
add_dependencies(placeInterface ${catkin_EXPORTED_TARGETS})
target_link_libraries(placeInterface ${catkin_LIBRARIES})

## Declare action interface for Place interface
add_executable(discardInterface src/RPDiscard.cpp src/RPActionInterface.cpp)
add_dependencies(discardInterface ${catkin_EXPORTED_TARGETS})
target_link_libraries(discardInterface ${catkin_LIBRARIES})
