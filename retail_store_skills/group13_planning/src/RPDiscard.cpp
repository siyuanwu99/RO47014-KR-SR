/*
 * @Author: your name
 * @Date: 2022-03-30 23:27:04
 * @LastEditTime: 2022-04-01 00:27:49
 * @LastEditors: your name
 * @Description: 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 * @FilePath: /ro47014_ws/src/retail_store_skills/group13_planning/src/RPDiscard.cpp
 */
#include "RPDiscard.h"

#include <actionlib/client/simple_action_client.h>
#include <ros/ros.h>
#include <tiago_custom_msgs/PlaceAction.h>

/* The implementation of RPDiscard.h */
namespace KCL_rosplan {

/* constructor */
RPDiscardInterface::RPDiscardInterface(ros::NodeHandle &nh) : _nh(nh) {
  // perform setup
}

/* action dispatch callback */
bool RPDiscardInterface::concreteCallback(
    const rosplan_dispatch_msgs::ActionDispatch::ConstPtr &msg) {
  ROS_INFO_STREAM("\033[1;32m[RPDiscardInterface]\033[0m value 00: " << msg->parameters[0].value);
  ROS_INFO_STREAM("\033[1;32m[RPDiscardInterface]\033[0m value 01: " << msg->parameters[1].value);
  ROS_INFO_STREAM("\033[1;32m[RPDiscardInterface]\033[0m object: " << msg->parameters[2].value);
  ROS_INFO_STREAM("\033[1;32m[RPDiscardInterface]\033[0m gripper: " << msg->parameters[3].value);

  // The action implementation goes here.
  typedef actionlib::SimpleActionClient<tiago_custom_msgs::PlaceAction> PlaceClient;
  PlaceClient rac("place_server_right", true);
  PlaceClient lac("place_server_left", true);

  // wait for the action server to come up
  while (!rac.waitForServer(ros::Duration(5.0)) && !lac.waitForServer(ros::Duration(5.0))) {
    ROS_INFO("Waiting for the place action server to come up");
  }

  tiago_custom_msgs::PlaceGoal goal;
  ROS_INFO("Discarding empty bottles!");

  // Use KB info on objects if not using aruco markers
  std::string waypoint_name = msg->parameters[1].value;
  std::string gripper_name  = msg->parameters[3].value;
  // ROS_INFO_STREAM("WAYPOINT: " >> waypoint_name);
  // ROS_INFO_STREAM("GRIPPER: " >> gripper_name);

  std::map<std::string, double> position, orientation;
  _nh.getParam("/objects/" + waypoint_name + "/position", position);
  _nh.getParam("/objects/" + waypoint_name + "/orientation", orientation);

  // Coordinates in the config are given in 'global' frame
  goal.position.header.frame_id = "map";
  goal.position.point.x = position["x"];
  goal.position.point.y = position["y"];
  goal.position.point.z = position["z"];

  ROS_INFO_STREAM("\033[1;32m[RPDiscardInterface]\033[0m goal position x: " << position["x"]);
  ROS_INFO_STREAM("\033[1;32m[RPDiscardInterface]\033[0m goal position y: " << position["y"]);
  ROS_INFO_STREAM("\033[1;32m[RPDiscardInterface]\033[0m goal position z: " << position["z"]);

  goal.aruco_id = 0;

  if (gripper_name == "rightgrip") { 
    rac.sendGoal(goal);
    rac.waitForResult();
  } else if (gripper_name == "leftgrip") {
    // goal.position.point.x -= 0.14;
    // goal.position.point.y -= 0.04;
    lac.sendGoal(goal);
    lac.waitForResult();
  } else {
    ROS_INFO("Invalid gripper name");
  }

  if (rac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED || lac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("Hooray, the object is placed!");
  else
    ROS_INFO("TIAGo failed to place the object for some reason...");

  // complete the action
  ROS_INFO("KCL: (%s) Place Action completing.", msg->name.c_str());
  return true;
}

}  // namespace KCL_rosplan

/*-------------*/
/* Main method */
/*-------------*/

int main(int argc, char **argv) {
  ros::init(argc, argv, "rosplan_discard_action", ros::init_options::AnonymousName);
  ros::NodeHandle nh("~");

  // create PDDL action subscriber
  KCL_rosplan::RPDiscardInterface rpti(nh);

  rpti.runActionInterface();

  return 0;
}
