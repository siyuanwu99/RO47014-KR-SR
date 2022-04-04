#include "RPPick.h"

#include <actionlib/client/simple_action_client.h>
#include <ros/ros.h>
#include <tiago_custom_msgs/PickAction.h>

/* The implementation of RPPick.h */
namespace KCL_rosplan {

/* constructor */
RPPickInterface::RPPickInterface(ros::NodeHandle &nh) : _nh(nh) {
  // perform setup
}

/* action dispatch callback */
bool RPPickInterface::concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr &msg) {
  // The action implementation goes here.
  typedef actionlib::SimpleActionClient<tiago_custom_msgs::PickAction> PickClient;
  PickClient rc("pick_server_right", true);
  PickClient lc("pick_server_left", true);

  // wait for the action server to come up
  while (!rc.waitForServer(ros::Duration(5.0))) {
    ROS_INFO("Waiting for the pick action server to come up");
  }

  tiago_custom_msgs::PickGoal goal;

  std::map<std::string, double> aruco;
  _nh.getParam("/objects/" + msg->parameters[2].value + "/aruco", aruco);
  ROS_INFO_STREAM("\033[1;32m[RPPickInterface]\033[0m value 00: " << msg->parameters[0].value);
  ROS_INFO_STREAM("\033[1;32m[RPPickInterface]\033[0m value 01: " << msg->parameters[1].value);
  goal.aruco_id    = aruco["id"];
  ROS_INFO_STREAM("\033[1;32m[RPPickInterface]\033[0m object: " << msg->parameters[2].value);
  ROS_INFO_STREAM("\033[1;32m[RPPickInterface]\033[0m gripper: " << msg->parameters[3].value);
  ROS_INFO_STREAM("\033[1;32m[RPPickInterface]\033[0m aruco id: " << goal.aruco_id);
  goal.object_type = "hagelslag";

  if (msg->parameters[3].value == "rightgrip") {
    ROS_INFO("Sending goal to the right gripper");
    rc.sendGoal(goal);
    rc.waitForResult();
  } else if (msg->parameters[3].value == "leftgrip") {
    ROS_INFO("Sending goal to the left gripper");
    lc.sendGoal(goal);
    lc.waitForResult();
  } else {
    ROS_INFO("Cannot pick object, no gripper specified");
    return false;
  }

  if (rc.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) { 
    ROS_INFO("Hooray, the object is picked up by right gripper!");
  } else if (lc.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
    ROS_INFO("Hooray, the object is picked up by left gripper!");
  } else {
    ROS_INFO("TIAGo failed to pick up the object for some reason...");
    // return false;
  }

  // complete the action
  ROS_INFO("KCL: (%s) Pick Action completing.", msg->name.c_str());
  return true;
}

}  // namespace KCL_rosplan

/*-------------*/
/* Main method */
/*-------------*/

int main(int argc, char **argv) {
  ros::init(argc, argv, "rosplan_pick_action", ros::init_options::AnonymousName);
  ros::NodeHandle nh("~");

  // create PDDL action subscriber
  KCL_rosplan::RPPickInterface rpti(nh);

  rpti.runActionInterface();

  return 0;
}
