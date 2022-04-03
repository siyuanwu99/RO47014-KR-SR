#include "RPPlace.h"

#include <actionlib/client/simple_action_client.h>
#include <ros/ros.h>
#include <tiago_custom_msgs/PlaceAction.h>

/* The implementation of RPPlace.h */
namespace KCL_rosplan {

/* constructor */
RPPlaceInterface::RPPlaceInterface(ros::NodeHandle &nh) : _nh(nh) {
  // perform setup
}

/* action dispatch callback */
bool RPPlaceInterface::concreteCallback(
    const rosplan_dispatch_msgs::ActionDispatch::ConstPtr &msg) {
  ROS_INFO_STREAM("\033[1;32m[RPPlaceInterface]\033[0m value 00: " << msg->parameters[0].value);
  ROS_INFO_STREAM("\033[1;32m[RPPlaceInterface]\033[0m value 01: " << msg->parameters[1].value);
  ROS_INFO_STREAM("\033[1;32m[RPPlaceInterface]\033[0m object: " << msg->parameters[2].value);
  ROS_INFO_STREAM("\033[1;32m[RPPlaceInterface]\033[0m gripper: " << msg->parameters[3].value);

  // The action implementation goes here.
  typedef actionlib::SimpleActionClient<tiago_custom_msgs::PlaceAction> PlaceClient;
  PlaceClient rac("place_server_right", true);
  PlaceClient lac("place_server_left", true);

  // wait for the action server to come up
  while (!rac.waitForServer(ros::Duration(5.0)) || !lac.waitForServer(ros::Duration(5.0))) {
    ROS_INFO("Waiting for the place action server to come up");
  }

  tiago_custom_msgs::PlaceGoal goal;
  ROS_INFO("Nihao!");

  /* read waypoint and gripper from messages */
  std::string waypoint_name = msg->parameters[1].value;
  std::string gripper_name  = msg->parameters[3].value;

  bool use_aruco = 0;
  if (!use_aruco) {
    // Use KB info on objects if not using aruco markers
    std::map<std::string, double> position, orientation;
    _nh.getParam("/objects/" + msg->parameters[1].value + "/position", position);
    _nh.getParam("/objects/" + msg->parameters[1].value + "/orientation", orientation);
    // Coordinates in the config are given in 'global' frame
    goal.position.header.frame_id = "map";

    goal.position.point.x = position["x"];
    goal.position.point.y = position["y"];
    goal.position.point.z = position["z"];

    ROS_INFO_STREAM("\033[1;32m[RPPlaceInterface]\033[0m goal position x: " << position["x"]);
    ROS_INFO_STREAM("\033[1;32m[RPPlaceInterface]\033[0m goal position y: " << position["y"]);
    ROS_INFO_STREAM("\033[1;32m[RPPlaceInterface]\033[0m goal position z: " << position["z"]);

    goal.aruco_id = 0;

    /*
    goal.pose.orientation.x = orientation["x"];
    goal.pose.orientation.y = orientation["y"];
    goal.pose.orientation.z = orientation["z"];
    goal.pose.orientation.w = orientation["w"];
    */
  } else {
    goal.position.header.frame_id = "map";

    goal.position.point.x = 0;
    goal.position.point.y = 0;
    goal.position.point.z = 0;

    // If you wanted to use aruco_ids to place, you'd have to
    // get the desired id from somewhere (i.e. params, a message)
    // and fill it in here (and change the bool in line 40 above
    goal.aruco_id = 0;
  }

  if (gripper_name == "rightgrip") {
    rac.sendGoal(goal);
    rac.waitForResult();
  } else if (gripper_name == "leftgrip") {
    // goal.position.point.x -= 0.15;
    // goal.position.point.y -= 0.04;
    lac.sendGoal(goal);
    lac.waitForResult();
  } else {
    ROS_INFO("Invalid gripper name");
  }

  if (rac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED ||
      lac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
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
  ros::init(argc, argv, "rosplan_place_action", ros::init_options::AnonymousName);
  ros::NodeHandle nh("~");

  // create PDDL action subscriber
  KCL_rosplan::RPPlaceInterface rpti(nh);

  rpti.runActionInterface();

  return 0;
}
