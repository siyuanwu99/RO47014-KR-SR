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
// while ( (!rc.waitForServer(ros::Duration(5.0))) || (!lc.waitForServer(ros::Duration(5.0)) ) ) {
/* action dispatch callback */
bool RPPickInterface::concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr &msg) {
  // The action implementation goes here.
  typedef actionlib::SimpleActionClient<tiago_custom_msgs::PickAction> PickClient;
  PickClient ac("pick_server_right", true);

  // wait for the action server to come up
  while (!ac.waitForServer(ros::Duration(5.0))) {
    ROS_INFO("Waiting for the pick action server to come up!!");
  }

  // show the info of the msg
  ROS_INFO_STREAM("\033[1;32m[RPPickInterface]\033[0m robot:"     << msg->parameters[0].value);
  ROS_INFO_STREAM("\033[1;32m[RPPickInterface]\033[0m waypoint: " << msg->parameters[1].value);
  ROS_INFO_STREAM("\033[1;32m[RPPickInterface]\033[0m gripper1: " << msg->parameters[2].value);
  ROS_INFO_STREAM("\033[1;32m[RPPickInterface]\033[0m object1: "  << msg->parameters[3].value);
  ROS_INFO_STREAM("\033[1;32m[RPPickInterface]\033[0m gripper2: " << msg->parameters[4].value);
  ROS_INFO_STREAM("\033[1;32m[RPPickInterface]\033[0m object2: "  << msg->parameters[5].value);

  // get the goals from the msg
  tiago_custom_msgs::PickGoal goal1;
  tiago_custom_msgs::PickGoal goal2;
  tiago_custom_msgs::PickGoal goal;
  std::map<std::string, double> aruco1;
  std::map<std::string, double> aruco2;
  _nh.getParam("/objects/" + msg->parameters[3].value + "/aruco", aruco1);
  _nh.getParam("/objects/" + msg->parameters[5].value + "/aruco", aruco2);
  goal1.aruco_id    = aruco1["id"];
  goal2.aruco_id    = aruco2["id"];
  goal1.object_type = "hagelslag";
  goal2.object_type = "hagelslag";
  goal.object_type  = "hagelslag";      
  ROS_INFO_STREAM("\033[1;32m[RPPickInterface]\033[0m aruco1 id: " << goal1.aruco_id);
  ROS_INFO_STREAM("\033[1;32m[RPPickInterface]\033[0m aruco2 id: " << goal2.aruco_id);

  //STEP 1:: How to send two goals in PickClient
  // Using std::vector<tiago_custom_msgs::PickGoal> Goals
  //    void SimpleActionClient<ActionSpec>::sendGoal(const Goal & goal,
  // /opt/ros/melodic/include/actionlib/client/simple_action_client.h:317:6:

  if (msg->parameters[2].value == "rightgrip") {
    ROS_INFO("Sending goals now!");
    goal.aruco_id = goal2.aruco_id * 100 + goal1.aruco_id;
    ac.sendGoal(goal);
    ac.waitForResult();
  } else if (msg->parameters[2].value == "leftgrip") {
    // ac.sendGoal(goal1); // sendGoal can only take one parameter tiago_custom_msgs::PickGoal goal1
    // // ac.sendGoal(goal2); // will overlap the first one
    // ac.waitForResult();    // this step is stuck, becuase of not using lc.wait and rc.wait above

    ROS_INFO("Sending goals now!");
    goal.aruco_id = goal1.aruco_id * 100 + goal2.aruco_id;
    ac.sendGoal(goal);
    ac.waitForResult();
  } else {
    ROS_INFO("Cannot pick object, no gripper specified");
    return false;
  }

  // version 1
  if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) { 
    ROS_INFO("Hooray, the objects are picked up by both grippers!");
  } else {
    ROS_INFO("TIAGo failed to pick up the objects for some reason...");   // wrong !!
    return false;
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
  // rosplan_interface_pick
  ros::init(argc, argv, "rosplan_pick_action", ros::init_options::AnonymousName);
  ros::NodeHandle nh("~");

  // create PDDL action subscriber
  KCL_rosplan::RPPickInterface rpti(nh);

  rpti.runActionInterface();

  return 0;
}
