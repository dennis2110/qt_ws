#include "ros/ros.h"
#include <cstdlib>
#include "controller_manager_msgs/SwitchController.h"
#include <std_msgs/String.h>
int main(int argc, char **argv)
{
  ros::init(argc, argv, "CMclient_test");
  ros::NodeHandle nh;
  ros::ServiceClient client = nh.serviceClient<controller_manager_msgs::SwitchController>("/Turtle/controller_manager/switch_controller");

  controller_manager_msgs::SwitchController SwitchControl;
  std::string pose_controller("pose_controller");
  std::string joint_state_controller("joint_state_controller");

  SwitchControl.request.strictness = 0;
  SwitchControl.request.start_controllers.push_back(pose_controller);
  SwitchControl.request.start_controllers.push_back(joint_state_controller);

  ROS_INFO("CALL service");
  client.call(SwitchControl);

  return 0;
}
