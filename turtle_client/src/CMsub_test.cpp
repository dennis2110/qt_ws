#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/JointState.h"
#include "controller_manager_msgs/SwitchController.h"
#include <std_msgs/String.h>

void jointstateCallback(const sensor_msgs::JointState::ConstPtr& JS){
  ROS_INFO("x:[%4.3f] y:[%4.3f]", JS->position.at(0), JS->position.at(1));


}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "CMsub_test");
  ros::NodeHandle nh;

  ros::Subscriber sub = nh.subscribe("/Turtle/joint_states", 1000, jointstateCallback);

  ros::spin();

  return 0;
}
