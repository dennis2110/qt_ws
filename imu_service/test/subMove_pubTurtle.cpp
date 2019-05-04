#include "ros/ros.h"
#include "std_msgs/Int64MultiArray.h"
#include "geometry_msgs/Twist.h"

void moveCallback(const std_msgs::Int64MultiArray::ConstPtr& msg)
{
  ros::NodeHandle nh;
  ros::Publisher turtle_pub = nh.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel",1000);
  geometry_msgs::Twist turtle_twist;
  turtle_twist.linear.x = (msg->data.at(0) + msg->data.at(1))/2;
  turtle_twist.linear.y = 0.0;
  turtle_twist.linear.z = 0.0;
  turtle_twist.angular.x =0.0;
  turtle_twist.angular.y =0.0;
  turtle_twist.angular.z =(msg->data.at(0) - msg->data.at(1))/20;
  turtle_pub.publish(turtle_twist);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "subMove_pubTurtle");
  ros::NodeHandle nh;
  ros::Publisher turtle_pub = nh.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel",1000);
  ros::Subscriber sub = nh.subscribe("move", 1000, moveCallback);


  ros::spin();
  return 0;
}
