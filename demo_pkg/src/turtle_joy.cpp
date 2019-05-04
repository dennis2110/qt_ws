#include "ros/ros.h"
#include "geometry_msgs/Twist.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "turtle_joy");
  ros::NodeHandle nh;

  ros::Publisher turtle_pub = nh.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 1000);

  ros::Rate loop_rate(10);
  while (ros::ok())
  {
    geometry_msgs::Twist turtleMove;

    turtleMove.linear.x = 2;
    turtleMove.linear.y = 0;
    turtleMove.linear.z = 0;
    turtleMove.angular.x = 0;
    turtleMove.angular.y = 0;
    turtleMove.angular.z = 0;

    turtle_pub.publish(turtleMove);

    ros::spinOnce();

    loop_rate.sleep();
  }

  return 0;
}
