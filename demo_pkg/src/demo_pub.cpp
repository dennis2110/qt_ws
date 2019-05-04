#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int16.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "demo_pub");
  ros::NodeHandle nh;

  ros::Publisher chatter_pub = nh.advertise<std_msgs::Int16>("/numcount", 1000);

  int16_t count = 0;
  ros::Rate loop_rate(10);
  while (ros::ok())
  {
    std_msgs::Int16 intmsg;
    intmsg.data = count;
    std_msgs::String msg;
    msg.data = "hello world";

    chatter_pub.publish(intmsg);
    count +=1;
    ros::spinOnce();

    loop_rate.sleep();
  }

  return 0;
}
