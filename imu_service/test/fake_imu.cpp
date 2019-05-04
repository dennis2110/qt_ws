#include "ros/ros.h"
#include "std_msgs/Float32.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "fake_imu");
  ros::NodeHandle nh;
  ros::Publisher fake_imu_pub = nh.advertise<std_msgs::Float32>("/imu/fake", 100);
  ros::Rate loop_rate(10);

  float i;
  while (ros::ok())
  {
    std_msgs::Float32 fake_yaw;
    fake_yaw.data = i;
    i+=0.1;
    fake_imu_pub.publish(fake_yaw);

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
