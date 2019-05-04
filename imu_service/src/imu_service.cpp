#include "ros/ros.h"
#include"imu_service/imu_status.h"
#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/Int64MultiArray.h>
#include <std_msgs/Float32.h>
#include <turtlesim/Pose.h>
using namespace std;
float imu_DATA;
void imu_READ(const std_msgs::Float32::ConstPtr& imu_msg)
{
  imu_DATA = imu_msg->data;
}

bool turn90deg(imu_service::imu_status::Request &req,
        imu_service::imu_status::Response &res)
{
  ros::NodeHandle n;
  ros::Publisher move_pub = n.advertise<std_msgs::Int64MultiArray>("move", 100);
  ros::Rate loop_rate(100);

  if(req.Action_name1==req.Action_name2)
  {
    ros::Subscriber IMU_sub = n.subscribe("/imu/fake", 100, imu_READ);
    for(int j=0;j<50;j++)
    {
      res.ActionDone=0;
      res.ActionDone=imu_DATA;
      ROS_INFO("SUM of DATA:%d",res.ActionDone);
      int pose_pub[2]={0,0};
      pose_pub[0]=res.ActionDone;
      pose_pub[1]=res.ActionDone/2;
      std_msgs::Int64MultiArray msgmove;
      msgmove.data.push_back(pose_pub[0]);
      msgmove.data.push_back(pose_pub[1]);
      move_pub.publish(msgmove);
      ros::spinOnce();
      loop_rate.sleep();
    }
  }
  return true;
}
///////////////////
void turtleTheta_READ(const turtlesim::Pose::ConstPtr& imu_msg)
{
  imu_DATA = imu_msg->theta;
}
bool turtleturn90deg(imu_service::imu_status::Request &req,
        imu_service::imu_status::Response &res)
{
  ros::NodeHandle n;
  ros::Publisher move_pub = n.advertise<std_msgs::Int64MultiArray>("move", 1000);
  ros::Rate loop_rate(100);
  if(req.Action_name1==req.Action_name2)
  {
    ros::Subscriber turtlepose_sub = n.subscribe("/turtle1/pose", 1000, turtleTheta_READ);
    float startTheta = imu_DATA;
    float turn90 = M_PI_2f32;
    float goalTheta = atan2(sin(turn90 +startTheta),cos(turn90+startTheta));
    ROS_INFO("startTheta:%4.3f\tgoalTheta:%4.3f",startTheta,goalTheta);
    do
    {
      res.ActionDone=0;

      int pose_pub[2]={10,-10};
      std_msgs::Int64MultiArray msgmove;
      msgmove.data.push_back(pose_pub[0]);
      msgmove.data.push_back(pose_pub[1]);
      move_pub.publish(msgmove);
      ROS_INFO("theta:%4.3f\tabs:%4.3f",imu_DATA,abs(goalTheta-imu_DATA));
      ros::spinOnce();
      loop_rate.sleep();
    }while(abs(goalTheta-imu_DATA)>=0.03);
    int pose_pub[2]={0,0};
    std_msgs::Int64MultiArray msgmove;
    msgmove.data.push_back(pose_pub[0]);
    msgmove.data.push_back(pose_pub[1]);
    move_pub.publish(msgmove);
    ros::spinOnce();
    loop_rate.sleep();
  }
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "imu_service");
  ros::NodeHandle n;
  ros::Rate loop_rate(100);
  ros::Publisher move_pub = n.advertise<std_msgs::Int64MultiArray>("move", 1000);
  ros::Subscriber IMU_sub = n.subscribe("/imu/fake", 100, imu_READ);
  ros::ServiceServer service = n.advertiseService("/turn90deg", turn90deg);
  //turtlesim
  ros::Subscriber turtlepose_sub = n.subscribe("/turtle1/pose", 1000, turtleTheta_READ);
  ros::ServiceServer turtleservice = n.advertiseService("/turtle/turn90deg", turtleturn90deg);
  ROS_INFO("IMU service ready.");
  ros::spin();

  return 0;
}
