#include <ros/ros.h>
#include <turtlesim/Pose.h>
#include <geometry_msgs/Twist.h>

float nowX,nowY,nowTheta;

//void moveGoal(turtlesim::Pose goal_pose, double distance_tolerance);
void checkRad(double& rad);

void chatterCallback(const turtlesim::Pose::ConstPtr& turtlePose)
{
  nowX = turtlePose->x;
  nowY = turtlePose->y;
  nowTheta = turtlePose->theta;
  //std::cout << nowX << std::endl;
  //ROS_INFO("Sub Loop  x=[%f]",nowTheta);
  ros::spinOnce();
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "demo_node");
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe("/turtle1/pose", 1000, chatterCallback);
  ros::Publisher turtle_pub = nh.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 1000);

  ros::Rate loop_rate(50);
  //turtlesim::Pose turtlePose;
  float goalX = 5.0;
  float goalY = 5.0;
  float goalTheta = 0.0;


  double rho,theta,alpha,beta,phi;
  geometry_msgs::Twist turtleMove;
  while (ros::ok())
  {
  //ROS_INFO("Main Loop  x=[%f]",nowTheta);

  rho = sqrt(pow((nowX-goalX),2)+pow((nowY-goalY),2));
  theta = static_cast<double>(atan2(nowY-goalY,nowX-goalX));
  phi = static_cast<double>(nowTheta);
  alpha = theta + 3.1415926 -phi;
  beta = -alpha -phi;
  //checkRad(theta);
  checkRad(alpha);
  checkRad(beta);
  std::cout << rho <<"\t" << theta <<"\t" << phi<<"\t" <<alpha<<"\t" <<beta<<"\t" <<nowX<<"\t" <<nowY<<"\t" <<0.5 * rho<<"\t" <<2*alpha-0.2*beta<<std::endl;
  turtleMove.linear.x = 0.5 * rho; //0.5
  turtleMove.linear.y = 0;
  turtleMove.linear.z = 0;
  turtleMove.angular.x = 0;
  turtleMove.angular.y = 0;
  turtleMove.angular.z = 2*alpha -0.2 * beta; //2  -0.2

  /*turtleMove.linear.x = 1.5*sqrt(pow((nowX-goalX),2)+pow((nowY-goalY),2));
  turtleMove.linear.y = 0;
  turtleMove.linear.z = 0;
  turtleMove.angular.x = 0;
  turtleMove.angular.y = 0;
  turtleMove.angular.z = 10*(atan2(goalY-nowY,goalX-nowX)- nowTheta);*/


  turtle_pub.publish(turtleMove);
  ros::spinOnce();
  loop_rate.sleep();
  }
}

void checkRad(double& rad){
  if(rad > 3.141592){
    rad = rad - (2*3.141592);
  }
  if(rad < -3.141592){
    rad = (2*3.141592) + rad;
  }
}
