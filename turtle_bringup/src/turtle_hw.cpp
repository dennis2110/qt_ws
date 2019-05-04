#include "turtle_hw.h"

TurtleRobot::TurtleRobot()
{
  wheel_cmd[0]=0;
  wheel_cmd[1]=0;
  wheel_eff[0]=30;
  wheel_eff[1]=40;
  wheel_vel[0]=50;
  wheel_vel[1]=60;
  wheel_pos[0]=70;
  wheel_pos[1]=80;
  l_wheel_vel = 10;
  r_wheel_vel = -10;

  ROS_INFO("Create JointStateHandle...");
  /********************* Register the Joint to Hardware Resource Manager
  *
  * For DiffWheel Module:
  *
  *      base_left_wheel_joint:       The base to Left Wheel
  *
  *      base_right_wheel_joint:      The base to Right Wheel
  *
  * ******************************************************************/

  /* Registe the Left wheel and Right wheel to hardware resource manager*/
  hardware_interface::JointStateHandle state_handle_left_wheel(
              "base_left_wheel_joint", &wheel_pos[0], &wheel_vel[0], &wheel_eff[0]);
  m_joint_state_interface.registerHandle(state_handle_left_wheel);

  hardware_interface::JointStateHandle state_handle_right_wheel(
              "base_right_wheel_joint", &wheel_pos[1], &wheel_vel[1], &wheel_eff[1]);
  m_joint_state_interface.registerHandle(state_handle_right_wheel);

  registerInterface(&m_joint_state_interface);

  ROS_INFO("Create JointHandle...");

  /********************* Register the Joint Command from Controller
  * For DiffWheel Module:
  *
  *      base_left_wheel_joint:       Joint Effort Interface(Command type: Torque)
  *
  *      base_right_wheel_joint:      Joint Effort Interface(Command type: Torque)
  *
  * ******************************************************************/

  /* registe wheel effort joint command register */
  hardware_interface::JointHandle position_handle_left_wheel(
              m_joint_state_interface.getHandle("base_left_wheel_joint"), &wheel_cmd[0]);
  m_joint_effort_interfece.registerHandle(position_handle_left_wheel);

  hardware_interface::JointHandle position_handle_right_wheel(
              m_joint_state_interface.getHandle("base_right_wheel_joint"), &wheel_cmd[1]);
  m_joint_effort_interfece.registerHandle(position_handle_right_wheel);

  registerInterface(&m_joint_effort_interfece);

}

TurtleRobot::~TurtleRobot() {}

void TurtleRobot::init(ros::NodeHandle *_node){
  turtle_cmd_sub = _node->subscribe("/turtle1/pose", 1000, &TurtleRobot::turtle_cmd_callback, this);
  turtle_move_pub = _node->advertise<geometry_msgs::Twist>("/turtle1/cmd_vel",100);
  num_count_sub = _node->subscribe("/numcount",1000, &TurtleRobot::numcount_callback, this);
  ROS_INFO("init turtle robot");
}

void TurtleRobot::read(ros::Time time, ros::Duration period){
  wheel_vel[0] = l_wheel_vel;
  wheel_vel[1] = r_wheel_vel;
  wheel_pos[0] = turtle_x;
  wheel_pos[1] = turtle_y;
  wheel_eff[0] = turtle_theta;
  ROS_INFO("update L and R wheel to hardward interface");
}

void TurtleRobot::write(ros::Time time, ros::Duration period){
  geometry_msgs::Twist turtle_twist;
  turtle_twist.linear.x = wheel_cmd[0];
  turtle_twist.angular.z= wheel_cmd[1];
  turtle_move_pub.publish(turtle_twist);
  ROS_INFO("write L and R wheel to turtlesim");
}

void TurtleRobot::turtle_cmd_callback(const turtlesim::PoseConstPtr _messages){
  l_wheel_vel = _messages->linear_velocity;
  r_wheel_vel = _messages->angular_velocity;
  turtle_x = _messages->x;
  turtle_y = _messages->y;
  turtle_theta = _messages->theta;
  ROS_INFO("update turtle velocity to L and R wheel");
  ROS_INFO("left_vel = %f",l_wheel_vel);
  ROS_INFO("right_vel= %f",r_wheel_vel);
}

void TurtleRobot::numcount_callback(const std_msgs::Int16ConstPtr _nummsg){
  num_cnt = _nummsg->data;
  ROS_INFO("num_cnt= %d", num_cnt);
}

