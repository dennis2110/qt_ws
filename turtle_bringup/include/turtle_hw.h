#ifndef TURTLE_HW_H
#define TURTLE_HW_H

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <turtlesim/Pose.h>
#include <geometry_msgs/Twist.h>


class TurtleRobot : public hardware_interface::RobotHW
{
public:
  TurtleRobot();
  ~TurtleRobot();

  void init(ros::NodeHandle* _node);

  ros::Time getTime() const {return ros::Time::now(); }
  ros::Duration getPeriod() const { return ros::Duration(0.01); }

  void read(ros::Time time, ros::Duration period);
  void write(ros::Time time, ros::Duration period);

  void turtle_cmd_callback(const turtlesim::PoseConstPtr _messages);

private:
  hardware_interface::JointStateInterface m_joint_state_interface;
  //hardware_interface::PositionJointInterface m_joint_position_interfece;
  hardware_interface::EffortJointInterface m_joint_effort_interfece;

//  double cmd[3];
//  double pos[3];
//  double vel[3];
//  double eff[3];

  double wheel_cmd[2];
  double wheel_pos[2];
  double wheel_vel[2];
  double wheel_eff[2];

  double l_wheel_vel;
  double r_wheel_vel;

  ros::Subscriber turtle_cmd_sub;
  ros::Publisher turtle_move_pub;
};

#endif // TURTLE_HW_H
