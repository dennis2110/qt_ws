#ifndef TURTLEPOSECONTROLLER_H
#define TURTLEPOSECONTROLLER_H

#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>

namespace turtle_controllers
{
  class PoseController : public controller_interface::
                Controller<hardware_interface::EffortJointInterface>
  {
    enum class JointType{
      ARM,
      WHEEL
    };
  public:
    PoseController();
    ~PoseController();

    bool init(hardware_interface::EffortJointInterface* robot, ros::NodeHandle &n);
    void update(const ros::Time& time, const ros::Duration& period);

    void starting(const ros::Time& time);
    void stopping(const ros::Time& time);
    bool read_parameter(JointType);

    // current node
    ros::NodeHandle m_node;
    /* registered robot and joints */
    hardware_interface::EffortJointInterface* m_robot;
    hardware_interface::JointHandle m_left_wheel;
    hardware_interface::JointHandle m_right_wheel;
    //for joint state
    hardware_interface::JointStateInterface* m_turtle_state;
    hardware_interface::JointStateHandle m_l_turtle_state;
    hardware_interface::JointStateHandle m_r_turtle_state;

  private:
    hardware_interface::JointHandle joint_;
  };

}

#endif // TURTLEPOSECONTROLLER_H
