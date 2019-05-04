#include "turtleposecontroller.h"

turtle_controllers::PoseController::PoseController(){

}

turtle_controllers::PoseController::~PoseController(){

}


bool turtle_controllers::PoseController::init(hardware_interface::EffortJointInterface *robot, ros::NodeHandle &n){
  // Register node for parent and robot handle
  m_node = n;
  m_robot = robot;
  // read the parameter at parameter server and registe to class
  if(!read_parameter(JointType::WHEEL)) return false;

//  // setup the subscribe for diff command
//  m_sub_diff_command = m_node.subscribe("diff_cmd", 1000, &FuzzyController::command_twist_callback, this);

//  // setup the subscribe for imu data
//  m_pose_imu = m_node.subscribe(imu_topic,100,&FuzzyController::callback_imu, this);

  return true;
}

void turtle_controllers::PoseController::update(const ros::Time &time, const ros::Duration &period){
  double a = m_left_wheel.getVelocity();
  ROS_INFO("get %f", a);
  m_left_wheel.setCommand(10);
  m_right_wheel.setCommand(0);

  return;
}

void turtle_controllers::PoseController::starting(const ros::Time &time){

}

void turtle_controllers::PoseController::stopping(const ros::Time &time){

}

bool turtle_controllers::PoseController::read_parameter(JointType _type){
  if(_type == JointType::WHEEL) {
    XmlRpc::XmlRpcValue joint_names;
    if(!m_node.getParam("wheels",joint_names)) {
        ROS_ERROR("No 'wheel joints' in controller. (namespace: %s)",
                m_node.getNamespace().c_str());
        return false;
    }
    if(joint_names.getType() != XmlRpc::XmlRpcValue::TypeArray) {
        ROS_ERROR("'wheel joints' is not a struct. (namespace: %s)",
                m_node.getNamespace().c_str());
        return false;
    }


    XmlRpc::XmlRpcValue name_value;
    name_value = joint_names[0];
    if(name_value.getType() != XmlRpc::XmlRpcValue::TypeString)
    {
        ROS_ERROR("joints are not strings. (namespace: %s)",
                m_node.getNamespace().c_str());
        return false;
    }

    ROS_INFO("Find \"%s\" wheel joints !", ((std::string)name_value).data() );
    m_left_wheel=m_robot->
            getHandle((std::string)name_value);
    //joint  l state
    //m_l_turtle_state = m_turtle_state->
    //        getHandle((std::string)name_value);
    //end
    name_value = joint_names[1];
    if(name_value.getType() != XmlRpc::XmlRpcValue::TypeString)
    {
        ROS_ERROR("wheel joints are not strings. (namespace: %s)",
                m_node.getNamespace().c_str());
        return false;
    }

    ROS_INFO("Find \"%s\" wheel joints !", ((std::string)name_value).data() );
    m_right_wheel=m_robot->
            getHandle((std::string)name_value);
    //joint  r state
    //m_r_turtle_state = m_turtle_state->
    //        getHandle((std::string)name_value);
    //end
  }
return true;
}


PLUGINLIB_EXPORT_CLASS(turtle_controllers::PoseController, controller_interface::ControllerBase)
