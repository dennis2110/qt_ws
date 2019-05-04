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
  double nowX = m_left_wheel.getPosition();
  double nowY = m_right_wheel.getPosition();
  double nowTheta = m_left_wheel.getEffort();
  double rho,theta,alpha,beta,phi;
  rho = sqrt(pow((nowX-goalX),2)+pow((nowY-goalY),2));
  theta = static_cast<double>(atan2(nowY-goalY,nowX-goalX));
  phi = static_cast<double>(nowTheta);
  alpha = theta + M_PIf64 -phi;
  beta = -alpha -phi;
  turtle_controllers::PoseController::checkRad(alpha);
  turtle_controllers::PoseController::checkRad(beta);
  std::cout << rho <<"\t" << theta <<"\t" << phi<<"\t" <<alpha<<"\t" <<beta<<"\t" <<nowX<<"\t" <<nowY<<"\t" <<0.5 * rho<<"\t" <<2*alpha-0.2*beta<<std::endl;
  m_left_wheel.setCommand(0.5 * rho);
  m_right_wheel.setCommand(2*alpha - 0.2*beta);

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

void turtle_controllers::PoseController::checkRad(double &rad){
  if(rad > M_PIf64){
    rad = rad - (2*M_PIf64);
  }
  if(rad < -M_PIf64){
    rad = (2*M_PIf64) + rad;
  }
}

PLUGINLIB_EXPORT_CLASS(turtle_controllers::PoseController, controller_interface::ControllerBase)
