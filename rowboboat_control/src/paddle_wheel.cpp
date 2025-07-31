#include "rowboboat_control/paddle_wheel.hpp"

namespace rowboboat_control
{
hardware_interface::CallbackReturn PaddleWheelHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (
    hardware_interface::ActuatorInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }
  logger_ = std::make_shared<rclcpp::Logger>(
    rclcpp::get_logger("controller_manager.resource_manager.hardware_component.actuator.PaddleWheel"));
  clock_ = std::make_shared<rclcpp::Clock>(rclcpp::Clock());

  rclcpp::NodeOptions options;
  options.arguments({ "--ros-args", "-r", "__node:=PaddleWheelInternal"});
  node_ = rclcpp::Node::make_shared("_", options);

  arm_topic_ = info_.hardware_parameters["arm_topic"];
  arm_name_ = info_.hardware_parameters["arm_name"];
  motion_radius_ = std::stod(info_.hardware_parameters["motion_radius"]);
  center_z_ = std::stod(info_.hardware_parameters["center_z"]);
  center_x_ = std::stod(info_.hardware_parameters["center_x"]);
  z_clamp_ = std::stod(info_.hardware_parameters["z_clamp"]);
  y_scale_ = std::stod(info_.hardware_parameters["y_scale"]);

  arm_publisher_ = node_->create_publisher<sensor_msgs::msg::JointState>(arm_topic_, rclcpp::QoS(1));
  point_publisher_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>("paddle_point", rclcpp::QoS(1));

  hw_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_states_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  angles_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    // RRBotSystemPositionOnly has exactly one state and command interface on each joint
    if (joint.command_interfaces.size() != 1)
    {
      RCLCPP_FATAL(
        get_logger(), "Joint '%s' has %zu command interfaces found. 1 expected.",
        joint.name.c_str(), joint.command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        get_logger(), "Joint '%s' have %s command interfaces found. '%s' expected.",
        joint.name.c_str(), joint.command_interfaces[0].name.c_str(),
        hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.size() != 1)
    {
      RCLCPP_FATAL(
        get_logger(), "Joint '%s' has %zu state interface. 1 expected.", joint.name.c_str(),
        joint.state_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(
        get_logger(), "Joint '%s' have %s state interface. '%s' expected.", joint.name.c_str(),
        joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn PaddleWheelHardware::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // reset values always when configuring hardware
  for (uint i = 0; i < hw_commands_.size(); i++)
  {
    hw_commands_[i] = 0;
    hw_states_[i] = 0;
    angles_[i] = 0;
  }

  RCLCPP_INFO(get_logger(), "Successfully configured!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
PaddleWheelHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (uint i = 0; i < info_.joints.size(); i++)
  {
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_states_[i]));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
PaddleWheelHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (uint i = 0; i < info_.joints.size(); i++)
  {
    command_interfaces.emplace_back(
      hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_commands_[i]));
  }

  return command_interfaces;
}

hardware_interface::CallbackReturn PaddleWheelHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
  RCLCPP_INFO(get_logger(), "Activating...");

  // command and state should be equal when starting
  for (uint i = 0; i < hw_states_.size(); i++)
  {
    hw_commands_[i] = hw_states_[i];
    angles_[i] = hw_states_[i];
  }

  RCLCPP_INFO(get_logger(), "Successfully activated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn PaddleWheelHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type PaddleWheelHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  for (uint i = 0; i < hw_states_.size(); i++)
  {
    hw_states_[i] = angles_[i];
  }

  return hardware_interface::return_type::OK;
}

void PaddleWheelHardware::paddle_motion(double &z, double z_clamp){
  z = std::max(z_clamp * -1, z);
  z = std::min(z_clamp, z);
  return;
}

bool PaddleWheelHardware::piper_ik(double x, double y, double z, std::vector<double> &result){
    double L1 = 0.28505;
    double L2 = 0.25065;
    double L3 = std::sqrt(x * x + y * y + z * z);

    double j1 = std::atan2(y, x);
    double j2 = 3.14159 - std::acos((L1*L1 + L3*L3 - L2*L2) / (2 * L1 * L3)) - std::asin(z/L3);
    double j3 = -std::acos((L1*L1 + L2*L2 - L3*L3) / (2 * L1 * L2));
    double j4 = j1;
    double j5 = -j3 - j2;
    double j6 = -j1 * 0.5;

    result.push_back(j1);
    result.push_back(j2);
    result.push_back(j3);
    result.push_back(j4);
    result.push_back(j5);
    result.push_back(j6);

    return true;
}

hardware_interface::return_type PaddleWheelHardware::write(
  const rclcpp::Time &time, const rclcpp::Duration &period)
  {
    double angle = angles_[0] + hw_commands_[0] * period.seconds();;

    angles_[0] = angle;
    hw_states_[0] = angles_[0];
    
    // Calculate circular motion
    double x = center_x_;
    double y = y_scale_ * std::sin(angle) * motion_radius_ * (-1);
    double z = std::cos(angle) * motion_radius_;

    // Clamp flat motion to bottom of motion
    paddle_motion(z, z_clamp_);

    // Shift motion up/down
    z = z + center_z_;

    geometry_msgs::msg::PoseStamped msg = geometry_msgs::msg::PoseStamped();
    msg.header.stamp = time;
    msg.header.frame_id = arm_name_ + "base_link";
    msg.pose.position.x = x;
    msg.pose.position.y = y;
    msg.pose.position.z = z + 0.123;
    msg.pose.orientation.w = 1;
    this->point_publisher_->publish(msg);

    std::vector<double> jangles;

    if ( piper_ik(x, y, z, jangles) ){
      sensor_msgs::msg::JointState msg = sensor_msgs::msg::JointState();
      msg.header.stamp = time;
      
      std::vector<std::string> joint_names_ = {arm_name_ + "joint1", arm_name_ + "joint2", arm_name_ + "joint3", arm_name_ + "joint4", arm_name_ + "joint5", arm_name_ + "joint6"};
      msg.name = joint_names_;
      
      msg.position = jangles;
      arm_publisher_->publish(msg);

      return hardware_interface::return_type::OK;
    }
    
    return hardware_interface::return_type::ERROR;
  }

}  // namespace rowboboat_control

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  rowboboat_control::PaddleWheelHardware, hardware_interface::ActuatorInterface)