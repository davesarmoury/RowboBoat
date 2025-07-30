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
    rclcpp::get_logger("controller_manager.resource_manager.hardware_component.system.PaddleWheel"));
  clock_ = std::make_shared<rclcpp::Clock>(rclcpp::Clock());

  rclcpp::NodeOptions options;
  options.arguments({ "--ros-args", "-r", "__node:=PaddleWheelInternal"});
  node_ = rclcpp::Node::make_shared("_", options);

  arm_publisher_ = node_->create_publisher<sensor_msgs::msg::JointState>("joint_ctrl_single", rclcpp::QoS(1));

  hw_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

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

    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(
        get_logger(), "Joint '%s' have %s command interfaces found. '%s' expected.",
        joint.name.c_str(), joint.command_interfaces[0].name.c_str(),
        hardware_interface::HW_IF_POSITION);
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

bool PaddleWheelHardware::piper_ik(float x, float y, float z, std::vector<float> *result){
    result = new std::vector<float>;
    float L1 = 0.28505;
    float L2 = 0.25065;
    float L3 = std::sqrt(x * x + y * y + z * z);

    float j1 = std::atan2(y, x);
    float j2 = 3.14159 - std::acos((L1*L1 + L3*L3 - L2*L2) / (2 * L1 * L3));
    float j3 = -std::acos((L1*L1 + L2*L2 - L3*L3) / (2 * L1 * L2));
    float j4 = 0.0;
    float j5 = 0.0;
    float j6 = 0.0;

    result->push_back(j1);
    result->push_back(j2);
    result->push_back(j3);
    result->push_back(j4);
    result->push_back(j5);
    result->push_back(j6);

    return true;
}

hardware_interface::CallbackReturn PaddleWheelHardware::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // reset values always when configuring hardware
  for (uint i = 0; i < hw_commands_.size(); i++)
  {
    hw_commands_[i] = 0;
    hw_states_[i] = 0;
  }

  for (uint i = 0; i < hw_states_.size(); i++)
  {
    
    hw_commands_[i] = 0;
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
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_[i]));
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

hardware_interface::return_type PaddleWheelHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
  std::stringstream ss;
  ss << "Writing commands:";

  for (uint i = 0; i < hw_commands_.size(); i++)
  {
    ss << std::fixed << std::setprecision(2) << std::endl
       << "\t" << hw_commands_[i] << " for joint '" << info_.joints[i].name << "'";
  }
  
  return hardware_interface::return_type::OK;
}

}  // namespace rowboboat_control

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  rowboboat_control::PaddleWheelHardware, hardware_interface::ActuatorInterface)