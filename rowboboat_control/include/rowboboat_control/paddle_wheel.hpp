#ifndef ROWBOBOAT_PADDLE_WHEEL_HPP_
#define ROWBOBOAT_PADDLE_WHEEL_HPP_

#include <memory>
#include <string>
#include <vector>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <limits>
#include <sstream>

#include "hardware_interface/actuator_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/logger.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

namespace rowboboat_control
{
class PaddleWheelHardware : public hardware_interface::ActuatorInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(PaddleWheelHardware)

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  rclcpp::Logger get_logger() const { return *logger_; }

  rclcpp::Clock::SharedPtr get_clock() const { return clock_; }
  
  bool piper_ik(double x, double y, double z, std::vector<double> &result);
  void paddle_motion(double &z, double motion_floor);

private:
  // Objects for logging
  std::shared_ptr<rclcpp::Logger> logger_;
  rclcpp::Clock::SharedPtr clock_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr arm_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr point_publisher_;
  rclcpp::Node::SharedPtr node_;

  std::string arm_topic_;
  std::string arm_name_;
  double motion_radius_;
  double center_z_;
  double center_x_;
  double z_clamp_;
  double y_scale_;

  std::vector<double> hw_commands_;
  std::vector<double> hw_states_;
  std::vector<double> angles_;
};

}  // namespace rowboboat_control

#endif  // ROWBOBOAT_PADDLE_WHEEL_HPP_