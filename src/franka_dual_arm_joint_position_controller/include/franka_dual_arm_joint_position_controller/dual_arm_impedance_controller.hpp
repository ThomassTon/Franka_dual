#pragma once
#include <controller_interface/controller_interface.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <franka_dual_msgs/srv/dual_joint_angle.hpp>

namespace franka_dual_arm_joint_position_controller {

class DualArmImpedanceController : public controller_interface::ControllerInterface {
public:
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_init() override;
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;

  controller_interface::return_type update(const rclcpp::Time &, const rclcpp::Duration &) override;

private:
  static constexpr std::size_t kNumJoints{14};

  std::array<double, kNumJoints> target_{};      // Desired joint positions
  std::array<double, kNumJoints> kp_;            // Stiffness gains
  std::array<double, kNumJoints> kd_;            // Damping gains

  std::string left_prefix_{"left_fr3_"};
  std::string right_prefix_{"right_fr3_"};

  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_;

  void topic_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
  bool validate_size(const std::vector<double> &v) const;
};

} 