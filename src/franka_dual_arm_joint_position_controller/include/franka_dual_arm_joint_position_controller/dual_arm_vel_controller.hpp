#pragma once
#include <controller_interface/controller_interface.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <franka_dual_msgs/srv/dual_joint_angle.hpp>

namespace franka_dual_arm_joint_position_controller {

class DualArmVelocityController : public controller_interface::ControllerInterface {
 public:
  DualArmVelocityController() = default;

  controller_interface::InterfaceConfiguration
  command_interface_configuration() const override;

  controller_interface::InterfaceConfiguration
  state_interface_configuration() const override;

  controller_interface::return_type
  update(const rclcpp::Time&, const rclcpp::Duration&) override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_init() override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State&) override;
 private:
  /* ---------- Callbacks ---------- */
  void topic_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
  void service_callback(
      const std::shared_ptr<franka_dual_msgs::srv::DualJointAngle::Request> req,
      std::shared_ptr<franka_dual_msgs::srv::DualJointAngle::Response> res);

  /* ---------- Helpers ---------- */
  bool validate_size(const std::vector<double>& vec) const;

  /* ---------- Members ---------- */
  static constexpr size_t kNumJoints = 14;
  


  std::array<double, kNumJoints> target_{};
  std::array<double, kNumJoints> last_valid_{};
  std::string left_prefix_{"left_"};
  std::string right_prefix_{"right_"};

  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_;
  rclcpp::Service<franka_dual_msgs::srv::DualJointAngle>::SharedPtr srv_;
};

}  