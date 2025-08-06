#pragma once
#include <array>
#include <deque>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include <franka_dual_msgs/srv/dual_traj.hpp>
#include "realtime_tools/realtime_buffer.hpp"

namespace franka_dual_arm_joint_position_controller {

class DualArmTrajectoryController : public controller_interface::ControllerInterface {
public:
  DualArmTrajectoryController() = default;

  /* === ros2_control mandatory === */
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_init() override;
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_activate(const rclcpp_lifecycle::State&) override;

  controller_interface::return_type update(const rclcpp::Time&, const rclcpp::Duration&) override;

  void service_callback(
      const std::shared_ptr<franka_dual_msgs::srv::DualTraj::Request> req,
      std::shared_ptr<franka_dual_msgs::srv::DualTraj::Response> res);

private:
  static constexpr size_t kNumJoints = 14;

  using TrajMsg = trajectory_msgs::msg::JointTrajectory;

  // incoming trajectory stored realtime‑safe
  realtime_tools::RealtimeBuffer<std::shared_ptr<TrajMsg>> traj_buffer_;

  // prefix parameters
  std::string left_prefix_{"left_fr3_"};
  std::string right_prefix_{"right_fr3_"};
  std::vector<std::string> joint_names_;

  // subscriber
  rclcpp::Subscription<TrajMsg>::SharedPtr traj_sub_;
  rclcpp::Service<franka_dual_msgs::srv::DualTraj>::SharedPtr srv_;
};

} // namespace franka_dual_arm_joint_posi