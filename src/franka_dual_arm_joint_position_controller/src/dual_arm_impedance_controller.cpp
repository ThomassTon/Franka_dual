#include "franka_dual_arm_joint_position_controller/dual_arm_impedance_controller.hpp"
#include <pluginlib/class_list_macros.hpp>



namespace franka_dual_arm_joint_position_controller {

/* ------- interface lists (14 × effort cmd + 14 × position/velocity state) ------- */
controller_interface::InterfaceConfiguration
DualArmImpedanceController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration cfg;
  cfg.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (int i = 1; i <= 7; ++i) {
    cfg.names.push_back(left_prefix_ + "joint" + std::to_string(i) + "/effort");
    cfg.names.push_back(right_prefix_ + "joint" + std::to_string(i) + "/effort");
  }
  return cfg;
}

controller_interface::InterfaceConfiguration
DualArmImpedanceController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration cfg;
  cfg.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (int i = 1; i <= 7; ++i) {
    cfg.names.push_back(left_prefix_ + "joint" + std::to_string(i) + "/position");
    cfg.names.push_back(left_prefix_ + "joint" + std::to_string(i) + "/velocity");
    cfg.names.push_back(right_prefix_ + "joint" + std::to_string(i) + "/position");
    cfg.names.push_back(right_prefix_ + "joint" + std::to_string(i) + "/velocity");
  }
  return cfg;
}

/* ---------------- lifecycle hooks ---------------- */
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
DualArmImpedanceController::on_init() {
  RCLCPP_INFO(get_node()->get_logger(), "DualArmImpedanceController on_init called!");
  left_prefix_ = get_node()->get_parameter("left_prefix").as_string();
  right_prefix_ = get_node()->get_parameter("right_prefix").as_string();
  std::cerr<<"left_prefix_:  "<<left_prefix_;
  // Declare gains as parameters with sane defaults
  kp_.fill(200.0);
  kd_.fill(10.0);

  auto kp_param = get_node()->declare_parameter("stiffness", std::vector<double>(kNumJoints, 200.0));
  auto kd_param = get_node()->declare_parameter("damping",   std::vector<double>(kNumJoints,  10.0));
  if (kp_param.size() == kNumJoints) std::copy(kp_param.begin(), kp_param.end(), kp_.begin());
  if (kd_param.size() == kNumJoints) std::copy(kd_param.begin(), kd_param.end(), kd_.begin());

  return CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
DualArmImpedanceController::on_activate(const rclcpp_lifecycle::State &) {
  sub_ = get_node()->create_subscription<std_msgs::msg::Float64MultiArray>(
      "~/target_positions", 10,
      std::bind(&DualArmImpedanceController::topic_callback, this, std::placeholders::_1));

  // Initialise target to current position to avoid jumps
  for (std::size_t i = 0; i < kNumJoints; ++i) {
    // position interfaces are ordered same as cfg, so index *2 gets position
    target_[i] = state_interfaces_[i * 2].get_value();
  }

  return CallbackReturn::SUCCESS;
}

/* ---------------- runtime update loop ---------------- */
controller_interface::return_type
DualArmImpedanceController::update(const rclcpp::Time &, const rclcpp::Duration &period) {
  (void)period; // Not used directly, but available for advanced features.
  for (std::size_t i = 0; i < kNumJoints; ++i) {
    double q  = state_interfaces_[i * 2].get_value();     // position
    double dq = state_interfaces_[i * 2 + 1].get_value(); // velocity

    double e     = target_[i] - q;
    double edot  = -dq;
    double tau   = kp_[i] * e + kd_[i] * edot;

    command_interfaces_[i].set_value(tau);
  }
  return controller_interface::return_type::OK;
}

/* ---------------- Callbacks ---------------- */
void DualArmImpedanceController::topic_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
  if (!validate_size(msg->data)) {
    RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 2000, "Target size must be 14");
    return;
  }
  std::copy(msg->data.begin(), msg->data.end(), target_.begin());
}

/* ---------------- helper ---------------- */
bool DualArmImpedanceController::validate_size(const std::vector<double> &v) const {
  return v.size() == kNumJoints;
}

} // namespace franka_dual_arm_impedance_controller

/* -------- pluginlib export -------- */
PLUGINLIB_EXPORT_CLASS(franka_dual_arm_joint_position_controller::DualArmImpedanceController,
                       controller_interface::ControllerInterface)
