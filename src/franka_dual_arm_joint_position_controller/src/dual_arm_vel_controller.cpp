#include "franka_dual_arm_joint_position_controller/dual_arm_vel_controller.hpp"
#include <pluginlib/class_list_macros.hpp>

namespace franka_dual_arm_joint_position_controller {

/* ------- interface lists (14 × position cmd + 14 × position state) ------- */
controller_interface::InterfaceConfiguration
DualArmVelocityController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration cfg;
  cfg.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (int i = 1; i <= 7; ++i) {
    cfg.names.push_back(left_prefix_  + "joint" + std::to_string(i) + "/velocity");
  }
  for (int i = 1; i <= 7; ++i) {
    cfg.names.push_back(right_prefix_ + "joint" + std::to_string(i) + "/velocity");
  }
  return cfg;
}
controller_interface::InterfaceConfiguration
DualArmVelocityController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration cfg;
  cfg.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (int i = 1; i <= 7; ++i) {
    cfg.names.push_back(left_prefix_  + "joint" + std::to_string(i) + "/velocity");
    cfg.names.push_back(left_prefix_  + "joint" + std::to_string(i) + "/position");
  }
  for (int i = 1; i <= 7; ++i) {
    cfg.names.push_back(right_prefix_ + "joint" + std::to_string(i) + "/velocity");
    cfg.names.push_back(right_prefix_ + "joint" + std::to_string(i) + "/position");
  }
  return cfg;
}

/* ---------------- lifecycle hooks ---------------- */
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
DualArmVelocityController::on_init() {
  RCLCPP_INFO(get_node()->get_logger(), "DualArmVelocityController on_init called!");
  left_prefix_  = get_node()->get_parameter("left_prefix").as_string();
  right_prefix_ = get_node()->get_parameter("right_prefix").as_string();
  // declare configurable prefixes
  // auto_declare<std::string>("left_prefix",  left_prefix_);
  // auto_declare<std::string>("right_prefix", right_prefix_);
  return CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
DualArmVelocityController::on_activate(const rclcpp_lifecycle::State&) {
  // left_prefix_  = get_node()->get_parameter("left_prefix").as_string();
  // right_prefix_ = get_node()->get_parameter("right_prefix").as_string();
  // std::cerr<<"left_prefix_!!!!!!!!!!!!!!!!!! "<<left_prefix_<<std::endl;
  sub_ = get_node()->create_subscription<std_msgs::msg::Float64MultiArray>(
      "target_positions", 10,
      std::bind(&DualArmVelocityController::topic_callback, this, std::placeholders::_1));

  srv_ = get_node()->create_service<franka_dual_msgs::srv::DualJointAngle>(
      "~/set_target",
      std::bind(&DualArmVelocityController::service_callback,
                this, std::placeholders::_1, std::placeholders::_2));

  /* 初始化目标为当前位姿（防止跳变） */
  for (size_t i = 0; i < kNumJoints; ++i)
    target_[i] = 0;
  last_valid_ = target_;
  return CallbackReturn::SUCCESS;
}

/* ---------------- runtime update loop ---------------- */
controller_interface::return_type
DualArmVelocityController::update(const rclcpp::Time&, const rclcpp::Duration&) {
  for (size_t i = 0; i < kNumJoints; ++i)
    command_interfaces_[i].set_value(target_[i]);
  return controller_interface::return_type::OK;
}

/* ---------------- Callbacks ---------------- */
void DualArmVelocityController::topic_callback(
    const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
  if (!validate_size(msg->data)) return;
  target_ = *reinterpret_cast<const std::array<double, kNumJoints>*>(msg->data.data());
  last_valid_ = target_;
}
void DualArmVelocityController::service_callback(
    const std::shared_ptr<franka_dual_msgs::srv::DualJointAngle::Request> req,
    std::shared_ptr<franka_dual_msgs::srv::DualJointAngle::Response> res) {
  if (!validate_size(req->data)) {
    res->success = false;
    res->message = "Size must be 14";
    return;
  }
  // RCLCPP_INFO(get_node()->get_logger(), "Got Target joint angle!!!!!!!");
  target_ = *reinterpret_cast<const std::array<double, kNumJoints>*>(req->data.data());
  last_valid_ = target_;
  res->success = true;
}

/* ---------------- helper ---------------- */
bool DualArmVelocityController::validate_size(const std::vector<double>& v) const {
  return v.size() == kNumJoints;
}

}  // namespace franka_dual_position_controller

/* -------- pluginlib export -------- */
PLUGINLIB_EXPORT_CLASS(franka_dual_arm_joint_position_controller::DualArmVelocityController,
                       controller_interface::ControllerInterface)