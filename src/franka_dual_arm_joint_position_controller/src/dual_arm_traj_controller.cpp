#include <algorithm>
#include <franka_dual_arm_joint_position_controller/dual_arm_traj_controller.hpp>
#include <Eigen/Dense>


namespace franka_dual_arm_joint_position_controller {

/* ---------------- interface lists ---------------- */
controller_interface::InterfaceConfiguration
DualArmTrajectoryController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration cfg;
  cfg.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (int i = 1; i <= 7; ++i) {
    cfg.names.push_back(left_prefix_  + "joint" + std::to_string(i) + "/position");
  }
  for (int i = 1; i <= 7; ++i) {
    cfg.names.push_back(right_prefix_ + "joint" + std::to_string(i) + "/position");
  }
  return cfg;
}
controller_interface::InterfaceConfiguration
DualArmTrajectoryController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration cfg;
  cfg.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (int i = 1; i <= 7; ++i) {
    cfg.names.push_back(left_prefix_  + "joint" + std::to_string(i) + "/position");
  }
  for (int i = 1; i <= 7; ++i) {
    cfg.names.push_back(right_prefix_ + "joint" + std::to_string(i) + "/position");
  }

  // for (int i = 1; i <= 7; ++i) {
  //   cfg.names.push_back(left_prefix_  + "joint" + std::to_string(i) + "/velocity");
  //   cfg.names.push_back(right_prefix_ + "joint" + std::to_string(i) + "/velocity");

  // }
  return cfg;
}

/* ---------------- lifecycle ---------------- */
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
DualArmTrajectoryController::on_init() {
  left_prefix_  = get_node()->get_parameter("left_prefix").as_string();
  right_prefix_ = get_node()->get_parameter("right_prefix").as_string();
  for (int i = 1; i <= 7; ++i) {
    joint_names_.push_back(left_prefix_  + "joint" + std::to_string(i));
  }
  for (int i = 1; i <= 7; ++i) {
    joint_names_.push_back(right_prefix_ + "joint" + std::to_string(i));
  }
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
DualArmTrajectoryController::on_activate(const rclcpp_lifecycle::State&) {
  // subscribe to trajectory topic under controller namespace
  srv_ = get_node()->create_service<franka_dual_msgs::srv::DualTraj>(
      "~/set_target",
      std::bind(&DualArmTrajectoryController::service_callback,
                this, std::placeholders::_1, std::placeholders::_2));


  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}


void DualArmTrajectoryController::service_callback(
    const std::shared_ptr<franka_dual_msgs::srv::DualTraj::Request> req,
    std::shared_ptr<franka_dual_msgs::srv::DualTraj::Response> res) {
  if (req->trajectory.points.empty()) {
          res->success = false;
          res->message = "Trajectory empty";
          return;
        }
        traj_buffer_.writeFromNonRT(
            std::make_shared<TrajMsg>(req->trajectory));
        res->success = true;
}

/* ---------------- update ---------------- */
controller_interface::return_type
DualArmTrajectoryController::update(const rclcpp::Time& now, const rclcpp::Duration& period) {
  // pull latest trajectory (if any)
  // auto traj_msg_rt = traj_buffer_.readFromRT();
  static std::shared_ptr<TrajMsg> last_traj;
  static size_t idx = 0;
  static rclcpp::Time start_time;
  Eigen::VectorXd current_q(kNumJoints);
  for (size_t i = 0; i < kNumJoints; ++i) {
    const auto& si = state_interfaces_.at(i);  // index 根据你配置的顺序来
    if (si.get_interface_name() != "position") continue;
    current_q[i] = si.get_value();
  }
  auto traj_msg_rt = traj_buffer_.readFromRT();
  auto traj_msg = *traj_msg_rt;

  if (traj_msg && traj_msg != last_traj) {
    idx = 0;
    start_time = now;
    last_traj = traj_msg;
    RCLCPP_INFO(get_node()->get_logger(), "New trajectory received");
  }
  // auto traj_msg = *traj_msg_rt;
  if (!traj_msg || traj_msg->points.empty()) {
    return controller_interface::return_type::OK;  // nothing to track
  }
  const auto& pts = traj_msg->points;
  // advance idx until time_from_start >= t_now
  const double t = (now - start_time).seconds();
  while (idx + 1 < pts.size() && pts[idx + 1].time_from_start.sec + pts[idx + 1].time_from_start.nanosec * 1e-9 < t) {
    ++idx;
  }
  // while (idx + 1 < pts.size() && pts[idx + 1].time_from_start.sec + pts[idx + 1].time_from_start.nanosec * 1e-9 < t) {
  //   ++idx;
  // }
  // RCLCPP_INFO(get_node()->get_logger(), "pts size %zu position: %.3f", pts.size(), t);
  const auto& q = pts[idx].positions;
  if (q.size() != kNumJoints) return controller_interface::return_type::ERROR;
  for (size_t i = 0; i < kNumJoints; ++i) {
      // double q_ref   = current_q[i];
      // double q_next  = q[i];
      // double vel    = (q_next - q_ref) / 1;
      // command_interfaces_[i].set_value(vel);
      // RCLCPP_INFO(get_node()->get_logger(), "Trajectory index %zu", idx);
    command_interfaces_[i].set_value(q[i]);
  }
  double error=0.0;
  for (size_t i = 0; i < kNumJoints; ++i){
    error += std::pow(current_q[i] - pts.back().positions[i], 2);
  }
  error /=14.0; 

  // if (error< 0.001){
  //   RCLCPP_INFO(get_node()->get_logger(), "Got target position !!!!!");
  // }

  return controller_interface::return_type::OK;
}

} // namespace

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(franka_dual_arm_joint_position_controller::DualArmTrajectoryController,
                       controller_interface::ControllerInterface)

