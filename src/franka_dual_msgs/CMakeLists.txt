cmake_minimum_required(VERSION 3.10)
project(franka_dual_msgs)

find_package(rosidl_default_generators REQUIRED)
find_package(trajectory_msgs REQUIRED)

find_package(builtin_interfaces REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/DualJointAngle.srv"
  "srv/DualTraj.srv"
  "srv/EEPose.srv"
  DEPENDENCIES trajectory_msgs builtin_interfaces
)

ament_package()
