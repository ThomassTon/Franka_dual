from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import  PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription,RegisterEventHandler, ExecuteProcess, TimerAction
from launch.event_handlers import OnProcessExit, OnProcessStart

from launch_ros.actions import Node
import os

def generate_launch_description():
    # spawn dual franka node
    robot_config_file = PathJoinSubstitution([
        FindPackageShare('panda_dual_basic'), 'config', 'franka_dual_gz_task_motion.config.yaml'
    ])

    franka_gz_launch = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([
                        FindPackageShare('panda_dual_basic'), 'launch', 'dual_franka_gz.launch.py'
                    ])
                ),
                launch_arguments={
                    'robot_config_file': robot_config_file,
                }.items(),


            )

    # sine wave task node
    joint_target_task = Node(
        package='dual_arm_motion_planner',       
        executable='tcp_target_task',            
        name='tcp_target_task',
        output='both',
    )

    delayed_joint_target_task = TimerAction(
        period=5.0,  # 秒，根据你环境调
        actions=[ joint_target_task ],
    )
    return LaunchDescription([
        franka_gz_launch,
        delayed_joint_target_task,
    ])