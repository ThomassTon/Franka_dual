from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import  PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

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
        executable='joint_target_task',            
        name='joint_target_task',
        output='log',
    )

    return LaunchDescription([
        franka_gz_launch,
        joint_target_task,
    ])