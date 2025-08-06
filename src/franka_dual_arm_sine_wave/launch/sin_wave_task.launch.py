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
        FindPackageShare('panda_dual_basic'), 'config', 'franka_dual_gz_task_wave.config.yaml'
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
    sine_wave_node = Node(
        package='franka_dual_arm_sine_wave',       
        executable='sine_wave_joint_publisher',            
        name='sine_wave_joint_publisher',
        output='log',
    )

    return LaunchDescription([
        franka_gz_launch,
        sine_wave_node,
    ])