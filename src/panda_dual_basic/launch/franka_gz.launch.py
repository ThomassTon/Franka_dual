#  Copyright (c) 2025 Franka Robotics GmbH
#
#  Licensed under the Apache License, Version 2.0 (the "License");
#  you may not use this file except in compliance with the License.
#  You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
#  Unless required by applicable law or agreed to in writing, software
#  distributed under the License is distributed on an "AS IS" BASIS,
#  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#  See the License for the specific language governing permissions and
#  limitations under the License.

############################################################################
# Parameters:
# arm_id: ID of the type of arm used (default: '')
# arm_prefix: Prefix for arm topics (default: '')
# namespace: Namespace for the robot (default: '')
# urdf_file: URDF file path relative to franka_description/robots (default: 'fr3/fr3.urdf.xacro')
# robot_ip: Hostname or IP address of the robot (default: '172.16.0.3')
# load_gripper: Use Franka Gripper as an end-effector (default: 'false')
# use_fake_hardware: Use fake hardware (default: 'false')
# fake_sensor_commands: Fake sensor commands (default: 'false')
# joint_state_rate: Rate for joint state publishing in Hz (default: '30')
#
# The franka.launch.py launch file provides a robust and flexible interface
# for launching core Franka Robotics components, including robot_state_publisher,
# ros2_control_node, joint_state_publisher, joint_state_broadcaster,
# franka_robot_state_broadcaster, and optionally franka_gripper, with support
# for both namespaced and non-namespaced environments.
# Example:
# ros2 launch franka_bringup franka.launch.py arm_id:=fr3 namespace:=NS1 robot_ip:=172.16.0.3

# This is an error prone commandline, you may prefer to write the parameters into a YAML file like:
#   franka_bringup/config/franka.config.yaml
# That is especially useful if you want to use multiple namespaces.
# In that case, it's not possible to specify the parameters on the command line,
# since each parameter would have to be somehow isolated or prefixed by the namespace.
# then later parsed by the launch file.
# See: example.launch.py for more details.
#
# You may wish to experiment with the namespace parameter to see how it affects topic names
# and service names. The default namespace is empty, which means that the
# topics and services are not namespaced. If you set the namespace to 'franka1',
# the topics and services will be namespaced with 'franka1'. For example, the
# joint_state_publisher will publish to '/franka1/joint_states' instead of '/joint_states'.
# and the controller_manager will look for the controllers in the 'franka1' namespace.
# To see the difference you can run the following command:
#   ros2 topic list | grep joint_states
#   ros2 service list | grep controller_manager
# This becomes usefull for example when you require multiple Franka robots doing
# possibly different but related tasks. So, you might have the "PICK" and "PLACE" robots
# in the same workspace, but they are not supposed to interfere with each other.
#
# This script generates a URDF file using the specified xacro file to configure the robot
# description and integrates with controllers.yaml for controller management.
# It is designed to be called (included) by higher-level launch files, such as example.launch.py,
# which will, by default, rely upon franka.config.yaml for robot-specific parameters.
# RViz is not launched by this script but can be included by higher-level launch files
# if use_rviz is enabled. Ensure urdf_file parameter (a xacro file) exists in
# franka_description/robots to avoid runtime errors.
#
# This approach improves upon earlier launch scripts, which often lacked namespace
# support and were less modular, offering a more consistent and maintainable solution.
# While some may prefer the older scripts for their simplicity in specific scenarios,
# franka.launch.py enhances flexibility and scalability for diverse Franka Robotics
# applications.
############################################################################

import os
from ament_index_python.packages import get_package_share_directory
import xacro
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription,RegisterEventHandler, ExecuteProcess, TimerAction
from launch.event_handlers import OnProcessExit

from launch.actions import OpaqueFunction, Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import UnlessCondition, IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

# Generates the "default" nodes (controller_manager, robot_state_publisher, etc.)
# for the Franka robot. This function is called by the main launch file.
# It uses the xacro library to process the URDF file and generate the robot description.


def generate_robot_nodes(context):
    load_gripper_launch_configuration = LaunchConfiguration('load_gripper').perform(context)
    load_gripper = load_gripper_launch_configuration.lower() == 'true'
    # urdf_path = PathJoinSubstitution([
    #     FindPackageShare('panda_dual_basic'), 'config', LaunchConfiguration('urdf_file')
    # ]).perform(context)
    # robot_description = xacro.process_file(
    #     urdf_path,
    #     mappings={
    #         'ros2_control': 'true',
    #         'arm_id': LaunchConfiguration('arm_id').perform(context),
    #         'arm_prefix': LaunchConfiguration('arm_prefix').perform(context),
    #         'robot_ip': LaunchConfiguration('robot_ip').perform(context),
    #         'hand': load_gripper_launch_configuration,
    #         'use_fake_hardware': LaunchConfiguration('use_fake_hardware').perform(context),
    #         'fake_sensor_commands': LaunchConfiguration('fake_sensor_commands').perform(context),
    #     }
    # ).toprettyxml(indent='  ')

    urdf_path = PathJoinSubstitution([
    FindPackageShare('panda_dual_basic'), 'config', LaunchConfiguration('urdf_file')
    ]).perform(context)

    with open(urdf_path, 'r') as urdf_file:
        robot_description = urdf_file.read()

    namespace = LaunchConfiguration('namespace').perform(context)
    controllers_yaml = PathJoinSubstitution([
        FindPackageShare('panda_dual_basic'), 'config', "dual_arm_controllers.yaml"
    ]).perform(context)

    # controllers_yaml = PathJoinSubstitution([
    #     FindPackageShare('panda_dual_basic'), 'config', "controllers.yaml"
    # ]).perform(context)

    joint_state_publisher_sources = ['franka/joint_states', 'franka_gripper/joint_states']
    joint_state_rate = int(LaunchConfiguration('joint_state_rate').perform(context))
    os.environ['GZ_SIM_RESOURCE_PATH'] = os.path.dirname(get_package_share_directory('franka_description'))
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    gazebo_empty_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': 'empty.sdf -r', }.items(),
    )

    # load_joint_state_broadcaster = ExecuteProcess(
    #     cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
    #         # '--controller-manager', '/NS2/controller_manager',
    #             'joint_state_broadcaster',
    #             '--param-file', controllers_yaml],
    #     output='both'
    # )

    load_joint_state_broadcaster = ExecuteProcess(
        cmd=['ros2', 'run', 'controller_manager', 'spawner',
            # '--controller-manager', '/NS2/controller_manager',
                'joint_state_broadcaster',
                '--param-file', controllers_yaml
                ],
        output='both'
    )

    # spawn_broadcaster =ExecuteProcess(
    #             cmd=['ros2', 'run', 'controller_manager', 'spawner',
    #                  'joint_state_broadcaster',
    #                  '--controller-manager-timeout', '120', '--unload-on-kill'],
    #             output='screen'
    #         )
    # load_joint_state_broadcaster = Node(
    #         package="controller_manager",
    #         executable="spawner",
    #         namespace="NS2",
    #         arguments=["joint_state_broadcaster", "-c", "--activate"],
    #     )

    # joint_position_controller = ExecuteProcess(
    #     cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
    #             # '--controller-manager', '/NS2/controller_manager',
    #             'franka_dual_arm_joint_position_controller'],
    #     output='both'
    # )
    
    joint_position_controller = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'controller_manager', 'spawner',   
            LaunchConfiguration('controller_name').perform(context),     
            # 'franka_dual_arm_joint_impendance_controller',
            # '--controller-manager', '/NS2/controller_manager',
            '--param-file', controllers_yaml,                  
            # '--set-state', 'active'                           
        ],
        output='both'
    )

    # joint_velocity_example_controller =  Node(
    #         package="controller_manager",
    #         executable="spawner",
    #         namespace="NS2",
    #         arguments=["joint_velocity_example_controller", "-c", "--activate"],
    #     )

    spawn_cm = Node(
            package='controller_manager',
            executable='ros2_control_node',
            namespace=namespace,
            parameters=[
                controllers_yaml],
            # remappings=[('joint_states', joint_state_publisher_sources[0])],
            output='screen',
            # on_exit=Shutdown(),
        )
    
    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        namespace=namespace,
        arguments=['-topic', '/robot_description'],
        output='both',
    )

    nodes = [
        # Node(
        #     package='controller_manager',
        #     executable='ros2_control_node',
        #     namespace=namespace,
        #     name="controller_manager",
        #     arguments=[ '--controller-manager-timeout', '30'],
        #     parameters=[
        #         controllers_yaml],
        #     # remappings=[('joint_states', joint_state_publisher_sources[0])],
        #     output='both',
        #     # on_exit=Shutdown(),
        # ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            namespace=namespace,
            parameters=[{'robot_description': robot_description}],
            output='both',
        ),
        
        gazebo_empty_world,
        spawn,
        # spawn_cm,
        # Node(
        #     package='controller_manager',
        #     executable='ros2_control_node',
        #     namespace=namespace,
        #     name="controller_manager",
        #     arguments=[ '--controller-manager-timeout', '30'],
        #     parameters=[
        #         controllers_yaml],
        #     # remappings=[('joint_states', joint_state_publisher_sources[0])],
        #     output='both',
        #     # on_exit=Shutdown(),
        # ),

        # RegisterEventHandler(
        #         event_handler=OnProcessExit(
        #             target_action=spawn,
        #             on_exit=spawn_cm,
        #         )
        # ),

        

        # Node(
        #     package='controller_manager',
        #     executable='spawner',
        #     namespace=namespace,
        #     arguments=['joint_state_broadcaster'],
        #     output='screen',
        # ),
        
        RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=spawn,
                    on_exit=[load_joint_state_broadcaster],
                )
        ),
        
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_broadcaster,
                on_exit=[joint_position_controller],
            )
        ),
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            namespace=namespace,
            parameters=[
                {'source_list': ['joint_states'],
                 'rate': 30}],
        ),
        # Node(
        #     package='joint_state_publisher',
        #     executable='joint_state_publisher',
        #     name='joint_state_publisher',
        #     namespace=namespace,
        #     parameters=[{
        #         'source_list': joint_state_publisher_sources,
        #         'rate': joint_state_rate,
        #         'use_robot_description': True,
        #     }],
        #     output='screen',
        # ),
        
        # Node(
        #     package='controller_manager',
        #     executable='spawner',
        #     namespace=namespace,
        #     arguments=['franka_robot_state_broadcaster'],
        #     parameters=[{'arm_id': LaunchConfiguration('arm_id').perform(context)}],
        #     condition=UnlessCondition(LaunchConfiguration('use_fake_hardware')),
        #     output='screen',
        # ),
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource([PathJoinSubstitution(
        #         [FindPackageShare('franka_gripper'), 'launch', 'gripper.launch.py'])]),
        #     launch_arguments={
        #         'namespace': namespace,
        #         'robot_ip': LaunchConfiguration('robot_ip').perform(context),
        #         'use_fake_hardware': LaunchConfiguration('use_fake_hardware').perform(context),
        #     }.items(),
        #     condition=IfCondition(LaunchConfiguration('load_gripper')),
        # ),
    ]

    return nodes

# The generate_launch_description function is the entry point (like "main")
# We use it to declare the launch arguments and call the generate_robot_nodes function.


def generate_launch_description():
    launch_args = [
        DeclareLaunchArgument('arm_id',
                              default_value='',
                              description='ID of the type of arm used'),
        DeclareLaunchArgument('arm_prefix',
                              default_value='',
                              description='Prefix for arm topics'),
        DeclareLaunchArgument('namespace',
                              default_value='',
                              description='Namespace for the robot'),
        DeclareLaunchArgument('urdf_file',
                              default_value='fr3/fr3.urdf.xacro',
                              description='Path to URDF file'),
        DeclareLaunchArgument('robot_ip',
                              default_value='172.16.0.3',
                              description='Hostname or IP address of the robot'),
        DeclareLaunchArgument('load_gripper',
                              default_value='false',
                              description='Use Franka Gripper as an end-effector'),
        DeclareLaunchArgument('use_fake_hardware',
                              default_value='false',
                              description='Use fake hardware'),
        DeclareLaunchArgument('fake_sensor_commands',
                              default_value='false',
                              description='Fake sensor commands'),
        DeclareLaunchArgument('joint_state_rate',
                              default_value='30',
                              description='Rate for joint state publishing (Hz)'),
    ]

    return LaunchDescription(launch_args + [OpaqueFunction(function=generate_robot_nodes)])
