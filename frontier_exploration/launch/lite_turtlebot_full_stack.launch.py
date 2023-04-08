# Copyright 2021 Clearpath Robotics, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# @author Roni Kreinin (rkreinin@clearpathrobotics.com)

# NOTE: File contents modified by Todor Stoyanov, please refer to original source.

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchContext, LaunchDescription, SomeSubstitutionsType, Substitution
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition, LaunchConfigurationEquals
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node

PACKAGE_NAME = 'frontier_exploration'

class OffsetParser(Substitution):
    def __init__(
            self,
            number: SomeSubstitutionsType,
            offset: float,
    ) -> None:
        self.__number = number
        self.__offset = offset

    def perform(
            self,
            context: LaunchContext = None,
    ) -> str:
        number = float(self.__number.perform(context))
        return f'{number + self.__offset}'


ARGUMENTS = [
    DeclareLaunchArgument('rviz', default_value='true',
                          choices=['true', 'false'],
                          description='Start rviz.'),
    DeclareLaunchArgument('slam', default_value='sync',
                          choices=['off', 'sync', 'async'],
                          description='Whether to run a SLAM'),
    DeclareLaunchArgument('localization', default_value='false',
                          choices=['true', 'false'],
                          description='Whether to run localization'),
    DeclareLaunchArgument('nav2', default_value='true',
                          choices=['true', 'false'],
                          description='Run nav2'),
    DeclareLaunchArgument('use_sim_time', default_value='true',
                          choices=['true', 'false'],
                          description='use_sim_time'),
    DeclareLaunchArgument('world', default_value='depot',
                          description='Ignition World'),
    DeclareLaunchArgument('model', default_value='lite',
                          choices=['standard', 'lite'],
                          description='Turtlebot4 Model'),
    DeclareLaunchArgument('robot_name', default_value='turtlebot4',
                          description='Robot name')
]


def generate_launch_description():

    # Directories
    pkg_turtlebot4_ignition_bringup = get_package_share_directory(
        'turtlebot4_ignition_bringup')
    pkg_turtlebot4_ignition_gui_plugins = get_package_share_directory(
        'turtlebot4_ignition_gui_plugins')
    pkg_turtlebot4_description = get_package_share_directory(
        'turtlebot4_description')
    pkg_turtlebot4_navigation = get_package_share_directory(
        'turtlebot4_navigation')
    pkg_turtlebot4_viz = get_package_share_directory(
        'turtlebot4_viz')

    pkg_irobot_create_common_bringup = get_package_share_directory(
        'irobot_create_common_bringup')
    pkg_irobot_create_description = get_package_share_directory(
        'irobot_create_description')
    pkg_irobot_create_ignition_bringup = get_package_share_directory(
        'irobot_create_ignition_bringup')
    pkg_irobot_create_ignition_plugins = get_package_share_directory(
        'irobot_create_ignition_plugins')

    pkg_ros_ign_gazebo = get_package_share_directory(
        'ros_ign_gazebo')

    #pkg_share = get_package_share_directory('frontier_exploration')
    #world_file_name = 'cafe.world'
    #world_path = os.path.join(pkg_share, 'worlds', world_file_name)

    # Set ignition resource path
    ign_resource_path = SetEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',
        value=[
            os.path.join(pkg_turtlebot4_ignition_bringup, 'worlds'), ':' +
            os.path.join(pkg_irobot_create_ignition_bringup, 'worlds'), ':' +
            str(Path(pkg_turtlebot4_description).parent.resolve()), ':' +
            str(Path(pkg_irobot_create_description).parent.resolve())])

    ign_gui_plugin_path = SetEnvironmentVariable(
        name='IGN_GUI_PLUGIN_PATH',
        value=[
            os.path.join(pkg_turtlebot4_ignition_gui_plugins, 'lib'), ':' +
            os.path.join(pkg_irobot_create_ignition_plugins, 'lib')])

    # Paths
    ign_gazebo_launch = PathJoinSubstitution(
        [pkg_ros_ign_gazebo, 'launch', 'ign_gazebo.launch.py'])
    turtlebot4_ros_ign_bridge_launch = PathJoinSubstitution(
        [pkg_turtlebot4_ignition_bringup, 'launch', 'ros_ign_bridge.launch.py'])
    rviz_launch = PathJoinSubstitution(
        [pkg_turtlebot4_viz, 'launch', 'view_robot.launch.py'])
    nav_launch = PathJoinSubstitution(
        [pkg_turtlebot4_navigation, 'launch', 'nav_bringup.launch.py'])
    node_launch = PathJoinSubstitution(
        [pkg_turtlebot4_ignition_bringup, 'launch', 'turtlebot4_nodes.launch.py'])
    create3_nodes_launch = PathJoinSubstitution(
        [pkg_irobot_create_common_bringup, 'launch', 'create3_nodes.launch.py'])
    create3_ignition_nodes_launch = PathJoinSubstitution(
        [pkg_irobot_create_ignition_bringup, 'launch', 'create3_ignition_nodes.launch.py'])
    robot_description_launch = PathJoinSubstitution(
        [pkg_turtlebot4_description, 'launch', 'robot_description.launch.py'])
    dock_description_launch = PathJoinSubstitution(
        [pkg_irobot_create_common_bringup, 'launch', 'dock_description.launch.py'])

    # Parameters
    param_file_cmd = DeclareLaunchArgument(
        'param_file',
        default_value=PathJoinSubstitution(
            [pkg_turtlebot4_ignition_bringup, 'config', 'turtlebot4_node.yaml']),
        description='Turtlebot4 Robot param file')

    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        default_value=PathJoinSubstitution(
            [pkg_turtlebot4_navigation, 'maps', 'depot.yaml']),
        description='Full path to map yaml file to load')

    # Launch configurations
    x, y, z = LaunchConfiguration('x'), LaunchConfiguration('y'), LaunchConfiguration('z')
    yaw = LaunchConfiguration('yaw')
    turtlebot4_node_yaml_file = LaunchConfiguration('param_file')

    # Ignition gazebo
    ignition_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([ign_gazebo_launch]),
        launch_arguments=[
            ('ign_args', [
                # LaunchConfiguration('world'), '.sdf',
                ' /workspace/depot2.sdf ',
                ' -v 4 -r ',
                ' --gui-config ', PathJoinSubstitution(
                    [pkg_turtlebot4_ignition_bringup,
                     'gui',
                     LaunchConfiguration('model'),
                     'gui.config'])])
        ]
    )

    # Robot description
    x_rob = OffsetParser(x, 0)
    y_rob = OffsetParser(y, 0)
    robot_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([robot_description_launch]),
        launch_arguments=[('model', LaunchConfiguration('model')),
                          ('use_sim_time', LaunchConfiguration('use_sim_time'))]
    )

    # Dock description
    x_dock = OffsetParser(x, 0.5)
    z_dock = OffsetParser(z, -0.5)
    yaw_dock = OffsetParser(yaw, 3.1416)
    dock_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([dock_description_launch]),
        launch_arguments={'gazebo': 'ignition'}.items()
    )

    # Spawn Turtlebot4
    spawn_robot = Node(
        package='ros_ign_gazebo',
        executable='create',
        arguments=[
            '-name', LaunchConfiguration('robot_name'),
            '-x', x_rob,
            '-y', y_rob,
            '-z', z,
            '-Y', yaw,
            '-topic', 'robot_description'],
        output='screen')

    # Spawn dock
    spawn_dock = Node(
        package='ros_ign_gazebo',
        executable='create',
        arguments=[
            '-name', 'standard_dock',
            '-x', x_dock,
            '-y', y,
            '-z', z_dock,
            '-Y', yaw_dock,
            '-topic', 'standard_dock_description'],
        output='screen')

    # ROS Ign bridge
    turtlebot4_ros_ign_bridge = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([turtlebot4_ros_ign_bridge_launch]),
        launch_arguments=[('model', LaunchConfiguration('model'))]
    )

    # Rviz2
    rviz2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([rviz_launch]),
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    # Navigation
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([nav_launch]),
        launch_arguments=[('slam', LaunchConfiguration('slam')),
                          ('nav2', LaunchConfiguration('nav2')),
                          ('localization', LaunchConfiguration('localization')),
                          ('use_sim_time', LaunchConfiguration('use_sim_time')),
                          ('map', LaunchConfiguration('map'))]
    )

    turtlebot4_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([node_launch]),
        launch_arguments=[('model', LaunchConfiguration('model')),
                          ('param_file', turtlebot4_node_yaml_file)]
    )

    # Create3 nodes
    create3_nodes = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([create3_nodes_launch])
    )

    create3_ignition_nodes = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([create3_ignition_nodes_launch]),
        launch_arguments=[('robot_name', LaunchConfiguration('robot_name'))]
    )

    # RPLIDAR static transforms
    rplidar_stf = Node(
            name='rplidar_stf',
            package='tf2_ros',
            executable='static_transform_publisher',
            output='screen',
            arguments=[
                '0', '0', '0', '0', '0.0', '0.0',
                'rplidar_link', [LaunchConfiguration('robot_name'), '/rplidar_link/rplidar']]
        )

    # OAKD static transforms
    oakd_pro_stf = Node(
            name='camera_stf',
            package='tf2_ros',
            executable='static_transform_publisher',
            output='screen',
            arguments=[
                '0', '0', '0', '0', '0', '0',
                'oakd_pro_rgb_camera_optical_frame',
                [LaunchConfiguration('robot_name'), '/oakd_pro_rgb_camera_frame/rgbd_camera']
            ],
            condition=LaunchConfigurationEquals('model', 'standard')
        )

    oakd_lite_stf = Node(
            name='camera_stf',
            package='tf2_ros',
            executable='static_transform_publisher',
            output='screen',
            arguments=[
                '0', '0', '0', '0', '0', '0',
                'oakd_lite_rgb_camera_optical_frame',
                [LaunchConfiguration('robot_name'), '/oakd_lite_rgb_camera_frame/rgbd_camera']
            ],
            condition=LaunchConfigurationEquals('model', 'lite')
        )

    # Define LaunchDescription variable
    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(ign_resource_path)
    ld.add_action(ign_gui_plugin_path)
    ld.add_action(param_file_cmd)
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(ignition_gazebo)
    ld.add_action(turtlebot4_ros_ign_bridge)
    ld.add_action(rviz2)
    ld.add_action(robot_description)
    ld.add_action(dock_description)
    ld.add_action(spawn_robot)
    ld.add_action(spawn_dock)
    ld.add_action(create3_nodes)
    ld.add_action(create3_ignition_nodes)
    ld.add_action(turtlebot4_node)
    ld.add_action(navigation)
    ld.add_action(rplidar_stf)
    ld.add_action(oakd_pro_stf)
    ld.add_action(oakd_lite_stf)
    return ld
