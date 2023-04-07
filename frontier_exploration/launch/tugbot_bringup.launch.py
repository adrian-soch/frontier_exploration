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
    DeclareLaunchArgument('use_sim_time', default_value='true',
                          choices=['true', 'false'],
                          description='use_sim_time'),
    DeclareLaunchArgument('robot_name', default_value='tugbot',
                          description='Robot name')
]


def generate_launch_description():

    # Directories
    pkg_ros_ign_gazebo = get_package_share_directory(
        'ros_ign_gazebo')
    pkg_cas726 = get_package_share_directory(
        'cas726')

    # Paths
    ign_gazebo_launch = PathJoinSubstitution(
        [pkg_ros_ign_gazebo, 'launch', 'ign_gazebo.launch.py'])
    tugbot_bridge_launch = PathJoinSubstitution(
        [pkg_cas726, 'launch', 'tugbot_bridge.launch.py'])
    rviz2_config = PathJoinSubstitution(
        [pkg_cas726, 'rviz', 'robot.rviz'])
    
    ign_resource_path = SetEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',
        value=[os.path.join(pkg_cas726, 'worlds/')])

    # Ignition gazebo
    ignition_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([ign_gazebo_launch]),
        launch_arguments=[
            ('ign_args', [
                'tugbot_warehouse_mod.sdf -v 4 -r'])
                #' -v 4 -r "https://fuel.ignitionrobotics.org/1.0/OpenRobotics/worlds/Tugbot in Warehouse"'])
        ]
    )

    # ROS Ign bridge
    tugbot_bridge = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([tugbot_bridge_launch])
    )

    # Rviz2
    
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz2_config],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        output='screen',
        condition=IfCondition(LaunchConfiguration('rviz'))
    )
    
    # lidar static transforms
    front_lidar_stf = Node(
            name='front_lidar_stf',
            package='tf2_ros',
            executable='static_transform_publisher',
            output='screen',
            arguments=[
                '0', '0', '0', '0', '0.0', '0.0',
                'base_link', [LaunchConfiguration('robot_name'), '/scan_front/scan_front']]
        )

    # camera static transforms
    front_cam_stf = Node(
            name='camera_stf',
            package='tf2_ros',
            executable='static_transform_publisher',
            output='screen',
            arguments=[
                '0', '0', '0', '0', '0', '0',
                'base_link',
                [LaunchConfiguration('robot_name'), '/camera_front/depth']
            ]
        )
    
    # Define LaunchDescription variable
    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(ignition_gazebo)
    ld.add_action(tugbot_bridge)
    ld.add_action(rviz2)
    ld.add_action(front_lidar_stf)
    ld.add_action(front_cam_stf)
    return ld
