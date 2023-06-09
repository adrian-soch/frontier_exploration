import os

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

learned_dir = get_package_share_directory('learned_frontier_detector')

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='learned_frontier_detector',
            executable='frontier_detector',
            name='learned_frontier_detector',
            output='screen'
        ),

        Node(
            package='frontier_exploration',
            executable='frontier_exploration_node.py',
            name='frontier_exploration_node',
            output='screen')
     ])