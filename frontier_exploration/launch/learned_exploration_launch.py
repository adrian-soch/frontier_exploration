from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='frontier_exploration',
            executable='learned_frontier_detector.py',
            name='learned_frontier_detector',
            output='screen'
        ),

        Node(
            package='frontier_exploration',
            executable='frontier_exploration_node.py',
            name='frontier_exploration_node',
            output='screen')
     ])