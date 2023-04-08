from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='frontier_exploration',
            executable='learned_frontier_detector.py',
            name='learned_frontier_detector',
            output='screen',
            parameters=[
                {"weights_path": "/workspace/install/frontier_exploration/lib/python3.8/site-packages/frontier_exploration/weights/yolov5n_frontier_64.pt"}
            ]
        ),

        Node(
            package='frontier_exploration',
            executable='frontier_exploration_node.py',
            name='frontier_exploration_node',
            output='screen')
     ])