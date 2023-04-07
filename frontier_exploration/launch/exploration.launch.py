from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='frontier_exploration',
            executable='classical_frontier_detector', # same as cmake
            name='classical_frontier_detector',
            # prefix='valgrind --leak-check=yes ', # Uncomment to run valgrind. Requires debug build
            output='screen',
            parameters=[
                {"region_size_thresh_": 16}, # number of points
                {"robot_width_": 0.5}, # meters
                {"occupancy_map_topic_": "map"} # occupany grid map topic
            ]
        ),

        Node(
            package='frontier_exploration',
            executable='frontier_exploration_node.py',
            name='frontier_exploration_node',
            output='screen')
     ])