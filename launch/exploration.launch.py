from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='frontier_exploration',
            executable='exploration_node', # same as cmake
            name='frontier_exploration_node',
            # prefix='valgrind --leak-check=yes ', # Uncomment to run valgrind. Requires debug build
            output='screen',
            parameters=[
                {"region_size_thresh_": 12}, # number of points
                {"robot_width_": 0.5}, # meters
                {"occupancy_map_topic_": "map"} # occupany grid map topic
            ]
        )
     ])