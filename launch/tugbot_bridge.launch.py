# Copyright 2021 Clearpath Robotics, Inc.
# @author Roni Kreinin (rkreinin@clearpathrobotics.com)

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

ARGUMENTS = [
    DeclareLaunchArgument('use_sim_time', default_value='true',
                          choices=['true', 'false'],
                          description='Use sim time'),
    DeclareLaunchArgument('robot_name', default_value='tugbot',
                          description='Ignition model name'),
    DeclareLaunchArgument('world', default_value='depot',
                          description='World name')
]


def generate_launch_description():
    namespace = LaunchConfiguration('robot_name')
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    # clock bridge
    clock_bridge = Node(package='ros_ign_bridge', executable='parameter_bridge',
                        namespace=namespace,
                        name='clock_bridge',
                        output='screen',
                        arguments=[
                            '/clock' + '@rosgraph_msgs/msg/Clock' + '[ignition.msgs.Clock'
                        ],
                        condition=IfCondition(use_sim_time))

    # cmd_vel bridge
    cmd_vel_bridge = Node(package='ros_ign_bridge', executable='parameter_bridge',
                          name='cmd_vel_bridge',
                          output='screen',
                          parameters=[{
                              'use_sim_time': use_sim_time
                          }],
                          arguments=[
                              ['/model/',LaunchConfiguration('robot_name'),'/cmd_vel' + 
                              '@geometry_msgs/msg/Twist' + 
                              ']ignition.msgs.Twist']
                          ],
                          remappings=[
                              (['/model/', LaunchConfiguration('robot_name'), '/cmd_vel'], '/cmd_vel')
                          ])

    # odom to base_link transform bridge
    odom_base_tf_bridge = Node(package='ros_ign_bridge', executable='parameter_bridge',
                               namespace=namespace,
                               name='odom_base_tf_bridge',
                               output='screen',
                               parameters=[{
                                   'use_sim_time': use_sim_time
                               }],
                               arguments=[
                                   ['/model/', LaunchConfiguration('robot_name'), '/tf' +
                                    '@tf2_msgs/msg/TFMessage' +
                                    '[ignition.msgs.Pose_V']
                               ],
                               remappings=[
                                   (['/model/', LaunchConfiguration('robot_name'), '/tf'], '/tf')
                               ])

    
    # front camera bridge
    front_camera_bridge = Node(
        package='ros_ign_bridge',
        executable='parameter_bridge',
        namespace=namespace,
        name='front_camera_bridge',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[
            ['/world/world_demo/model/', LaunchConfiguration('robot_name'),
                '/link/camera_front/sensor/color/image' +
                '@sensor_msgs/msg/Image' +
                '[ignition.msgs.Image'],
            ['/world/world_demo/model/', LaunchConfiguration('robot_name'),
                '/link/camera_front/sensor/depth/depth_image' +
                '@sensor_msgs/msg/Image' +
                '[ignition.msgs.Image'],
            ['/world/world_demo/model/', LaunchConfiguration('robot_name'),
                '/link/camera_front/sensor/depth/depth_image/points' +
                '@sensor_msgs/msg/PointCloud2' +
                '[ignition.msgs.PointCloudPacked'],
            ['/world/world_demo/model/', LaunchConfiguration('robot_name'),
                '/link/camera_front/sensor/depth/depth_image/camera_info' +
                '@sensor_msgs/msg/CameraInfo' +
                '[ignition.msgs.CameraInfo'],
            ['/world/world_demo/model/', LaunchConfiguration('robot_name'),
                '/link/camera_front/sensor/color/image/camera_info' +
                '@sensor_msgs/msg/CameraInfo' +
                '[ignition.msgs.CameraInfo'],
                ],
        remappings=[
            (['/world/world_demo/model/', LaunchConfiguration('robot_name'),
                '/link/camera_front/sensor/color/image' ],
             '/front/color_image'),
            (['/world/world_demo/model/', LaunchConfiguration('robot_name'),
                '/link/camera_front/sensor/color/image/camera_info' ],
             '/front/color_image/camera_info'),
            (['/world/world_demo/model/', LaunchConfiguration('robot_name'),
                '/link/camera_front/sensor/depth/depth_image' ],
             '/front/depth_image'),
            (['/world/world_demo/model/', LaunchConfiguration('robot_name'),
                '/link/camera_front/sensor/depth/depth_image/camera_info' ],
             '/front/depth_image/camera_info'),
            (['/world/world_demo/model/', LaunchConfiguration('robot_name'),
                '/link/camera_front/sensor/depth/depth_image/points' ],
             '/front/points'),
                ])
    # laser front bridge
    lidar_front_bridge = Node(
        package='ros_ign_bridge',
        executable='parameter_bridge',
        namespace=namespace,
        name='lidar_front_bridge',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time
        }],
        arguments=[
            ['/world/world_demo/model/', LaunchConfiguration('robot_name'),
             '/link/scan_front/sensor/scan_front/scan' +
             '@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan']
        ],
        remappings=[
            (['/world/world_demo/model/', LaunchConfiguration('robot_name'),
              '/link/scan_front/sensor/scan_front/scan'],
             '/front/scan')
        ])

    # Create launch description and add actions
    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(clock_bridge)
    ld.add_action(cmd_vel_bridge)
    ld.add_action(odom_base_tf_bridge)
    ld.add_action(front_camera_bridge)
    ld.add_action(lidar_front_bridge)
    return ld
