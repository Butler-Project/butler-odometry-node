from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'wheel_radius',
            default_value='0.048',
            description='Wheel radius in meters'
        ),
        DeclareLaunchArgument(
            'wheel_base',
            default_value='0.073',
            description='Distance between wheel centers in meters'
        ),
        DeclareLaunchArgument(
            'publish_tf',
            default_value='true',
            description='Whether to publish odom->base_link TF'
        ),
        Node(
            package='odometry_node',
            executable='odometry_node',
            name='odometry_node',
            output='screen',
            parameters=[{
                'wheel_radius': LaunchConfiguration('wheel_radius'),
                'wheel_base': LaunchConfiguration('wheel_base'),
                'publish_tf': LaunchConfiguration('publish_tf'),
            }]
        ),
    ])
