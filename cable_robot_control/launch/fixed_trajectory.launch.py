from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('trajectory_file', default_value=''),
        DeclareLaunchArgument('execute', default_value='false'),
        Node(
            package='cable_robot_control',
            executable='fixed_trajectory_node',
            name='fixed_trajectory',
            output='screen',
            parameters=[{
                'trajectory_file': LaunchConfiguration('trajectory_file'),
                'execute': LaunchConfiguration('execute'),
            }],
        ),
    ])
