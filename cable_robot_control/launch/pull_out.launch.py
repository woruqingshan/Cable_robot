import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share_dir = get_package_share_directory('cable_robot_control')
    default_trajectory = os.path.join(share_dir, 'config', 'pull_out.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('execute', default_value='false'),
        DeclareLaunchArgument('trajectory_file', default_value=default_trajectory),
        Node(
            package='cable_robot_control',
            executable='fixed_trajectory_node',
            name='fixed_pull_out_trajectory',
            output='screen',
            parameters=[{
                'trajectory_file': LaunchConfiguration('trajectory_file'),
                'execute': LaunchConfiguration('execute'),
            }],
        ),
    ])
