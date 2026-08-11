from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('readonly', default_value='true'),
        DeclareLaunchArgument('allow_torque_commands', default_value='false'),
        DeclareLaunchArgument('waypoint_file', default_value='waypoints/taught_waypoints.yaml'),
        DeclareLaunchArgument('jog_step', default_value='10'),
        DeclareLaunchArgument('jog_duration', default_value='0.4'),
        Node(
            package='cable_robot_control',
            executable='teach_trajectory_node',
            name='teach_trajectory',
            output='screen',
            emulate_tty=True,
            parameters=[{
                'readonly': LaunchConfiguration('readonly'),
                'allow_torque_commands': LaunchConfiguration('allow_torque_commands'),
                'waypoint_file': LaunchConfiguration('waypoint_file'),
                'jog_step': LaunchConfiguration('jog_step'),
                'jog_duration': LaunchConfiguration('jog_duration'),
            }],
        ),
    ])
