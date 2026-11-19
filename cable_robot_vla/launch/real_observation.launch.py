"""Physical observation only. Does not launch drivers, Gazebo or controllers."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("config", description="Reviewed real observation YAML configuration"),
        Node(package="cable_robot_vla", executable="real_observation", namespace="/idc_vla/real",
             name="real_observation", output="screen", parameters=[LaunchConfiguration("config"),
             {"use_sim_time": False}])])
