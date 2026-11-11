"""Launch only the two mock ROS nodes; start the HTTP server independently."""
import os
import sys
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    config = os.path.join(get_package_share_directory("cable_robot_vla"), "config", "local_mock.yaml")
    defaults = {"config_file": config, "server_url": "http://127.0.0.1:8000", "python_executable": sys.executable,
                "advance_simulation": "false", "allow_unvalidated_simulation": "false",
                "validation_mode": "disabled", "p2_config_path": "", "log_dir": "log/vla",
                "scene_id": "stage1_synthetic", "scene_sha256": "0" * 64,
                "model_id": "synthetic_joint_state_v1"}
    declarations = [DeclareLaunchArgument(name, default_value=value) for name, value in defaults.items()]
    shared = {name: ParameterValue(LaunchConfiguration(name), value_type=str)
              for name in ("validation_mode", "scene_id", "scene_sha256", "model_id")}
    allow = ParameterValue(LaunchConfiguration("allow_unvalidated_simulation"), value_type=bool)
    return LaunchDescription([*declarations,
        Node(package="cable_robot_vla", executable="mock_robot", namespace="/idc_vla/mock",
             prefix=[LaunchConfiguration("python_executable")], output="screen",
             parameters=[LaunchConfiguration("config_file"), shared, {"allow_unvalidated_simulation": allow}]),
        Node(package="cable_robot_vla", executable="vla_pipeline", namespace="/idc_vla/mock",
             prefix=[LaunchConfiguration("python_executable")], output="screen",
             parameters=[LaunchConfiguration("config_file"), {"validation_mode": shared["validation_mode"],
                "server_url": ParameterValue(LaunchConfiguration("server_url"), value_type=str),
                "log_dir": ParameterValue(LaunchConfiguration("log_dir"), value_type=str),
                "p2_config_path": ParameterValue(LaunchConfiguration("p2_config_path"), value_type=str),
                "advance_simulation": ParameterValue(LaunchConfiguration("advance_simulation"), value_type=bool),
                "allow_unvalidated_simulation": allow}])])
