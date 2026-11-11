"""Independent P3 preview, official external URDF, private TF and optional RViz."""
import sys
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, SetEnvironmentVariable, RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def start(context):
    from cable_robot_vla.visualization.description import load_description_model
    def value(name): return LaunchConfiguration(name).perform(context)
    domain = int(value("domain_id"))
    if not 1 <= domain <= 101:
        raise ValueError("preview requires a private ROS domain in 1..101")
    root, config = value("workspace_root"), value("model_config")
    model = load_description_model(root, config or None)
    prefix = "/idc_vla/preview"
    rsp = Node(package="robot_state_publisher", executable="robot_state_publisher", namespace=prefix,
        parameters=[{"robot_description": ParameterValue(model.urdf_xml, value_type=str), "use_sim_time": False}],
        remappings=[("joint_states", prefix + "/joint_states"), ("/tf", prefix + "/tf"), ("/tf_static", prefix + "/tf_static")], output="screen")
    preview = Node(package="cable_robot_vla", executable="trajectory_preview", namespace=prefix,
        prefix=[value("python_executable")], parameters=[{"workspace_root": root, "model_config": config,
            "input_path": value("input_path"), "source_action_id": value("source_action_id"),
            "allow_synthetic": value("allow_synthetic").lower() == "true", "loop": value("loop").lower() == "true",
            "replay_hz": float(value("replay_hz"))}], output="screen")
    nodes = [rsp, preview]
    if value("gui").lower() == "true":
        rviz = Node(package="rviz2", executable="rviz2", namespace=prefix,
            arguments=["-d", str(Path(get_package_share_directory("cable_robot_vla")) / "config/trajectory_preview.rviz")],
            remappings=[("/tf", prefix + "/tf"), ("/tf_static", prefix + "/tf_static")], output="screen")
        nodes.append(rviz)
    return [SetEnvironmentVariable("ROS_LOCALHOST_ONLY", "1"), SetEnvironmentVariable("ROS_DOMAIN_ID", str(domain)),
        *nodes, *[RegisterEventHandler(OnProcessExit(target_action=node, on_exit=[EmitEvent(event=Shutdown(reason="owned preview process exited"))])) for node in nodes]]


def generate_launch_description():
    args = {"workspace_root": None, "model_config": "", "input_path": "", "source_action_id": "",
            "allow_synthetic": "false", "loop": "false", "replay_hz": "20.0", "domain_id": "73",
            "gui": "true", "python_executable": sys.executable}
    return LaunchDescription([*[DeclareLaunchArgument(name, **({"default_value": default} if default is not None else {})) for name, default in args.items()],
        OpaqueFunction(function=start)])
