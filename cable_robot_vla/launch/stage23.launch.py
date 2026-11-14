"""Isolated Fortress, bridge, feedback TF, native inspection and optional scenario control."""
import json
import sys
from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, SetEnvironmentVariable, ExecuteProcess, RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessStart, OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def start(context):
    def value(name): return LaunchConfiguration(name).perform(context)
    runtime_path = Path(value("runtime_path")).resolve()
    runtime = json.loads(runtime_path.read_text())
    prefix = "/idc_vla/sim"
    if runtime["namespace"] != prefix or not 1 <= runtime["domain_id"] <= 101:
        raise ValueError("generated private simulation runtime required")
    library = Path(value("plugin_dir")).resolve()
    if not (library / "libstage23_initialize.so").is_file():
        raise ValueError("build the Gazebo-only initialization plugin before launch")
    enabled = value("enable_simulation").lower() == "true"
    sim = ExecuteProcess(cmd=["ign", "gazebo", "-s", "-r", str(runtime_path.parent / "stage23.sdf")], output="screen")
    def owner(event, _):
        (runtime_path.parent / "owner.json").write_text(json.dumps({"simulator_pid": event.pid}))
        return []
    bridge = Node(package="ros_gz_bridge", executable="parameter_bridge", namespace=prefix,
        parameters=[{"config_file": str(runtime_path.parent / "bridge.yaml")}], output="screen")
    rsp = Node(package="robot_state_publisher", executable="robot_state_publisher", namespace=prefix,
        parameters=[{"robot_description": ParameterValue((runtime_path.parent / "source_robot.urdf").read_text(), value_type=str), "use_sim_time": True}],
        remappings=[("joint_states", prefix + "/joint_states"), ("/tf", prefix + "/tf"), ("/tf_static", prefix + "/tf_static"), ("/clock", prefix + "/clock")], output="screen")
    pipeline = Node(package="cable_robot_vla", executable="simulation_pipeline", namespace=prefix,
        prefix=[value("python_executable")], parameters=[{"runtime_path": str(runtime_path), "server_url": value("server_url"),
            "execution_backend": "gazebo" if enabled else "inspection", "enable_simulation": enabled,
            "log_dir": value("log_dir"), "use_sim_time": True}], remappings=[("/clock", prefix + "/clock")], output="screen")
    nodes = [sim, bridge, rsp, pipeline]
    if value("gui").lower() == "true":
        from ament_index_python.packages import get_package_share_directory
        nodes.append(Node(package="rviz2", executable="rviz2", namespace=prefix,
            arguments=["-d", str(Path(get_package_share_directory("cable_robot_vla")) / "config/stage23.rviz")],
            parameters=[{"use_sim_time": True}], remappings=[("/clock", prefix + "/clock"), ("/tf", prefix + "/tf"), ("/tf_static", prefix + "/tf_static")], output="screen"))
    return [SetEnvironmentVariable("ROS_LOCALHOST_ONLY", "1"), SetEnvironmentVariable("ROS_DOMAIN_ID", str(runtime["domain_id"])),
        SetEnvironmentVariable("IGN_PARTITION", runtime["partition"]), SetEnvironmentVariable("IGN_GAZEBO_SYSTEM_PLUGIN_PATH", str(library)),
        RegisterEventHandler(OnProcessStart(target_action=sim, on_start=owner)), *nodes,
        *[RegisterEventHandler(OnProcessExit(target_action=node, on_exit=[EmitEvent(event=Shutdown(reason="owned simulation component exited"))])) for node in nodes]]


def generate_launch_description():
    args = {"runtime_path": None, "plugin_dir": None, "python_executable": sys.executable,
        "server_url": "http://127.0.0.1:18000", "enable_simulation": "false", "gui": "true", "log_dir": None}
    return LaunchDescription([*[DeclareLaunchArgument(name, **({"default_value": default} if default is not None else {}))
        for name, default in args.items()], OpaqueFunction(function=start)])
