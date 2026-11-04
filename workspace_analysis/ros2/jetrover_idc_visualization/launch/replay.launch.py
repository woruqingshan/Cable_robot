"""One owned, loopback-only replay session; no vendor/hardware launch inclusion."""
import os
from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, SetEnvironmentVariable, RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def start(context):
    from jetrover_idc_visualization.research import use_workspace
    root = use_workspace(LaunchConfiguration('workspace_root').perform(context))
    from workspace_analysis.robot.model import load_model
    model = load_model(root/'config/workspace.yaml')
    domain = int(LaunchConfiguration('domain_id').perform(context))
    if not 1 <= domain <= 101: raise ValueError('private domain_id must be 1..101')
    share = Path(get_package_share_directory('jetrover_idc_visualization'))
    params = {'workspace_root':str(root),'benchmark_dir':LaunchConfiguration('benchmark_dir').perform(context),
              'input_profile':LaunchConfiguration('input_profile').perform(context),
              'expected_manifest_sha256':LaunchConfiguration('expected_manifest_sha256').perform(context),
              'replay_hz':float(LaunchConfiguration('replay_hz').perform(context))}
    prefix = '/idc_replay'
    rsp = Node(package='robot_state_publisher',executable='robot_state_publisher',name='robot_state_publisher',namespace=prefix,
        parameters=[{'robot_description':ParameterValue(model.urdf_xml,value_type=str),'use_sim_time':False}],
        remappings=[('joint_states',prefix+'/joint_states'),('/tf',prefix+'/tf'),('/tf_static',prefix+'/tf_static')],output='screen')
    replay = Node(package='jetrover_idc_visualization',executable='replay_node',name='replay',namespace=prefix,parameters=[params],output='screen')
    panel = Node(package='jetrover_idc_visualization',executable='replay_panel',name='panel',namespace=prefix,output='screen')
    rviz = Node(package='rviz2',executable='rviz2',name='rviz2',namespace=prefix,
        arguments=['-d',str(share/'config/replay.rviz')],parameters=[{'use_sim_time':False}],
        remappings=[('/tf',prefix+'/tf'),('/tf_static',prefix+'/tf_static')],output='screen')
    actions = [SetEnvironmentVariable('ROS_LOCALHOST_ONLY','1'),SetEnvironmentVariable('ROS_DOMAIN_ID',str(domain)),
               SetEnvironmentVariable('PYTHONDONTWRITEBYTECODE','1'),rsp,replay]
    # Headless transport mode is for automated ROS tests; GUI is the default.
    graphical = LaunchConfiguration('gui').perform(context).lower() == 'true'
    nodes = [rsp,replay]
    if graphical: actions += [panel,rviz]; nodes += [panel,rviz]
    for node in nodes:
        actions.append(RegisterEventHandler(OnProcessExit(target_action=node,
            on_exit=[EmitEvent(event=Shutdown(reason='owned replay process exited'))])))
    return actions


def generate_launch_description():
    share = Path(get_package_share_directory('jetrover_idc_visualization'))
    return LaunchDescription([
        DeclareLaunchArgument('workspace_root',description='Explicit existing workspace_analysis source root'),
        DeclareLaunchArgument('benchmark_dir',description='Read-only completed T5.1 run'),
        DeclareLaunchArgument('input_profile',default_value=str(share/'config/t51_inputs.json')),
        DeclareLaunchArgument('expected_manifest_sha256',default_value=''),
        DeclareLaunchArgument('domain_id',default_value='83'),
        DeclareLaunchArgument('replay_hz',default_value='20.0'),
        DeclareLaunchArgument('gui',default_value='true'),
        OpaqueFunction(function=start)])
