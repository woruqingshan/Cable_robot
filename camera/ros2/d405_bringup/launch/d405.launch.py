from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    return LaunchDescription([IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(get_package_share_directory("realsense2_camera"),"launch","rs_launch.py")),launch_arguments={"camera_namespace":"d405","camera_name":"wrist","enable_depth":"true","enable_color":"true","align_depth.enable":"false","pointcloud.enable":"false","depth_module.depth_profile":"848x480x30","rgb_camera.color_profile":"848x480x30"}.items())])
