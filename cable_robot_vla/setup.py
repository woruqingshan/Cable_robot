from glob import glob
from setuptools import find_packages, setup

setup(
    name="cable_robot_vla", version="0.1.0", packages=find_packages(),
    data_files=[("share/ament_index/resource_index/packages", ["resource/cable_robot_vla"]),
                ("share/cable_robot_vla", ["package.xml"]),
                ("share/cable_robot_vla/launch", glob("launch/*.launch.py")),
                ("share/cable_robot_vla/config", glob("config/*.yaml") + glob("config/*.rviz"))],
    install_requires=["setuptools", "vla-protocol==0.1.0", "httpx==0.28.1", "Pillow==11.2.1", "PyYAML==6.0.2"],
    zip_safe=True, maintainer="Cable Robot contributors", maintainer_email="siyuen@example.com",
    description="Isolated ROS 2 local VLA integration with mock execution only",
    license="Proprietary",
    entry_points={"console_scripts": [
        "mock_robot = cable_robot_vla.mock_robot_node:main",
        "vla_pipeline = cable_robot_vla.vla_node:main",
        "trajectory_preview = cable_robot_vla.visualization.node:main"]})
