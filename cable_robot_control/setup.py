import os
from glob import glob

from setuptools import setup

package_name = 'cable_robot_control'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml'))),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py'))),
    ],
    install_requires=['setuptools', 'PyYAML'],
    zip_safe=True,
    maintainer='siyuen',
    maintainer_email='siyuen@example.com',
    description='JetRover cable VLA fixed trajectory control overlay.',
    license='Proprietary',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'teach_trajectory_node = cable_robot_control.teach_trajectory_node:main',
            'fixed_trajectory_node = cable_robot_control.fixed_trajectory_node:main',
            'fixed_insert_node = cable_robot_control.fixed_trajectory_node:insert_main',
            'fixed_pull_out_node = cable_robot_control.fixed_trajectory_node:pull_out_main',
        ],
    },
)
