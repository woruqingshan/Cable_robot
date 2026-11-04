from glob import glob
from setuptools import find_packages, setup

name = 'jetrover_idc_visualization'
setup(name=name, version='0.1.0', packages=find_packages(exclude=['test']),
      data_files=[('share/ament_index/resource_index/packages', ['resource/'+name]),
                  ('share/'+name, ['package.xml']),
                  ('share/'+name+'/launch', glob('launch/*.launch.py')),
                  ('share/'+name+'/config', glob('config/*'))],
      install_requires=['setuptools'], zip_safe=False,
      maintainer='JetRover Research', maintainer_email='maintainer@example.com',
      description='Verified offline research replay; no robot control interfaces.',
      license='Apache-2.0',
      entry_points={'console_scripts': [
          'replay_node = jetrover_idc_visualization.node:main',
          'replay_panel = jetrover_idc_visualization.panel:main',
          'check_replay = jetrover_idc_visualization.reader:main']})
