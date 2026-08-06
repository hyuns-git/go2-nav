import os
from glob import glob
from setuptools import setup

package_name = 'go2_nav_bridge'

setup(
name=package_name,
version='1.1.0',
packages=[package_name],
data_files=[
('share/ament_index/resource_index/packages', ['resource/' + package_name]),
('share/' + package_name, ['package.xml']),
(os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
(os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
],
install_requires=['setuptools'],
zip_safe=True,
maintainer='unitree',
maintainer_email='unitree@example.com',
description='Go2 EDU scan maker + cmd_vel bridge + waypoint tools + stair traverse',
license='MIT',
entry_points={
'console_scripts': [
'scan_maker = go2_nav_bridge.scan_maker:main',
'cmd_vel_bridge = go2_nav_bridge.cmd_vel_bridge:main',
'waypoint_tool = go2_nav_bridge.waypoint_tool:main',
'stair_traverse_node = go2_nav_bridge.stair_traverse_node:main',
],
},
)
