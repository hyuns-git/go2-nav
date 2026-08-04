"""매핑: scan_maker + slam_toolbox.

sync:=true  (기본) 동기 SLAM. 스캔을 버리지 않아 품질 높음.
sync:=false           비동기. CPU 부족 시(/scan hz < 5) 사용.
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

SCAN_PARAMS = [{
    'accumulate_sec': 0.75,
    'max_frames': 10,
    'min_height': 0.05,
    'max_height': 0.60,
    'angle_increment': 0.03491,
    'range_min': 0.30,
    'range_max': 12.0,
    'scan_rate': 10.0,
    'tf_rate': 50.0,
    'min_points_per_scan': 150,
}]


def generate_launch_description():
    pkg = get_package_share_directory('go2_nav_bridge')
    cfg = os.path.join(pkg, 'config', 'slam_precise.yaml')
    sync = LaunchConfiguration('sync')

    return LaunchDescription([
        DeclareLaunchArgument('sync', default_value='true'),
        Node(package='go2_nav_bridge', executable='scan_maker',
             name='go2_scan_maker', output='screen', parameters=SCAN_PARAMS),
        Node(package='slam_toolbox', executable='sync_slam_toolbox_node',
             name='slam_toolbox', output='screen', parameters=[cfg],
             condition=IfCondition(sync)),
        Node(package='slam_toolbox', executable='async_slam_toolbox_node',
             name='slam_toolbox', output='screen', parameters=[cfg],
             condition=UnlessCondition(sync)),
    ])
