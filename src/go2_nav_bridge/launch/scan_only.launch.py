"""스캔만 실행 (검증용). SLAM 없이 /scan, /odom, TF 만 발행."""
from launch import LaunchDescription
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
    return LaunchDescription([
        Node(package='go2_nav_bridge', executable='scan_maker',
             name='go2_scan_maker', output='screen', parameters=SCAN_PARAMS),
    ])
