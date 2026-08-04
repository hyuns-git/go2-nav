"""자율주행: scan_maker + cmd_vel_bridge + Nav2 전체 스택.

주의: 실행 직후에는 map 프레임이 없어 planner_server 가 무응답 상태입니다.
      초기 위치를 주면 연쇄적으로 active 가 됩니다. docs/03-navigation.md 참조.
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('go2_nav_bridge')
    nav2 = get_package_share_directory('nav2_bringup')
    map_yaml = LaunchConfiguration('map')
    params = LaunchConfiguration('params_file')

    return LaunchDescription([
        DeclareLaunchArgument('map', default_value='/home/unitree/maps/office.yaml'),
        DeclareLaunchArgument('params_file',
                              default_value=os.path.join(pkg, 'config', 'nav2_go2.yaml')),
        Node(package='go2_nav_bridge', executable='scan_maker',
             name='go2_scan_maker', output='screen',
             parameters=[{
                 'accumulate_sec': 0.55,   # 주행 중엔 반응성 위해 단축
                 'max_frames': 8,
                 'min_height': 0.05,
                 'max_height': 0.60,
                 'angle_increment': 0.03491,
                 'range_min': 0.30,
                 'range_max': 12.0,
                 'scan_rate': 10.0,
                 'tf_rate': 50.0,
                 'min_points_per_scan': 150,
             }]),
        Node(package='go2_nav_bridge', executable='cmd_vel_bridge',
             name='cmd_vel_bridge', output='screen',
             parameters=[{'max_vx': 0.36, 'max_vy': 0.0, 'max_vyaw': 0.60,
                          'min_vx': 0.10, 'min_vyaw': 0.18}]),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2, 'launch', 'bringup_launch.py')),
            launch_arguments={'map': map_yaml, 'params_file': params,
                              'use_sim_time': 'false', 'autostart': 'true'}.items()),
    ])
