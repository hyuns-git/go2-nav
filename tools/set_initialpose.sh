#!/bin/bash
# map 프레임 부트스트랩. Nav2 실행 직후 실행하면 나머지 노드가 연쇄 활성화됨.
# 사용: bash set_initialpose.sh [x] [y] [yaw_rad]
X=${1:-0.0}; Y=${2:-0.0}; YAW=${3:-0.0}
QZ=$(python3 -c "import math;print(math.sin($YAW/2))")
QW=$(python3 -c "import math;print(math.cos($YAW/2))")
ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped \
"{header: {frame_id: 'map'}, pose: {pose: {position: {x: $X, y: $Y, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: $QZ, w: $QW}}, covariance: [0.25,0,0,0,0,0, 0,0.25,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0.068]}}"
echo "--- map -> odom 확인 ---"
timeout 5 ros2 run tf2_ros tf2_echo map odom
