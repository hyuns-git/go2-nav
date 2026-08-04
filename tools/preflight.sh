#!/bin/bash
echo "═══════════════ Go2 PREFLIGHT ═══════════════"
echo "[1] ROS2 환경"
echo "  ROS_DISTRO   = ${ROS_DISTRO:-<없음>}"
echo "  RMW          = ${RMW_IMPLEMENTATION:-<기본>}"
echo "  unitree ament: $(echo $AMENT_PREFIX_PATH | tr ':' '\n' | grep -c unitree) 개"
echo "[2] 필수 토픽"
for t in /utlidar/cloud_deskewed /utlidar/robot_odom /api/sport/request; do
  if ros2 topic list 2>/dev/null | grep -qx "$t"; then echo "  OK   $t"; else echo "  X    $t"; fi
done
echo "[3] 토픽 주파수"
for t in /utlidar/cloud_deskewed /utlidar/robot_odom; do
  printf "  %-30s " "$t"
  timeout 5 ros2 topic hz "$t" 2>/dev/null | grep -m1 average || echo "무응답"
done
echo "[4] 패키지"
for p in slam_toolbox nav2_bringup nav2_map_server tf2_tools go2_nav_bridge; do
  printf "  %-20s " "$p"
  ros2 pkg prefix $p >/dev/null 2>&1 && echo OK || echo "X 없음"
done
echo "[5] 리소스"
echo "  CPU cores: $(nproc)   Load: $(cut -d' ' -f1-3 /proc/loadavg)"
free -h | grep Mem | awk '{print "  RAM: "$3" / "$2}'
echo "═════════════════════════════════════════════"
