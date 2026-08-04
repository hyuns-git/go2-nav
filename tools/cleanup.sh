#!/bin/bash
# 모든 관련 노드 종료. 작업 시작 전 항상 실행 권장.
pkill -f slam_toolbox; pkill -f scan_maker; pkill -f cmd_vel_bridge
pkill -f nav2; pkill -f map_server; pkill -f amcl
pkill -f controller_server; pkill -f planner_server
pkill -f bt_navigator; pkill -f recoveries_server; pkill -f waypoint_follower
sleep 3
ros2 daemon stop >/dev/null 2>&1 && ros2 daemon start >/dev/null 2>&1
sleep 2
echo "--- 남은 노드 ---"
ros2 node list
