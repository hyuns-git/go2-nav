#!/bin/bash
# 매핑 중 15분마다 자동 저장. 매핑과 별도 터미널에서 실행.
i=1
while true; do
  sleep 900
  f="/home/unitree/maps/auto$(printf %02d $i)"
  ros2 service call /slam_toolbox/serialize_map \
    slam_toolbox/srv/SerializePoseGraph "{filename: '$f'}" >/dev/null 2>&1
  ros2 run nav2_map_server map_saver_cli -f "$f" --occ 0.65 --free 0.25 >/dev/null 2>&1
  sed -i '/^mode:/d' "$f.yaml" 2>/dev/null
  echo "[$(date +%H:%M)] autosave -> $f"
  i=$((i+1))
done
