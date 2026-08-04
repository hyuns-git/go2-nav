#!/bin/bash
# Nav2 라이프사이클 상태 확인. timeout 필수 (Foxy 는 무응답 시 무한 대기).
for n in map_server amcl controller_server planner_server recoveries_server bt_navigator; do
  printf "%-22s " $n
  timeout 3 ros2 lifecycle get /$n 2>/dev/null || echo "무응답"
done
