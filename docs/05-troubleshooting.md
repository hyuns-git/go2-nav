# 트러블슈팅 — 실제로 겪은 문제와 해결

2026-08-04 구축 과정에서 실제로 막혔던 지점들을 순서대로 기록합니다.
2026-08-05 계단 등반 모듈 개발 및 지도 확장 과정에서 겪은 문제(12~15)를 추가했습니다.

---

## Foxy 특유의 제약 (먼저 알아둘 것)

### `ros2 topic echo`에 `--field`, `--once`가 없다

Galactic부터 추가된 옵션입니다. Foxy에서는 이렇게 대체하세요.

| 하려는 것 | Galactic+ | **Foxy** |
|---|---|---|
| 한 번만 보기 | `--once` | `timeout 3 ros2 topic echo /토픽 \| head -40` |
| 특정 필드만 | `--field a.b` | 전체 출력 후 `grep`, 또는 python rclpy |
| 배열 생략 | `--no-arr` | `--no-arr` (Foxy에도 있음) |

`head`로 자를 때 나오는 `BrokenPipeError`는 정상입니다.

### `ros2 lifecycle get`이 무한 대기한다

응답 없는 노드를 만나면 멈춥니다. **항상 `timeout`을 붙이세요.**

```bash
timeout 3 ros2 lifecycle get /planner_server 2>/dev/null || echo "무응답"
```

### rosbag2에 `/clock` 발행이 없다

Galactic부터 지원됩니다. 게다가 apt로 설치되는 slam_toolbox foxy-devel은
`use_sim_time`을 제대로 활용하지 않습니다.
**Foxy에서 rosbag 기반 오프라인 재매핑은 권장하지 않습니다.**
실기 온라인 매핑 + `.posegraph` 이어붙이기로 가세요.

### Keepout Filter가 없다

Galactic부터입니다. 금지구역은 **맵 이미지에 직접 그려야** 합니다
([02-mapping.md](02-mapping.md) 5절).

---

## 문제 1 — `Package 'go2_nav_bridge' not found`

```
searching: ['/home/unitree/cyclonedds_ws/install/rmw_cyclonedds_cpp', '/opt/ros/foxy']
```

검색 경로에 `unitree_ros2`도 `ros2_ws`도 없습니다.

**원인**: 새 터미널에서 환경을 source하지 않음.

**해결**
```bash
source ~/unitree_ros2/setup.sh
source ~/ros2_ws/install/setup.bash
```

**영구 적용** — `.bashrc`는 **새로 여는 터미널**부터 적용됩니다.
지금 열린 터미널에는 위 두 줄을 직접 실행하세요.
```bash
cat >> ~/.bashrc << 'EOF'

# --- Go2 ROS2 ---
source ~/unitree_ros2/setup.sh
source ~/ros2_ws/install/setup.bash
EOF
```

> `source ~/.bashrc`는 쓰지 마세요. ROS setup을 두 번 source하면
> `AMENT_PREFIX_PATH`에 경로가 중복으로 쌓입니다.

**주의**: `unitree_ros2/setup.sh`는 `CYCLONEDDS_URI`로 네트워크
인터페이스를 지정합니다. 이걸 안 한 터미널에서는 로봇 토픽이 아예 안 보입니다.

---

## 문제 2 — 맵이 두 가지 크기로 번갈아 바뀐다

**원인**: slam_toolbox 인스턴스가 2개 실행 중. 각자 다른 맵을 `/map`에 발행.

**확인**
```bash
ros2 node list | grep -E "slam|scan_maker"
ps aux | grep -E "slam_toolbox|scan_maker" | grep -v grep
```

**해결**
```bash
pkill -f slam_toolbox; pkill -f scan_maker; sleep 3
ros2 daemon stop && ros2 daemon start
```

**예방**: 매 작업 시작 전 `pkill`을 습관화하세요.
MobaXterm 탭에 이름을 붙여 어느 탭에서 뭘 돌렸는지 관리하세요.

---

## 문제 3 — RViz에 스캔이 안 보인다

원인이 셋입니다. 순서대로 확인하세요.

| 원인 | 확인 | 해결 |
|---|---|---|
| **노드가 죽음** | `Showing [0] points`인데 messages received는 큼 | 노드가 Ctrl+C로 종료됐는지 확인. 별도 터미널에서 유지 |
| **Size가 너무 작음** | `Size (Pixels): 1` | `5`로. 또는 Style을 `Spheres` + Size(m) 0.05 |
| **Color Transformer** | `Intensity` + `Channel Name: intensity` | **`FlatColor`**로. scan_maker의 LaserScan에는 intensity 필드가 없음 |

---

## 문제 4 — RViz에 맵이 안 보인다

**Map → Durability Policy가 `Volatile`이면 안 보입니다.**
`map_server`와 slam_toolbox는 Transient Local로 발행합니다.

→ **`Transient Local`로 변경**

(렌더링 자체가 깨지는 경우는 문제12 참고 — 이것과는 다른 원인입니다.)

---

## 문제 5 — `map_server` configure 실패

```
ros2 lifecycle set /map_server configure
Transitioning failed
```

**원인**: 맵 yaml의 `mode: trinary` 키. Foxy `map_server`가 인식하지 못합니다.
`map_saver`가 `--mode` 옵션으로 저장할 때 써넣는 키입니다.

**해결**
```bash
sed -i '/^mode:/d' ~/maps/파일.yaml
```

**예방** — 저장할 때마다:
```bash
ros2 run nav2_map_server map_saver_cli -f ~/maps/이름 --occ 0.65 --free 0.25
sed -i '/^mode:/d' ~/maps/이름.yaml
```

**추가 권장**: yaml 마지막 줄에 개행이 없으면 파서가 마지막 키를 놓칠 수
있습니다. `image:`는 상대 경로(파일명만)가 더 안정적입니다.

정상 yaml:
```yaml
image: office.pgm
resolution: 0.05
origin: [-7.71, -7.39, 0.0]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.25
```

---

## 문제 6 — `planner_server` 무응답, 나머지 노드 inactive (가장 헷갈리는 것)

```
map_server             active [3]
amcl                   active [3]
controller_server      active [3]
planner_server         무응답
recoveries_server      inactive [2]
bt_navigator            inactive [2]
```

**로그**
```
[global_costmap]: Timed out waiting for transform from base_footprint to map
tf error: Invalid frame ID "map" ... frame does not exist
```

**원인**: `map → odom` TF를 발행하는 것은 AMCL인데,
AMCL은 **초기 위치를 받기 전까지 발행하지 않습니다.**
`global_costmap`이 무한 대기 → `planner_server` configure 미완료 →
서비스 무응답 → lifecycle manager가 그 뒤를 못 올림.

**이건 버그가 아니라 실행 순서입니다.**

**해결** — 초기 위치를 주면 연쇄적으로 풀립니다.
```bash
ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped "{header: {frame_id: 'map'}, pose: {pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}, covariance: [0.25,0,0,0,0,0, 0,0.25,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0.068]}}"

timeout 5 ros2 run tf2_ros tf2_echo map odom
```

여전히 안 올라오면 수동으로:
```bash
timeout 20 ros2 lifecycle set /planner_server configure
timeout 10 ros2 lifecycle set /planner_server activate
timeout 10 ros2 lifecycle set /recoveries_server activate
timeout 10 ros2 lifecycle set /bt_navigator activate
```

> RViz에서 Fixed Frame을 `map`으로 두면 아무것도 안 보여
> 2D Pose Estimate를 못 찍는 딜레마가 있습니다. 그래서 명령으로
> 먼저 TF를 만들고, 그 다음 RViz에서 정밀하게 다시 찍습니다.

**주의 (문제13과 연결)**: `--once`가 discovery 타이밍 때문에 씹히는 경우가
있습니다. 반복 발행으로 바꿀 때는 **반드시 이 pub을 실행한 바로 그 터미널에서만**
Ctrl+C 하세요. Nav2를 띄운 터미널에서 잘못 Ctrl+C 하면 스택 전체가 죽습니다.

---

## 문제 7 — 맵이 흐릿하고 벽이 흩뿌려진 점으로 나온다

**파라미터 문제가 아니라 주행 방식 문제일 가능성이 큽니다.**

| 잘못된 주행 | 올바른 주행 |
|---|---|
| 방 중앙에서 원형 | **벽에서 1.5~2m** 유지 |
| 코너 없음 | **사각형**, 코너마다 3+3초 정지 |
| 한 방향 1바퀴 | **양방향 2바퀴** |
| 문 열어둠 | 문 닫기 |

유효점 수평거리 p50이 0.95m이므로, 방 중앙(벽까지 5~7m)에서는
벽에 닿는 점이 전체의 5% 미만입니다.

같은 파라미터로 주행만 바꿔 재측정한 결과: 흩뿌려진 점 → **벽 이웃수 2.51**

**실사례 (2026-08-05)**: 사무실만 재주행했는데 이웃수 4.66이 나온 적이 있음.
`pkill` 정리 누락, `/scan` hz 저하, RViz로 실시간 확인 없이 다 걷고 나서야
결과를 본 것이 원인으로 의심됨. 걷는 동안 RViz를 계속 띄워놓고 벽이 두 겹으로
갈라지는 순간 바로 멈춰서 되돌아가 재스캔하는 것이 사후 확인보다 훨씬 빠르다.

---

## 문제 8 — `map_saver_cli`가 timeout

```
[ERROR] [map_saver]: Failed to save the map: timeout
```

**원인**: `/map` 토픽을 발행하는 노드가 없음. slam_toolbox가 이미 종료됨.

**저장은 매핑이 돌아가는 중에만 됩니다.**

---

## 문제 9 — Nav2가 `/cmd_vel`은 내는데 로봇이 안 움직인다

| 확인 | 조치 |
|---|---|
| 휴대폰 앱이 연결되어 있는가 | 앱 연결 해제 |
| 로봇이 기립 상태인가 | 조종기 `L2+A` |
| 속도가 너무 작은가 | `cmd_vel_bridge`의 `min_vx` 0.10, `min_vyaw` 0.18 |
| sport lease가 잠겼는가 | `timeout 3 ros2 topic echo /api/sport/response --no-arr \| head -20` |

Nav2는 목표 근처에서 0.02~0.05 m/s를 냅니다.
Go2 sport mode는 이 정도로는 발을 떼지 않습니다.
`cmd_vel_bridge`의 `_boost()` 함수가 이를 최소 속도로 끌어올립니다.

---

## 문제 10 — `python3 - << 'PY'` 붙여넣기가 깨진다

터미널 붙여넣기에서 줄이 붙거나 잘리는 일이 흔합니다.

**대안 1** — `nano` 사용 (긴 파일은 이쪽 권장)
```bash
nano ~/go2_tools/파일.py
# 붙여넣기 → Ctrl+O → Enter → Ctrl+X
python3 -m py_compile ~/go2_tools/파일.py && echo OK
```

**대안 2** — MobaXterm SFTP 패널에 드래그 앤 드롭
복붙 사고가 원천적으로 없습니다.

**주의**: `mkdir -p`를 안 하고 `cat > ~/없는폴더/파일`을 하면
`No such file or directory`가 납니다.

---

## 문제 11 — 자원 관련

| 증상 | 조치 |
|---|---|
| `/scan` hz가 5 이하 | `sync:=false`(async), `minimum_travel_distance` 상향 |
| loadavg가 코어 수 초과 | RViz 종료, costmap 축소, `max_particles` 하향 |
| RViz가 느림 (X11 forwarding) | PointCloud2 끄고 LaserScan만. MobaXterm Compression 켜기. HDMI 직결이 최선 |
| posegraph가 너무 큼 | `minimum_travel_distance` 0.25 → 0.35 |

---

## 문제 12 — RViz에서 Map이 안 그려짐 (GLSL 에러)

```
[ERROR] rviz2: Vertex Program:rviz/glsl120/indexed_8bit_image.vert
Fragment Program:rviz/glsl120/indexed_8bit_image.frag GLSL link result:
active samplers with a different type refer to the same texture image unit
```

**원인**: MobaXterm/X11 원격 디스플레이의 구형 GPU 드라이버(OpenGL 3.1,
GLSL 1.4)가 Map 디스플레이가 쓰는 8bit indexed 텍스처 셰이더와 호환되지
않습니다. LaserScan 등은 정상 렌더링되지만 점유격자 이미지만 안 그려질 수
있습니다. 이 상태에서 "빨간 점을 벽선에 맞추라"는 작업 자체가 눈으로
불가능하니, Map이 실제로 그려지는지부터 확인하세요.

**해결** — 소프트웨어 렌더링 강제:
```bash
export LIBGL_ALWAYS_SOFTWARE=1
rviz2
```
속도는 느려지지만 이 셰이더 충돌을 우회하는 가장 확실한 방법입니다.

---

## 문제 13 — `/initialpose`가 계속 (0,0,0)으로 리셋된다

**증상**: 2D Pose Estimate로 제대로 맞춰도 1초 뒤 다시 어긋남.
AMCL 로그에 `initialPoseReceived`가 1초 간격으로 계속 찍힘.

**원인**: 부트스트랩용으로 반복 발행했던
`ros2 topic pub /initialpose ...`(--once 없이 실행한 것)를 Ctrl+C로
멈추지 않고 다른 작업으로 넘어감. 이 프로세스가 백그라운드에 남아
1초마다 예전 값을 계속 재발행하면서, 방금 맞춘 정확한 위치를 덮어씀.

**확인**
```bash
ps aux | grep "topic pub" | grep -v grep
```

**해결**
```bash
pkill -f "topic pub"
```
그다음 초기 위치를 다시 맞춥니다. **명령줄로 반복 발행하는 방식은 되도록
피하고, RViz의 2D Pose Estimate(클릭 한 번으로 끝나고 백그라운드에
아무것도 안 남음)를 우선 사용하세요.**

---

## 문제 14 — AMCL 위치추정이 계속 발산한다 (제자리 회전 시 특히)

**증상**: 전역 위치 재탐색(`/reinitialize_global_localization`) 후 회전시켜도
입자가 수렴하지 않거나, 수렴한 것처럼 보여도 스캔이 벽과 안 맞음(확신은
있는데 틀린 위치로 오수렴).

**원인**: Go2는 다리로 걷기 때문에 제자리에서 빠르게 회전할 때 다리 미끄러짐으로
오도메트리 오차가 커집니다. AMCL의 `alpha1~4`(모션 노이즈 파라미터)가
0.30으로 다소 높게 잡혀 있어(바퀴 로봇 대비 보정한 값), 빠른 회전 시 입자가
과도하게 퍼지거나 엉뚱하게 수렴할 수 있습니다.

**해결 순서**
```bash
# 1) 모션 노이즈를 낮춰서 재시작 없이 시도
ros2 param set /amcl alpha1 0.15
ros2 param set /amcl alpha2 0.15
ros2 param set /amcl alpha3 0.15
ros2 param set /amcl alpha4 0.15

# 2) 전역 재탐색
ros2 service call /reinitialize_global_localization std_srvs/srv/Empty
```

그다음:
- **360도씩 한 번에 돌리지 말고 30~45도씩 끊어서, 매번 3~5초 정지하며
  `/particlecloud`(PoseArray, Arrow Length를 1.0 이상으로 키워서 확인)가
  좁아지는지 관찰**
- **제자리 회전보다 짧은 직진(1~2m)이 이 로봇에서는 더 안정적으로 수렴하는
  경우가 많음** — 회전으로 안 되면 직진으로 전환
- 로봇의 실제 위치를 어느 정도 알고 있다면(매핑 시작 지점, 이미 등록된
  waypoint 등) 위치(x,y) 공분산은 좁게, yaw 공분산만 크게(예: 6.85) 줘서
  "위치는 확신, 방향만 탐색"하게 만들면 훨씬 빨리 수렴함

**판정 기준**: 빨간 스캔점이 검은 벽선과 5cm 이내로 겹치고, PoseArray
화살표들이 흩어지지 않고 한 방향으로 뭉쳐 있어야 함.

---

## 문제 15 — 빌드했는데 새 실행파일(`ros2 run`)이 안 보인다

```
ros2 run go2_nav_bridge waypoint_tool save stair_entry
No executable found
```

**원인**: `setup.py`의 `entry_points`에는 등록되어 있는데, 워크스페이스
소스(`~/ros2_ws/src/go2_nav_bridge`)가 최신 코드와 동기화가 안 됐거나,
colcon 빌드 캐시가 꼬여서 새 스크립트가 반영 안 됨.

**확인**
```bash
ls ~/ros2_ws/src/go2_nav_bridge/go2_nav_bridge/
cat ~/ros2_ws/src/go2_nav_bridge/setup.py | grep -A6 entry_points
```

**해결** — 해당 패키지만 클린 빌드
```bash
cd ~/ros2_ws
rm -rf build/go2_nav_bridge install/go2_nav_bridge log/latest_build/go2_nav_bridge
colcon build --packages-select go2_nav_bridge --symlink-install
source install/setup.bash
ros2 pkg executables go2_nav_bridge
```

**주의**: 빌드는 그 터미널에만 반영됩니다. **다른 터미널도 전부**
`source ~/ros2_ws/install/setup.bash`를 다시 해야 새 실행파일을 인식합니다
(문제 1과 같은 원인).

---

## 응급 정지

1. 조종기 **L2+A**(기립) 또는 **L2+B**(엎드림)
2. 전원 버튼 길게
3. ROS: `pkill -f cmd_vel_bridge` (종료 시 STOPMOVE 자동 발행)
4. 계단 등반 중이면: `stair_traverse_node`를 Ctrl+C — 종료 시 Move(0,0,0) → StopMove 발행됨

---

## 전체 초기화

```bash
pkill -f slam_toolbox; pkill -f scan_maker; pkill -f cmd_vel_bridge
pkill -f nav2; pkill -f map_server; pkill -f amcl
pkill -f controller_server; pkill -f planner_server
pkill -f bt_navigator; pkill -f recoveries_server; pkill -f waypoint_follower
pkill -f "topic pub"; pkill -f stair_traverse_node
sleep 3
ros2 daemon stop && ros2 daemon start
sleep 2
ros2 node list
```

---

## 로그 확인

```bash
# 최근 로그 디렉토리
ls -t ~/.ros/log/ | head -5

# 특정 노드 로그 (파일 자체이므로 tail 직접)
ls -t ~/.ros/log/planner_server_*.log | head -1 | xargs tail -50
ls -t ~/.ros/log/map_server_*.log | head -1 | xargs tail -50
ls -t ~/.ros/log/amcl_*.log | head -1 | xargs tail -50
```
