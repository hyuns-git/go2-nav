# 07 — 계단 자동 등반 (꺾인 착지참 포함)

## 왜 SLAM/Nav2 밖에서 처리하는가

`go2_scan_maker`의 `base_footprint` 투영은 평지 보행 중 ±3~7°의 흔들림을
보정하는 용도다. 실제 계단에서는 pitch가 지속적으로 20~30°+ 로 커지므로 이
전제가 깨지고, 2D `/scan` 자체가 의미 없어진다 (계단은 2D 점유격자로 표현할
수 없는 지형). 따라서 계단 구간은 다음 두 지점을 잇는, 지도 밖의
스크립트화된 구간으로 다룬다.

- `stair_entry` : 3층 지도 위, 계단을 정면으로 마주보는 지점 (Nav2 `NavigateToPose`로 도달)
- (다음 층 지도가 있는 경우) 도착 지점 : 계단을 다 오른 뒤 AMCL을 재초기화할 지점

이 건물 계단은 단일 직선이 아니라 **9단 → 평지 착지참에서 180도 유턴 → 5단**
구조다. `stair_traverse_node`는 계단을 "구간(segment) 리스트"로 다뤄서
(climb → turn → climb) 착지참에서의 오판을 방지한다.

---

## Nav2 쪽 준비

### 1. 계단실을 금지구역으로 처리

Nav2가 실제 계단 단 위로 경로를 잡지 못하도록, `stair_entry`보다 살짝
안쪽(계단 쪽) 지점부터 계단실 반대쪽 끝까지를 점유(검정)로 칠한다.

```bash
# RViz Publish Point로 두 대각 좌표 수집
ros2 topic echo /clicked_point

python3 ~/go2-nav/tools/block_area.py ~/maps/office_ext01.yaml office_ext01_edit.pgm \
  <x1> <y1> <x2> <y2>

sed -i 's/^image:.*/image: office_ext01_edit.pgm/' ~/maps/office_ext01.yaml
```

검증 (중간값 없이 0/205/254만 있어야 함):
```bash
python3 -c "
from PIL import Image; import numpy as np
a=np.array(Image.open('/home/unitree/maps/office_ext01_edit.pgm'))
u=np.unique(a); print(u)
print('OK' if set(u.tolist())<={0,205,254} else '경고: 중간값 존재')
"
```

`stair_entry` 지점 자체는 자유공간으로 남겨서 Nav2가 거기까지는 도달할 수
있게 해야 한다.

### 2. `stair_entry` 웨이포인트 등록

로봇을 계단을 정면으로 보는 자세로 계단 입구에 세운 뒤, TF가 살아있는 상태에서:

```bash
timeout 3 ros2 run tf2_ros tf2_echo map base_footprint   # 멈추지 않고 값이 나오는지 확인
ros2 run go2_nav_bridge waypoint_tool save stair_entry
ros2 run go2_nav_bridge waypoint_tool list
```

`waypoint_tool save`는 RViz 클릭이 필요 없다. 로봇의 현재 TF 위치·방향을
그대로 스냅샷해서 `~/maps/waypoints.yaml`에 저장한다.

---

## `stair_traverse_node`

### 설치

`src/go2_nav_bridge/go2_nav_bridge/stair_traverse_node.py`를 추가하고,
`setup.py`의 `entry_points`에 다음 줄이 있는지 확인 후 빌드한다.

```python
'stair_traverse_node = go2_nav_bridge.stair_traverse_node:main',
```

```bash
cd ~/ros2_ws
colcon build --packages-select go2_nav_bridge --symlink-install
source install/setup.bash
ros2 pkg executables go2_nav_bridge | grep stair
```

### 동작 원리

전제:
- `/api/sport/request`(unitree_api/msg/Request)로 SportClient API 호출
  (`cmd_vel_bridge`가 API 1008 Move에 쓰는 것과 동일한 방식)
- `/utlidar/robot_odom`(nav_msgs/Odometry)의 IMU 쿼터니언으로 pitch/yaw 계산

구간 종류:
- **climb** : 전진하며 오름. pitch가 `pitch_climb_deg` 이상으로 올라갔다가
  다시 `pitch_level_deg` 이하로 `level_hold_sec` 이상 유지되면 종료. 단,
  `steps * seconds_per_step * min_climb_margin` 이전에는 종료 판정을 하지
  않아서 착지참을 "다음 flight의 끝"으로 착각하지 않게 한다.
- **turn** : 제자리 회전. odom yaw 변화량을 누적해 목표 각도(허용오차
  `turn_tolerance_deg`)에 도달하면 종료.

계단을 실제로 오르내리는 동작(발 위치, 균형)은 전적으로 Go2 내장 `ClassicWalk`
게이트가 처리한다. `stair_traverse_node`가 튜닝하는 건 오르는 동작 자체가
아니라, "지금 어느 구간에 있는지"를 판단해 전진/회전을 전환하는 오케스트레이션
레이어다. SDK가 "계단 끝"이나 "착지참 도착" 같은 신호를 주지 않기 때문에
직접 pitch/yaw로 추정해야 한다.

### 파라미터 보정

계단 실측(단수, 총 수평거리, 경사각)이 없어도, **컨트롤러로 한 번 수동으로
오른 기록**(`/utlidar/robot_odom`을 echo하거나 rosbag)에서 뽑아 쓰면 된다.

| 값 | 얻는 방법 |
|---|---|
| `seconds_per_step` | 각 flight 진입~착지참까지 걸린 시간 ÷ 그 flight 단수 |
| `pitch_climb_deg` | 해당 flight에서 관측된 pitch 최대값보다 살짝 낮게 |
| `pitch_level_deg` / `level_hold_sec` | 착지참에서 pitch가 0 근처로 돌아온 뒤 유지된 시간 (참고용) |

### 실행

```bash
ros2 run go2_nav_bridge stair_traverse_node --ros-args \
  -p flight_step_counts:="[9, 5]" \
  -p turn_angles_deg:="[180.0]" \
  -p turn_direction:=1.0 \
  -p seconds_per_step:=1.2 \
  -p forward_speed:=0.25 \
  -p turn_speed:=0.4 \
  -p max_duration_sec:=90.0
```

- `turn_direction`: +1.0 = 반시계, -1.0 = 시계. 실제 착지참에서 유턴 방향에 맞게.
- `flight_step_counts`와 `turn_angles_deg`의 길이 관계: `len(turn_angles_deg) == len(flight_step_counts) - 1`.

전체 흐름:
```
SpeedLevel(-1) → FreeAvoid(false) → ClassicWalk(true)
  → climb(9단) → turn(180°) → climb(5단)
  → Move(0,0,0) → StopMove() → ClassicWalk(false) → BalanceStand()
```

---

## 전체 시나리오 연결

```bash
# 1) 3층 맵에서 계단 입구까지 자율 이동
ros2 run go2_nav_bridge waypoint_tool go stair_entry

# 2) 도착 확인 후 (방향이 저장된 yaw와 맞는지) 계단 등반
ros2 run go2_nav_bridge stair_traverse_node --ros-args \
  -p flight_step_counts:="[9, 5]" -p turn_angles_deg:="[180.0]" \
  -p turn_direction:=1.0 -p seconds_per_step:=1.2
```

4층 지도가 아직 없다면 여기까지가 이번 단계의 범위다. 4층 지도를 만든
뒤에는 계단 상단 도착 지점에서 `nav2_map_server`의 `load_map` 서비스로 맵을
전환하고 `/initialpose`로 AMCL을 그 지점 좌표로 재초기화하면, 3층 출발점부터
4층 목표 지점까지 완전히 이어서 자율주행할 수 있다.

---

## 안전 체크리스트

- 첫 실행은 반드시 사람이 옆에서 지켜보며(추락/전복 대비) 저속으로 진행.
- 페이로드(Jetson Orin Nano 독, LiDAR)가 있는 구성에서는 `AutoRecoverySet(true)`가
  넘어졌을 때 격하게 일어나다가 장비를 파손할 수 있다는 게 공식 문서의 경고다.
  계단 근처에서는 `AutoRecoverySet(false)`를 검토하고, 사람이 즉시 개입 가능한
  거리에서 테스트할 것.
- `max_duration_sec` 타임아웃으로 중간에 멈추는 경우를 대비해 `Damp()`를
  즉시 호출할 수 있는 비상정지 스크립트를 준비해둘 것.
- 회전(turn) 구간은 계단 위가 아니라 반드시 평평한 착지참에서만 실행되어야
  한다. `min_climb_margin`을 너무 낮게 잡으면 계단 중간에서 회전을 시도해
  낙상 위험이 커지니, 착지참 도착 판정은 보수적으로 잡을 것.

## 이번 단계에서 하지 않는 것

- 4층 맵 생성 및 층간 맵 전환 자동화 — 다음 단계.
- 완전 자동 트리거(perception 기반 계단 자동 감지로 웨이포인트 없이 진입) —
  현재 SDK 상태 인터페이스(`rt/sportmodestate`)에는 "전방 지형 종류" 필드가
  없어서, 고정 웨이포인트 + 실측 파라미터 기반 스크립트 방식이 현실적인 접근.
