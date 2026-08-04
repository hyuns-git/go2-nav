# 자율주행 & Point-to-Point (터미널별 명령 포함)

---

## 반드시 알아야 할 실행 순서

**Nav2를 띄운 직후에는 `map` 프레임이 존재하지 않습니다.**

`map → odom` TF를 발행하는 것은 AMCL인데, AMCL은 **초기 위치를 받기 전까지
이 TF를 발행하지 않습니다.** 그래서 `global_costmap`이 무한 대기에 빠지고,
`planner_server`의 configure가 끝나지 않고, `recoveries_server`와
`bt_navigator`가 `inactive`에 머뭅니다.

```
Invalid frame ID "map" passed to canTransform argument target_frame
- frame does not exist
```

**이건 버그가 아니라 순서 문제입니다.**

```
Nav2 실행
   ↓
map_server, amcl, controller_server 만 active
planner_server 무응답 / recoveries_server, bt_navigator inactive   ← 정상
   ↓
초기 위치 입력 (2D Pose Estimate 또는 /initialpose)
   ↓
map → odom TF 생성
   ↓
나머지 노드가 연쇄적으로 active
```

---

## 터미널 구성 — 총 5개

MobaXterm 탭 5개를 열고 이름을 붙이세요: `NAV`, `LIFE`, `POSE`, `WP`, `RVIZ`

**모든 터미널 공통 준비**
```bash
source ~/unitree_ros2/setup.sh
source ~/ros2_ws/install/setup.bash
```

---

## 터미널 NAV — Nav2 스택 실행

```bash
# 사전 정리
pkill -f slam_toolbox; pkill -f scan_maker; pkill -f cmd_vel_bridge
pkill -f nav2; pkill -f map_server; pkill -f amcl
pkill -f controller_server; pkill -f planner_server
pkill -f bt_navigator; pkill -f recoveries_server; pkill -f waypoint_follower
sleep 3

ros2 launch go2_nav_bridge navigation.launch.py \
  map:=/home/unitree/maps/test2.yaml
#yaml 파일 이름을 내가 실행시키고 싶은 파일이름으로 수정시킨 후 꼭 실행!!!
```

이 런치는 세 가지를 함께 띄웁니다:
- `scan_maker` (accumulate_sec 0.55 — 주행 반응성 위해 매핑보다 짧게)
- `cmd_vel_bridge`
- Nav2 전체 스택

> **맵 yaml에 `mode:` 줄이 있으면 `map_server`가 configure에 실패합니다.**
> Foxy `map_server`가 이 키를 인식하지 못합니다. 저장 시
> `sed -i '/^mode:/d' 파일.yaml`로 제거하세요.

---

## 터미널 LIFE — 라이프사이클 감시

```bash
for n in map_server amcl controller_server planner_server recoveries_server bt_navigator; do
  printf "%-22s " $n; timeout 3 ros2 lifecycle get /$n 2>/dev/null || echo "무응답"
done
```

> `timeout`을 반드시 붙이세요. 응답 없는 노드를 만나면
> `ros2 lifecycle get`이 무한 대기합니다.

**초기 상태(정상)**
```
map_server             active [3]
amcl                   active [3]
controller_server      active [3]
planner_server         무응답        ← map 프레임 대기 중
recoveries_server      inactive [2]
bt_navigator           inactive [2]
```

---

## 터미널 POSE — 초기 위치 부트스트랩

**RViz에서 Fixed Frame을 `map`으로 두면 아무것도 안 보여서
2D Pose Estimate를 찍을 수 없는 딜레마가 있습니다.**
명령으로 먼저 TF를 만들어 주세요.

```bash
ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped "{header: {frame_id: 'map'}, pose: {pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}, covariance: [0.25,0,0,0,0,0, 0,0.25,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0.068]}}"
```

위치가 정확하지 않아도 됩니다. **TF만 살아나면** 나머지 노드가 올라오고,
그 다음 RViz에서 제대로 찍으면 됩니다.

**확인**
```bash
timeout 5 ros2 run tf2_ros tf2_echo map odom
```
값이 나오면 성공.

**라이프사이클 재확인** (터미널 LIFE) — 여전히 안 올라오면 수동으로:
```bash
timeout 20 ros2 lifecycle set /planner_server configure
timeout 10 ros2 lifecycle set /planner_server activate
timeout 10 ros2 lifecycle set /recoveries_server activate
timeout 10 ros2 lifecycle set /bt_navigator activate
```

전부 `active [3]`이 되어야 합니다.

---

## 터미널 RVIZ — 시각화 및 위치 지정

```bash
rviz2
```

### 설정

| Display | 항목 | 값 |
|---|---|---|
| Global Options | Fixed Frame | **`map`** |
| Map | Topic | `/map` |
| Map | **Durability Policy** | **`Transient Local`** |
| LaserScan | Topic | `/scan` |
| LaserScan | Style | `Points` |
| LaserScan | Size (Pixels) | `5` |
| LaserScan | **Color Transformer** | **`FlatColor`** |
| LaserScan | Color | `255; 0; 0` |
| LaserScan | Decay Time | `0` |
| Views | Type | `TopDownOrtho` |

**진단용으로 추가 권장**

| Display | Topic | 설정 |
|---|---|---|
| Map (2번째) | `/global_costmap/costmap` | Color Scheme `costmap` |
| Map (3번째) | `/local_costmap/costmap` | Color Scheme `costmap` |
| Path | `/plan` | 전역 경로 |
| Path | `/local_plan` | 지역 경로 |
| PoseArray | `/particlecloud` | AMCL 입자 |

global costmap이 안 보이면 `map_subscribe_transient_local: True`인지
확인하세요.

### 초기 위치 정밀 지정 (가장 중요)

1. 로봇을 **맵 만들 때 출발했던 자리 근처**에 둡니다
2. 상단 **2D Pose Estimate** 클릭
3. 맵 위 로봇의 실제 위치를 클릭하고, **로봇이 바라보는 방향으로 드래그**
4. **빨간 스캔 점이 검은 벽선과 5cm 이내로 겹칠 때까지** 반복

잘 안 맞으면 조종기로 **제자리에서 천천히 한 바퀴** 회전시키세요.
AMCL 입자(`/particlecloud`)가 수렴하는 것이 보입니다.

**여기서 대충 맞추면 이후 모든 주행이 실패합니다.**

---

## 단계별 주행 테스트

> **조종기를 손에 들고 로봇 옆에 계세요.** 이상하면 즉시 개입.

### 테스트 A — 짧은 회전 (뒤쪽 2m)

RViz `Nav2 Goal`로 로봇 **뒤쪽 2m** 지정.
기대: 제자리 회전 → 직진 → 도착

### 테스트 B — 직선 5m

빈 공간 5m 앞.

### 테스트 C — 장애물 회피

경로 중간에 박스를 놓고 같은 목표. local costmap에 박스가 뜨는지 확인.

### 테스트 D — 좁은 통로

문(폭 0.9m 이상) 통과. 실패하면 `inflation_radius` 하향.

### 테스트 E — 장거리 (20m+, 코너 포함)

### 터미널 DIAG로 확인 (Foxy에는 `--once`, `--field` 없음)

```bash
timeout 3 ros2 topic echo /cmd_vel | head -20
timeout 3 ros2 topic echo /plan --no-arr | head -20
timeout 3 ros2 topic echo /local_plan --no-arr | head -20
timeout 3 ros2 topic echo /amcl_pose | head -30
```

---

## 터미널 WP — 웨이포인트 등록 및 이동

### 등록

로봇을 원하는 위치로 이동시킨 뒤(조종기 또는 Nav2 Goal):

```bash
ros2 run go2_nav_bridge waypoint_tool save 출발점
ros2 run go2_nav_bridge waypoint_tool save 엘리베이터
ros2 run go2_nav_bridge waypoint_tool save 회의실A
ros2 run go2_nav_bridge waypoint_tool save 탕비실
```

**등록 요령**

| 항목 | 이유 |
|---|---|
| 벽에서 **1m 이상** | `inflation_radius` 0.45 때문에 벽 근처는 도달 불가 판정 |
| 문 앞이 아니라 문에서 1.5m | 좁은 곳은 실패 확률이 높음 |
| 최종 바라볼 방향까지 맞춘 뒤 저장 | yaw도 함께 기록됨 |

저장 위치: `~/maps/waypoints.yaml` (직접 편집해 미세조정 가능)

### 이동

```bash
ros2 run go2_nav_bridge waypoint_tool list
ros2 run go2_nav_bridge waypoint_tool go 회의실A
ros2 run go2_nav_bridge waypoint_tool tour 엘리베이터 회의실A 탕비실 출발점
```

### 도달 정확도 측정

목표 지점 바닥에 테이프로 X 표시 → 도착 후 줄자로 오차 측정.

| 오차 | 판정 |
|---|---|
| 15cm 미만 | 우수 |
| 15~30cm | 양호 (실용 충분) |
| 30~50cm | `xy_goal_tolerance` 하향, AMCL 입자 상향 |
| 50cm 초과 | 맵 품질 또는 위치추정 재검토 |

---

## 속도 상향 프로토콜

**절대 한 번에 올리지 마세요.** 각 단계에서 3회 성공을 확인하고 진행합니다.

| 단계 | max_vel_x | max_speed_xy | max_vel_theta | 통과 조건 |
|---|---|---|---|---|
| L0 | 0.20 | 0.20 | 0.35 | 직선 5m 3/3 |
| **L1 (기본)** | **0.30** | **0.30** | **0.50** | 코너 포함 20m 3/3 |
| L2 | 0.40 | 0.40 | 0.60 | 좁은 문 3/3, 장애물 회피 3/3 |
| L3 | 0.50 | 0.50 | 0.70 | 50m 순회 3/3 |
| L4 | 0.60 | 0.60 | 0.80 | 권장 상한 |

**`max_vel_x`와 `max_speed_xy`를 반드시 함께 올리세요.**
`max_speed_xy`가 합성 속도 상한이라 이것이 실질적 제한이 됩니다.
하나만 올리면 무의미합니다.

**`cmd_vel_bridge`의 max는 DWB보다 10~20% 크게** 두세요.
같으면 클램프가 자주 걸려 DWB 예측과 실제가 어긋납니다.

**상향 중단 신호**: AMCL covariance 증가, 코너에서 벽에 근접,
회전 시 스캔 번짐, recovery 빈발

**속도를 올리면 함께 조정**

| 올린 것 | 함께 조정 |
|---|---|
| `max_vel_x` | `accumulate_sec` 하향, `sim_time` 상향, local costmap 확대 |
| `max_vel_theta` | `accumulate_sec` 하향, `vtheta_samples` 상향 |

---

## 종료

```bash
pkill -f nav2; pkill -f map_server; pkill -f amcl
pkill -f controller_server; pkill -f planner_server
pkill -f bt_navigator; pkill -f recoveries_server
pkill -f waypoint_follower; pkill -f scan_maker; pkill -f cmd_vel_bridge
```

`cmd_vel_bridge`는 종료 시 STOPMOVE를 발행하므로 로봇이 자동 정지합니다.
