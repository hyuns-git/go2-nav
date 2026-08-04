# 매핑 절차 (터미널별 명령 포함)

**소요 시간**: 검증 30분 + 시험 매핑 15분 + 본 매핑 30~60분

---

## 0. 매번 실행하는 사전 정리

**어떤 작업을 시작하든 이것부터.** 노드가 중복 실행되면 TF가 충돌하고
`/map`이 두 개 발행되어 RViz에서 맵 크기가 계속 바뀝니다.

```bash
pkill -f slam_toolbox; pkill -f scan_maker; pkill -f cmd_vel_bridge
pkill -f nav2; pkill -f map_server; pkill -f amcl
pkill -f controller_server; pkill -f planner_server
pkill -f bt_navigator; pkill -f recoveries_server; pkill -f waypoint_follower
sleep 3
ros2 daemon stop && ros2 daemon start
sleep 2
ros2 node list
```

`ros2 node list`에 아무것도 없거나 `transform_listener_impl_*`만 있으면 정상.

---

## 1. 터미널 구성

**총 4개 터미널이 필요합니다.** MobaXterm 탭을 4개 열고 이름을 붙이세요
(탭 우클릭 → Rename): `MAP`, `SAVE`, `DIAG`, `RVIZ`

**모든 터미널에서 환경을 먼저 잡아야 합니다.**

```bash
source ~/unitree_ros2/setup.sh
source ~/ros2_ws/install/setup.bash
```

`~/.bashrc`에 넣어두면 자동으로 적용됩니다:
```bash
cat >> ~/.bashrc << 'EOF'

# --- Go2 ROS2 ---
source ~/unitree_ros2/setup.sh
source ~/ros2_ws/install/setup.bash
EOF
```

> 이걸 안 하면 `Package 'go2_nav_bridge' not found`가 뜹니다.
> `.bashrc`는 **새로 여는 터미널**부터 적용되며, 지금 열린 터미널에는
> 위 두 줄을 직접 실행하세요.

---

## 2. 사전 검증 (본 매핑 전 필수)

### 터미널 MAP — 스캔만 실행

```bash
ros2 launch go2_nav_bridge scan_only.launch.py
```

기대 로그:
```
[go2_scan_maker]: scan_maker: 180 beams (2.00 deg), accum 0.75s, z=[0.05,0.60]
```

`waiting for odometry...`는 첫 수신 전 한 번만 뜨는 정상 경고입니다.

### 터미널 DIAG — 스캔 품질 측정

```bash
python3 ~/go2-nav/tools/scan_quality.py 30
```

| 결과 | 판정 | 조치 |
|---|---|---|
| 55% 이상 | 우수 | 진행 |
| **40~55%** | **양호** | 진행 |
| 25~40% | 부족 | `accumulate_sec` 상향, `angle_increment` 확대 |
| 25% 미만 | 심각 | 높이 밴드 재측정 (`tools/cloud_inspect.py`) |

### 터미널 DIAG — TF 확인

```bash
timeout 5 ros2 run tf2_ros tf2_echo odom base_footprint
```

`Translation`의 **z가 항상 0.000**이고 Quaternion의 **x, y가 0.000**이어야 합니다.
이것이 `base_footprint`가 중력 정렬 프레임으로 동작한다는 증거입니다.

### 터미널 RVIZ — 회전 테스트 (가장 중요)

```bash
rviz2
```

**설정**

| 항목 | 값 |
|---|---|
| Global Options → Fixed Frame | `odom` |
| Add → TF | (체크) |
| Add → LaserScan → Topic | `/scan` |
| LaserScan → Style | `Points` |
| LaserScan → Size (Pixels) | `5` |
| LaserScan → **Color Transformer** | **`FlatColor`** |
| LaserScan → Color | `255; 0; 0` (빨강) |
| LaserScan → **Decay Time** | **`10`** |
| Views → Type | **`TopDownOrtho`** |

> `Color Transformer`를 기본값 `Intensity`로 두면 점이 안 보입니다.
> `scan_maker`가 만드는 LaserScan에는 intensity 필드가 없기 때문입니다.

**테스트**: 로봇을 기립시키고 조종기로 **제자리에서 아주 천천히**
한 바퀴(20초 이상) 회전.

| 관찰 | 판정 | 조치 |
|---|---|---|
| 벽선이 **한 겹** 유지 | 통과 | 다음 단계 |
| 회전 방향으로 번짐 | 누적 과다 | `accumulate_sec` 0.75 → 0.5 |
| 여러 겹으로 갈라짐 | TF 충돌 | 구 `odom_tf_publisher` 중복 실행 확인 |
| 점이 거의 없음 | 밴드 오류 | `tools/cloud_inspect.py` 재측정 |

---

## 3. 시험 매핑 (본 매핑 전 리허설 — 건너뛰지 말 것)

**본 매핑 2시간을 날리지 않으려면 15분짜리 리허설이 필수입니다.**

### 터미널 MAP

```bash
# 위 scan_only 를 Ctrl+C 로 종료 후
ros2 launch go2_nav_bridge mapping.launch.py sync:=true
```

`sync:=true`는 동기 SLAM입니다. 스캔을 버리지 않아 품질이 높습니다.
CPU가 못 버티면(`/scan` hz가 5 이하로 떨어지면) `sync:=false`로 전환하세요.

### 터미널 RVIZ — 설정 변경

| 항목 | 값 |
|---|---|
| Fixed Frame | `odom` → **`map`** |
| Add → Map → Topic | `/map` |
| Map → **Durability Policy** | **`Transient Local`** (필수) |
| LaserScan → Decay Time | `10` → **`0`** |
| Views → Type | `TopDownOrtho` |

> `Durability Policy`를 `Volatile`로 두면 맵이 아예 안 보입니다.

### 주행 (규칙 엄수)

```
출발점 S 선정 — 벽에서 1.5~2m, 바닥에 테이프 표시
   ↓ 10초 정지
벽을 따라 사각형으로 주행 (한 변 5~7m, 둘레 20~30m)
   · 속도 0.3 m/s 이하
   · 벽에서 1.5~2m 유지   ★ 방 중앙 금지
   ↓
코너: [3초 정지] → 30도씩 3번 나눠 회전 → [3초 정지]
   ↓
S 복귀 → 10초 정지        ★ RViz에서 맵이 살짝 정렬되면 루프 폐합 성공
   ↓
반대 방향으로 한 바퀴 더 → S 복귀 → 10초 정지
```

**왜 이렇게 하는가**

| 규칙 | 이유 |
|---|---|
| 벽에서 1.5~2m | 유효점 수평거리 p50이 0.95m. 방 중앙에서는 벽에 닿는 점이 5% 미만 |
| 사각형(원 아님) | 코너가 scan matching의 기준점이 됨. 원형 경로에는 기준점이 없음 |
| 회전 전 3초 정지 | 직선 구간에서 누적된 오차를 보정한 뒤 회전. 틀어진 채로 도는 것 방지 |
| 회전 후 3초 정지 | 회전은 오차가 가장 크게 생기는 동작. 새 방향에서 재보정 |
| 30도씩 나눠 회전 | 오도메트리는 회전에서 가장 부정확 |
| 양방향 2바퀴 | 루프 폐합은 확률적. 한 번에 안 걸릴 수 있음 |
| 문 닫기 | 자유공간이 밖으로 새는 것 방지 |

### 터미널 SAVE — 저장 및 평가

```bash
ros2 run nav2_map_server map_saver_cli -f ~/maps/test01 --occ 0.65 --free 0.25
sed -i '/^mode:/d' ~/maps/test01.yaml          # ★ Foxy map_server 는 mode 키를 못 읽음
ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph \
  "{filename: '/home/unitree/maps/test01'}"

python3 ~/go2-nav/tools/eval_map.py ~/maps/test01.pgm
```

### 판정 기준

| 항목 | 통과 |
|---|---|
| **벽 평균 이웃수** | **3.0 미만** (가장 중요) |
| 벽이 연속된 선 | 흩뿌려진 점이 아님 |
| 직각 코너 90도 | 유지 |
| 시작점 부근 벽 정합 | 어긋나지 않음 |
| 자유공간 누출 없음 | 벽 안에 갇힘 |

**통과 못 하면 주행 방식부터 점검하세요.** 파라미터가 아니라 주행이
원인인 경우가 대부분입니다.

---

## 4. 본 매핑

### 터미널 MAP

```bash
pkill -f slam_toolbox; pkill -f scan_maker; sleep 2
ros2 launch go2_nav_bridge mapping.launch.py sync:=true
```

### 터미널 SAVE — 자동 중간 저장 (15분 간격)

```bash
bash ~/go2-nav/tools/autosave.sh
```

### 터미널 DIAG — 상태 감시

```bash
watch -n 5 'timeout 4 ros2 topic hz /scan 2>&1 | grep -m1 average; cat /proc/loadavg'
```

| 지표 | 정상 | 이상 시 |
|---|---|---|
| `/scan` hz | 9~11 | 5 이하 → `sync:=false`로 전환 |
| loadavg | 코어 수 이하 | 초과 → RViz 끄기, async 전환 |

### 주행 순서

```
[00분] S에서 30초 정지 (초기 스캔 안정화)
   ↓
외곽을 벽 따라 시계방향 한 바퀴 (1.5~2m 유지, 코너마다 3+3초)
   ↓
S 복귀 → 15초 정지          ★ 대루프 폐합
   ↓
반대 방향 한 바퀴 → S 복귀 → 15초 정지
   ↓
방 하나씩: 복도에서 진입 → 벽 따라 한 바퀴 → 같은 문으로 퇴출 → 복도 복귀
   (매 구역마다 복도 복귀 = 소루프 폐합)
   ↓
마지막에 외곽 한 바퀴 더 → S 복귀
```

**주행 중 벽이 두 겹으로 갈라지면 그 자리에서 멈추고,
방금 지나온 구간을 되돌아가 재스캔하세요.** 무시하면 끝까지 따라갑니다.

**배터리 교체**: SOC 25%에서 그 자리에 정지 → 수동 저장 →
**로봇을 움직이지 말고** 교체 → `deserialize_map`으로 재개

```bash
ros2 service call /slam_toolbox/deserialize_map \
  slam_toolbox/srv/DeserializePoseGraph \
  "{filename: '/home/unitree/maps/파일명', match_type: 1}"
```
`match_type: 1` = START_AT_FIRST_NODE, `2` = START_AT_GIVEN_POSE

### 최종 저장

```bash
ros2 run nav2_map_server map_saver_cli -f ~/maps/office --occ 0.65 --free 0.25
sed -i '/^mode:/d' ~/maps/office.yaml
ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph \
  "{filename: '/home/unitree/maps/office'}"

python3 ~/go2-nav/tools/eval_map.py ~/maps/office.pgm
tar czf ~/maps/office_$(date +%Y%m%d_%H%M).tar.gz -C ~/maps \
  office.pgm office.yaml office.posegraph office.data
```

**4개 파일을 모두 저장하세요.** `.pgm`/`.yaml`은 Nav2용,
`.posegraph`/`.data`는 이어 매핑 및 slam_toolbox localization용입니다.

---

## 5. 금지구역 만들기 (책상 밑, 계단 등)

Foxy에는 Keepout Filter가 없으므로 **맵 이미지에 직접 그리는 것이 유일한 방법**입니다.

### 방법 A — 좌표로 사각형 막기

**터미널 DIAG**에서 좌표 수집 (RViz 상단 `Publish Point` 도구로 클릭):
```bash
ros2 topic echo /clicked_point
```

막을 영역의 두 대각 모서리 좌표를 메모한 뒤:
```bash
python3 ~/go2-nav/tools/block_area.py ~/maps/office.yaml office_edit.pgm \
  -3.2 -5.1  1.8 -6.9
```
여러 구역이면 4개씩(x1 y1 x2 y2) 이어서 붙이면 됩니다.

### 방법 B — GIMP

1. `office.pgm`을 GIMP로 열기
2. **연필(Pencil) 도구** 사용 ★ 붓(Paintbrush) 금지 — 안티에일리어싱이 중간값을 만듦
3. 전경색 **완전 검정 (0,0,0)**
4. 사각 선택 후 `편집 → 전경색으로 채우기`가 가장 확실
5. `Export As` → `office_edit.pgm` → **Raw** 선택
6. `office.yaml`의 `image:`를 `office_edit.pgm`으로 수정

**검증**:
```bash
python3 -c "
from PIL import Image; import numpy as np
a=np.array(Image.open('/home/unitree/maps/office_edit.pgm'))
u=np.unique(a); print(u)
print('OK' if set(u.tolist())<={0,205,254} else '경고: 중간값 존재')
"
```

**원본은 절대 덮어쓰지 마세요.** 나중에 맵을 다시 손볼 때 필요합니다.
