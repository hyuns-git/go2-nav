# go2-nav — Unitree Go2 EDU 2D SLAM & Point-to-Point Navigation (ROS 2 Foxy)

Unitree Go2 EDU의 내장 4D LiDAR L1만으로 실내 2D 맵을 만들고, 그 맵 위에서
이름 붙인 지점 사이를 자율 이동(Point-to-Point)시키는 완전한 구성.

**모든 파라미터는 실측 데이터에 근거해 결정되었습니다.** 근거는
[docs/01-measurements.md](docs/01-measurements.md)에 전부 기록되어 있습니다.

---

## 검증 환경

| 항목 | 값 |
|---|---|
| 로봇 | Unitree Go2 EDU |
| 컴퓨터 | 확장독 Jetson Orin Nano (`192.168.123.18`, user `unitree`) |
| OS | Ubuntu 20.04 (JetPack 5), arm64 |
| ROS | ROS 2 Foxy |
| DDS | CycloneDDS (`unitree_ros2`) |
| LiDAR | Unitree 4D LiDAR L1 (내장) |
| 검증일 | 2026-08-04 |

---

## 왜 표준 파이프라인을 쓰지 않았는가

일반적인 `pointcloud_to_laserscan` + `slam_toolbox` 조합은 Go2에서 흐릿한 맵을
만듭니다. 실측으로 확인한 원인 3가지:

### 1. 포인트클라우드의 86.5%가 더미다

`/utlidar/cloud_deskewed`는 프레임당 11,554점을 보내지만, 그중
**10,000점이 정확히 `(0,0,0)`** 입니다. L1이 "반사 없음"을 0으로 채워
보내기 때문입니다. `isfinite()` 검사로는 걸러지지 않습니다.

이 점들은 odom 원점에 고정되어 있으므로, 로봇이 이동하면
**출발점에 12만 점짜리 유령 장애물**이 생깁니다.

→ 해결: `cloud_to_xyz()`에서 `(0,0,0)` 명시적 제거

### 2. 4족보행의 pitch/roll이 스캔에 실린다

`base_link`는 몸통 고정 프레임이라 걸을 때 ±3~7° 흔들립니다.
pitch 5°면 5m 앞 벽에서 높이 슬라이스가 44cm 밀립니다.

→ 해결: `odom → base_footprint(yaw만) → base_link` 3단 프레임 도입.
SLAM과 Nav2의 기준을 `base_footprint`로 통일.

### 3. 타임스탬프가 두 시계에 걸쳐 있다

메인 컨트롤 보드(LiDAR 발행)와 확장독 컴퓨터의 시계가
**실측 -49.13초** 차이납니다. 클라우드 원본 스탬프로 TF를 조회하면
tf2 버퍼(기본 10초) 밖이라 스캔이 버려지거나 엉뚱한 자세에 붙습니다.

→ 해결: 클라우드가 이미 odom 프레임이라는 점을 이용해 **TF 조회를 제거**.
단일 노드가 스캔과 TF를 같은 로컬 스탬프로 발행.

---

## 아키텍처

```
/utlidar/cloud_deskewed (PointCloud2, frame=odom, 11Hz, 11554 pts)
/utlidar/robot_odom     (Odometry,    frame=odom, ~50Hz)
            │
            ▼
  ┌────────────────────────────────────────┐
  │ go2_scan_maker  (핵심 노드)             │
  │  1. (0,0,0) 더미 제거  → 1556 pts       │
  │  2. odom 프레임에서 0.75초 누적          │
  │  3. 최신 yaw로 base_footprint 투영       │
  │  4. z=[0.05,0.60] 슬라이스 → LaserScan  │
  │  5. 스캔·TF·odom 동일 스탬프 발행 ★      │
  └────────────────────────────────────────┘
            │  /scan (180빔 2.0°, 10Hz, 유효율 51.7%)
            │  /odom
            │  /tf : odom→base_footprint→base_link
            ▼
   sync_slam_toolbox_node  →  /map
            │
            ▼
   nav2 (AMCL + NavFn + DWB)  →  /cmd_vel
            │
            ▼
   cmd_vel_bridge  →  /api/sport/request (API 1008 Move)
```

`pointcloud_to_laserscan`은 사용하지 않습니다.

---

## 빠른 시작

```bash
git clone <이 저장소> ~/go2-nav
cd ~/go2-nav
bash install.sh
```

이후 [docs/02-mapping.md](docs/02-mapping.md) → [docs/03-navigation.md](docs/03-navigation.md) 순서.

---

## 문서

| 파일 | 내용 |
|---|---|
| [docs/01-measurements.md](docs/01-measurements.md) | **실측 데이터 전문** — 모든 파라미터의 근거 |
| [docs/02-mapping.md](docs/02-mapping.md) | 매핑 절차 (터미널별 명령 포함) |
| [docs/03-navigation.md](docs/03-navigation.md) | 자율주행 & Point-to-Point (터미널별 명령 포함) |
| [docs/04-parameters.md](docs/04-parameters.md) | 전체 파라미터 표 + 튜닝 방법 |
| [docs/05-troubleshooting.md](docs/05-troubleshooting.md) | 실제로 겪은 문제와 해결 |
| [docs/06-cad.md](docs/06-cad.md) | CAD 도면 활용 (검증/보정용) |

---

## 달성 결과

| 지표 | 값 | 판정 |
|---|---|---|
| 스캔 유효 빔 비율 | 51.7% (180빔 중 93) | 양호 |
| 맵 벽 평균 이웃수 | 2.51 | 얇고 깔끔 (이중벽 없음) |
| 회전 시 벽 겹침 | 한 겹 유지 | 통과 |
| TF `base_footprint` z | 0.000 고정 | 중력 정렬 정상 |

---

## 라이선스

MIT
