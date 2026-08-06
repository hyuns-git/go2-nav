# go2-nav — Unitree Go2 EDU 2D SLAM & Point-to-Point Navigation (ROS 2 Foxy)

Unitree Go2 EDU의 내장 4D LiDAR L1만으로 실내 2D 맵을 만들고, 그 맵 위에서
이름 붙인 지점 사이를 자율 이동(Point-to-Point)시키는 완전한 구성.

여기에 더해, **층간 이동(계단 등반)을 위한 전용 제어 모듈**(`stair_traverse_node`)을
포함합니다. Nav2 경로계획이 닿지 않는 계단 구간(꺾인 착지참 포함)을 별도
스크립트로 처리해, 한 층의 지도에서 다른 층의 목표 지점까지 이어서 이동하는
멀티플로어 시나리오의 기반을 제공합니다.

**모든 파라미터는 실측 데이터에 근거해 결정되었습니다.** 근거는 [docs/01-measurements.md](docs/01-measurements.md)에 전부 기록되어 있습니다.

---

## 검증 환경

| 항목    | 값                                                       |
| ----- | ------------------------------------------------------- |
| 로봇    | Unitree Go2 EDU                                         |
| 컴퓨터   | 확장독 Jetson Orin Nano (`192.168.123.18`, user `unitree`) |
| OS    | Ubuntu 20.04 (JetPack 5), arm64                         |
| ROS   | ROS 2 Foxy                                              |
| DDS   | CycloneDDS (`unitree_ros2`)                             |
| LiDAR | Unitree 4D LiDAR L1 (내장)                                |
| 최초 검증일   | 2026-08-04                                          |
| 계단 등반 모듈 추가 | 2026-08-05 (사무실 외부 복도~계단 입구까지 지도 확장, `stair_traverse_node` 도입) |

---

## 왜 표준 파이프라인을 쓰지 않았는가

일반적인 `pointcloud_to_laserscan` + `slam_toolbox` 조합은 Go2에서 흐릿한 맵을
만듭니다. 실측으로 확인한 원인 3가지:

### 1. 포인트클라우드의 86.5%가 더미다

`/utlidar/cloud_deskewed`는 프레임당 11,554점을 보내지만, 그중 **10,000점이 정확히 `(0,0,0)`** 입니다. L1이 "반사 없음"을 0으로 채워
보내기 때문입니다. `isfinite()` 검사로는 걸러지지 않습니다.

이 점들은 odom 원점에 고정되어 있으므로, 로봇이 이동하면 **출발점에 12만 점짜리 유령 장애물**이 생깁니다.

→ 해결: `cloud_to_xyz()`에서 `(0,0,0)` 명시적 제거

### 2. 4족보행의 pitch/roll이 스캔에 실린다

`base_link`는 몸통 고정 프레임이라 걸을 때 ±3~7° 흔들립니다.
pitch 5°면 5m 앞 벽에서 높이 슬라이스가 44cm 밀립니다.

→ 해결: `odom → base_footprint(yaw만) → base_link` 3단 프레임 도입.
SLAM과 Nav2의 기준을 `base_footprint`로 통일.

**주의**: 이 보정은 평지 보행 중 ±3~7° 수준의 흔들림을 전제로 합니다.
실제 계단(지속적으로 20~30°+ 기울어짐)에서는 이 전제가 깨지므로, 계단 구간은
SLAM/Nav2 파이프라인 밖에서 `stair_traverse_node`가 별도로 처리합니다
([docs/07-stair-crossing.md](docs/07-stair-crossing.md) 참조).

### 3. 타임스탬프가 두 시계에 걸쳐 있다

메인 컨트롤 보드(LiDAR 발행)와 확장독 컴퓨터의 시계가 **실측 -49.13초** 차이납니다. 클라우드 원본 스탬프로 TF를 조회하면
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


   ── 계단 구간 (지도/Nav2 경로계획 밖에서 별도 처리) ──

   waypoint_tool go stair_entry   (Nav2로 계단 입구까지 자율 이동)
            │
            ▼
   stair_traverse_node
     · FreeAvoid(false) + ClassicWalk(true)
     · Move()를 10Hz로 재발행하며 IMU pitch로 각 계단 구간
       (climb) 시작/종료 자동 감지
     · 착지참에서는 yaw 변화량을 누적해 제자리 회전(turn) 구간 처리
     · flight_step_counts / turn_angles_deg로 여러 구간(꺾인 계단) 지원
            │
            ▼
   다음 층 도착 (다음 층 지도가 있으면 map 전환 + AMCL 재초기화로 이어서 자율주행)
```

`pointcloud_to_laserscan`은 사용하지 않습니다.

---

## 빠른 시작

```bash
git clone <이 저장소> ~/go2-nav
cd ~/go2-nav
bash install.sh
```

이후 [docs/02-mapping.md](docs/02-mapping.md) → [docs/03-navigation.md](docs/03-navigation.md) → [docs/07-stair-crossing.md](docs/07-stair-crossing.md) 순서.

---

## 문서

| 파일                                                                                    | 내용                                 |
| ------------------------------------------------------------------------------------- | ---------------------------------- |
| [docs/01-measurements.md](docs/01-measurements.md)       | **실측 데이터 전문** — 모든 파라미터의 근거        |
| [docs/02-mapping.md](docs/02-mapping.md)                 | 매핑 절차 (터미널별 명령 포함)                 |
| [docs/03-navigation.md](docs/03-navigation.md)           | 자율주행 & Point-to-Point (터미널별 명령 포함) |
| [docs/04-parameters.md](docs/04-parameters.md)           | 전체 파라미터 표 + 튜닝 방법 (stair_traverse_node 포함) |
| [docs/05-troubleshooting.md](docs/05-troubleshooting.md) | 실제로 겪은 문제와 해결 (AMCL 수렴/RViz 렌더링 포함) |
| [docs/06-cad.md](docs/06-cad.md)                         | CAD 도면 활용 (검증/보정용)                 |
| [docs/07-stair-crossing.md](docs/07-stair-crossing.md)   | **신규** — 계단(꺾인 착지참 포함) 자동 등반 절차 및 `stair_traverse_node` 사용법 |

---

## 달성 결과

| 지표                    | 값                 | 판정             |
| --------------------- | ----------------- | -------------- |
| 스캔 유효 빔 비율            | 51.7% (180빔 중 93) | 양호             |
| 맵 벽 평균 이웃수 (사무실 최초 맵) | 2.51              | 얇고 깔끔 (이중벽 없음) |
| 맵 벽 평균 이웃수 (복도~계단 입구 확장 맵) | 2.75          | 양호 (기준 3.0 미만) |
| 회전 시 벽 겹침             | 한 겹 유지            | 통과             |
| TF `base_footprint` z | 0.000 고정          | 중력 정렬 정상       |
| 계단 등반 (컨트롤러 수동 조작)     | 9단 + 착지참 유턴 180° + 5단, 3→4층 성공 | 검증 완료 (자동화는 진행 중) |

---

## 현재 진행 상황 (2026-08-05 기준)

- [x] 사무실 내부 2D 지도 생성 및 Point-to-Point 자율주행
- [x] 사무실 외부(복도~엘리베이터 로비~계단 입구)까지 지도 확장
- [x] 계단실을 Nav2 금지구역으로 처리 (경로계획이 계단으로 들어가지 않도록)
- [x] `stair_entry` 웨이포인트 등록 및 `stair_traverse_node`(꺾인 계단 대응) 개발
- [x] 확장 지도에서 AMCL 위치추정 안정화
- [ ] `office_start → stair_entry` 자율주행 + `stair_traverse_node` 연계 end-to-end 테스트
- [ ] 4층 지도 생성 및 층간 맵 전환(AMCL 재초기화) 자동화

---

## 라이선스

MIT
