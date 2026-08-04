# 전체 파라미터 표와 근거

모든 값의 실측 근거는 [01-measurements.md](01-measurements.md) 참조.

---

## 1. scan_maker (자체 노드)

| 파라미터 | 매핑 | 주행 | 근거 |
|---|---|---|---|
| `accumulate_sec` | **0.75** | 0.55 | 스윕에서 8프레임(0.75s)이 포화점. 10프레임도 53%로 동일. 주행 시에는 반응성 위해 단축 |
| `max_frames` | 10 | 8 | 메모리 상한 |
| `min_height` | **0.05** | 0.05 | 실측 바닥(-0.116~-0.05)에서 10cm 여유. 0.02는 +2%p 얻지만 보행 중 기울면 바닥 오검출 |
| `max_height` | **0.60** | 0.60 | Z p90=0.364. 0.6 위에 점 없음 |
| `angle_increment` | **0.03491** (2.0도) | 0.03491 | 1.0도 대비 +3~6%p. 빈당 점이 여러 개라 노이즈 감소. 180빔은 실내 2D SLAM에 충분 |
| `range_min` | **0.30** | 0.30 | 실측 최소 거리 0.30. 자기 몸통·다리 제거 |
| `range_max` | 12.0 | 12.0 | 실측 최대 7.83m. 여유만 |
| `scan_rate` | 10.0 | 10.0 | 클라우드 11Hz와 정합 |
| `tf_rate` | 50.0 | 50.0 | Nav2 TF 요구 충족 |
| `min_points_per_scan` | 150 | 150 | 불량 스캔 폐기 |

### 누적 시간이 회전 시 번짐을 만들지 않는 이유

일반적인 누적은 센서 프레임에서 하므로 회전 시 번집니다.
**우리는 odom 프레임에서 누적**하므로 점이 월드에 고정됩니다.
로봇이 회전해도 점 위치는 변하지 않습니다.
번짐의 원인은 오도메트리 드리프트와 움직이는 물체뿐입니다.

---

## 2. slam_toolbox (`config/slam_precise.yaml`)

### 프레임·기본

| 파라미터 | 값 | 근거 |
|---|---|---|
| `base_frame` | **`base_footprint`** | `base_link`는 보행 시 pitch/roll이 실림. 중력 정렬 프레임 필요 |
| `resolution` | 0.05 | 5cm/px. 0.03은 노이즈만 증가 |
| `max_laser_range` | **10.0** | 실측 p99=7.46m. 15로 두면 없는 데이터를 기대 |
| `minimum_time_interval` | 0.25 | 스캔 10Hz 대비 적절 |
| `transform_timeout` | 0.4 | Orin Nano CPU 여유 |
| `tf_buffer_duration` | 30.0 | 대규모 최적화 대비 |
| `stack_size_to_use` | 40000000 | serialize 시 필요 |
| `enable_interactive_mode` | true | RViz에서 수동 루프 폐합 가능 |

### 스캔 처리

| 파라미터 | 값 | 근거 |
|---|---|---|
| `minimum_travel_distance` | 0.25 | 노드 과다 생성 방지. **대공간은 0.35로 상향 권장** (방 하나에 posegraph 16MB가 나옴) |
| `minimum_travel_heading` | 0.20 | 회전 오차 보정 빈도 |
| `scan_buffer_size` | 40 | 기본 10 → 확대 |
| `scan_buffer_maximum_scan_distance` | 12.0 | range_max와 정합 |

### 루프 클로저 (대공간용 확대)

| 파라미터 | 기본 | 설정 | 근거 |
|---|---|---|---|
| `loop_search_maximum_distance` | 3.0 | **6.0** | 층 한 바퀴 돌면 드리프트가 3m를 쉽게 초과. 3m 안에서만 후보를 찾으면 폐합 실패 |
| `loop_search_space_dimension` | 8.0 | **12.0** | 루프 탐색 격자. 드리프트 큰 대공간에서 확대 필요 |
| `loop_match_minimum_chain_size` | 10 | **8** | 복도처럼 특징 적은 곳에서 짧은 체인도 시도 |
| `loop_match_minimum_response_coarse` | 0.35 | 0.35 | 1차 통과 임계 |
| `loop_match_minimum_response_fine` | 0.45 | 0.45 | 2차 확정 임계 |
| `correlation_search_space_dimension` | 0.5 | **0.7** | 보행 오도메트리 오차가 바퀴보다 큼 |
| `ceres_loss_function` | None | **HuberLoss** | 잘못된 루프 폐합 1건이 맵 전체를 망가뜨리는 것 방지 |

### sync vs async

**매핑은 `sync_slam_toolbox_node`.** async는 CPU가 밀리면 스캔을 버립니다.
맵을 만드는 단계에서 스캔을 버리면 안 됩니다.
`/scan` hz가 5 이하로 떨어지면 `sync:=false`로 전환하세요.

---

## 3. Nav2 (`config/nav2_go2.yaml`)

### 전역

| 파라미터 | 값 | 근거 |
|---|---|---|
| 모든 `base_link` | → **`base_footprint`** | 프레임 통일 |
| 모든 `use_sim_time` | **False** | 실기 |
| `map_subscribe_transient_local` | **True** | map_server가 Transient Local로 발행. False면 costmap이 맵을 못 받음 |

### AMCL

| 파라미터 | 기본 | 설정 | 근거 |
|---|---|---|---|
| `alpha1~4` | 0.2 | **0.30** | 보행 로봇 오도메트리 노이즈가 바퀴보다 큼 |
| `min_particles` | 500 | **800** | 스캔이 희박(유효 93빔)해 입자를 늘려야 수렴 |
| `max_particles` | 2000 | **3000** | 위와 동일 |
| `max_beams` | 60 | **120** | 180빔 중 120 사용. 희박한 스캔에서 60은 부족 |
| `laser_max_range` | 100.0 | **10.0** | 실측 범위 |
| `laser_min_range` | -1.0 | **0.30** | 자기 몸통 제거 |
| `laser_model_type` | — | `likelihood_field` | 표준 |

### Controller (DWB)

| 파라미터 | 설정 | 근거 |
|---|---|---|
| `controller_frequency` | **10.0** | 기본 20은 Orin Nano에 과함 |
| `max_vel_x` | **0.30** | L1 단계. 사다리로 상향 |
| `max_speed_xy` | **0.30** | ★ max_vel_x와 반드시 동일. 합성 속도 상한이라 이것이 실질 제한 |
| `max_vel_y` | 0.0 | 횡이동은 L3 이후 |
| `max_vel_theta` | **0.50** | 회전이 빠르면 오도메트리 오차 급증 |
| `acc_lim_x` | **1.0** | 0.25면 0.3m/s 도달에 1.2초. DWB 예측을 못 따라감 |
| `acc_lim_theta` | **1.2** | 위와 동일 |
| `decel_lim_x` | **-1.0** | 대칭 |
| `decel_lim_theta` | **-1.2** | 대칭 |
| `xy_goal_tolerance` | **0.30** | 보행 로봇은 정밀 정지가 어려움 |
| `yaw_goal_tolerance` | **0.45** | 위와 동일 |

### Planner (NavFn)

| 파라미터 | 설정 | 근거 |
|---|---|---|
| `plugin` | `nav2_navfn_planner/NavfnPlanner` | Foxy 표준. Smac은 Foxy에 없음 |
| `expected_planner_frequency` | **2.0** | 기본 20은 과함. 전역 경로는 자주 안 바뀜 |
| `allow_unknown` | **true** | ★ false면 미지 영역 통과 경로를 아예 안 만듦. 불완전한 맵에서 "경로 없음" 빈발 |
| `tolerance` | 0.5 | 목표 도달 불가 시 대체점 반경 |
| `use_astar` | false | Dijkstra가 더 매끄러움 |

### Costmap

| 파라미터 | 설정 | 근거 |
|---|---|---|
| `footprint` | `[[0.45,0.25],[0.45,-0.25],[-0.45,-0.25],[-0.45,0.25]]` | Go2 실제 0.70x0.31m에 각 방향 10cm 여유 |
| `inflation_radius` | **0.45** | 기본 0.55는 과함. 좁은 문 통과 실패 시 0.35로 |
| `cost_scaling_factor` | 3.0 | 표준 |
| `resolution` | 0.05 | 맵과 동일 |

### Recovery

| 파라미터 | 설정 | 근거 |
|---|---|---|
| `rotational_acc_lim` | **1.2** | 기본 3.2는 Go2에 위험 |
| `max_rotational_vel` | 0.6 | 기본 1.0 하향 |
| `recovery_plugins` | `["spin","wait"]` 권장 | **`backup`(후진)은 뒤가 안 보여 위험**. 좁은 공간 테스트 시 제외 |

---

## 4. cmd_vel_bridge

| 파라미터 | 값 | 근거 |
|---|---|---|
| `max_vx` | 0.36 | DWB의 0.30보다 20% 크게. 같으면 클램프가 자주 걸려 DWB 예측과 어긋남 |
| `max_vyaw` | 0.60 | 위와 동일 |
| `min_vx` | **0.10** | ★ Nav2는 목표 근처에서 0.02~0.05를 냄. Go2 sport mode는 이 정도로 발을 안 뗌. "명령은 가는데 안 움직임" 현상 방지 |
| `min_vyaw` | **0.18** | 위와 동일 |
| `watchdog_sec` | 0.4 | 명령이 끊기면 0.4초 내 STOPMOVE |

---

## 5. 증상 → 첫 조치

| 증상 | 첫 조치 |
|---|---|
| 스캔이 희박 | `accumulate_sec` 상향 |
| 회전 시 스캔 번짐 | `accumulate_sec` 하향 |
| 벽이 두 겹 | `loop_search_maximum_distance` 6 → 8 |
| 맵이 갑자기 찌그러짐 | `loop_match_minimum_response_fine` 0.45 → 0.55 |
| 바닥이 장애물로 잡힘 | `min_height` 상향 |
| 위치가 튐 | AMCL `alpha1~4` 하향 |
| 안 움직임 | bridge `min_vx` 상향 |
| 좌우 진동 | DWB `PathAlign.scale` 하향, `sim_time` 상향 |
| 좁은 문 못 감 | `inflation_radius` 0.45 → 0.35 |
| 벽에 붙어 감 | `BaseObstacle.scale` 상향 |
| 목표에서 뱅뱅 | `yaw_goal_tolerance` 상향 |
| 경로 없음 | `allow_unknown: true`, planner `tolerance` 상향 |
| CPU 100% | costmap 축소, `max_particles` 하향, async 전환 |
| recovery 남발 | `movement_time_allowance` 상향 + 근본원인 조사 |

---

## 6. 실행 중 파라미터 변경 (재시작 없이)

```bash
ros2 param get /controller_server FollowPath.max_vel_x
ros2 param set /controller_server FollowPath.max_vel_x 0.4
ros2 param set /controller_server FollowPath.max_speed_xy 0.4
ros2 param set /amcl alpha1 0.2
ros2 param set /local_costmap/local_costmap inflation_layer.inflation_radius 0.35
```

**재시작하면 사라집니다.** 좋은 값을 찾으면 yaml에 반영하세요.

---

## 7. 튜닝 3원칙

1. **한 번에 하나만** 바꾼다
2. **바꾸기 전 값을 기록**한다
3. **같은 테스트를 3회** 반복한다 (로봇 동작은 확률적)

기록 양식:
```
[시각] 파라미터  이전값 → 새값
  테스트: (무엇을)
  결과: 1회 __ / 2회 __ / 3회 __
  판정: 개선 / 악화 / 무변화
```
