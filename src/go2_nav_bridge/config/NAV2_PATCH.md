# nav2_go2.yaml 만드는 법

Foxy 기본 파일을 복사한 뒤 아래 패치를 적용합니다.
(직접 작성하면 Foxy 비호환 키가 섞이기 쉬움)

```bash
cp /opt/ros/foxy/share/nav2_bringup/params/nav2_params.yaml \
   ~/ros2_ws/src/go2_nav_bridge/config/nav2_go2.yaml
cd ~/ros2_ws/src/go2_nav_bridge/config
```

## 1. 프레임 · 시간 (전역 치환)

```bash
sed -i 's/base_link/base_footprint/g' nav2_go2.yaml
sed -i 's/use_sim_time: True/use_sim_time: False/g' nav2_go2.yaml
sed -i 's/map_subscribe_transient_local: False/map_subscribe_transient_local: True/g' nav2_go2.yaml
```

`map_subscribe_transient_local`이 False면 costmap이 맵을 못 받습니다.

## 2. 원본 값 확인 (버전마다 다름)

```bash
grep -n "robot_radius\|footprint\|inflation_radius\|max_vel\|min_vel\|max_speed_xy\|acc_lim\|decel_lim\|laser_m\|controller_frequency\|tolerance\|particles\|alpha[1-4]\|max_beams\|allow_unknown\|expected_planner_frequency\|rotational_acc_lim" nav2_go2.yaml
```

## 3. 값 적용 (grep 결과에 맞게 좌변 조정)

```bash
# 라이다 실측 범위
sed -i 's/laser_max_range: 100.0/laser_max_range: 10.0/' nav2_go2.yaml
sed -i 's/laser_min_range: -1.0/laser_min_range: 0.30/' nav2_go2.yaml

# 속도 — max_vel_x 와 max_speed_xy 를 반드시 함께
sed -i 's/max_vel_x: 0.26/max_vel_x: 0.30/' nav2_go2.yaml
sed -i 's/max_speed_xy: 0.26/max_speed_xy: 0.30/' nav2_go2.yaml
sed -i 's/max_vel_theta: 1.0/max_vel_theta: 0.50/' nav2_go2.yaml

# 가속도 — 너무 낮으면 DWB 예측을 못 따라감
sed -i 's/acc_lim_x: 2.5/acc_lim_x: 1.0/' nav2_go2.yaml
sed -i 's/decel_lim_x: -2.5/decel_lim_x: -1.0/' nav2_go2.yaml
sed -i 's/acc_lim_theta: 3.2/acc_lim_theta: 1.2/' nav2_go2.yaml
sed -i 's/decel_lim_theta: -3.2/decel_lim_theta: -1.2/' nav2_go2.yaml

# 목표 허용 오차
sed -i 's/xy_goal_tolerance: 0.25/xy_goal_tolerance: 0.30/g' nav2_go2.yaml
sed -i 's/yaw_goal_tolerance: 0.25/yaw_goal_tolerance: 0.45/' nav2_go2.yaml

# CPU
sed -i 's/controller_frequency: 20.0/controller_frequency: 10.0/' nav2_go2.yaml
sed -i 's/expected_planner_frequency: 20.0/expected_planner_frequency: 2.0/' nav2_go2.yaml

# 미지 영역 통과 허용 (false 면 불완전한 맵에서 경로 생성 실패)
sed -i 's/allow_unknown: false/allow_unknown: true/' nav2_go2.yaml

# 인플레이션
sed -i 's/inflation_radius: 0.55/inflation_radius: 0.45/g' nav2_go2.yaml

# AMCL — 보행 로봇 노이즈 + 희박한 스캔
sed -i 's/alpha1: 0.2/alpha1: 0.30/; s/alpha2: 0.2/alpha2: 0.30/' nav2_go2.yaml
sed -i 's/alpha3: 0.2/alpha3: 0.30/; s/alpha4: 0.2/alpha4: 0.30/' nav2_go2.yaml
sed -i 's/max_beams: 60/max_beams: 120/' nav2_go2.yaml
sed -i 's/min_particles: 500/min_particles: 800/' nav2_go2.yaml
sed -i 's/max_particles: 2000/max_particles: 3000/' nav2_go2.yaml

# recovery 회전 (기본 3.2 는 Go2 에 위험)
sed -i 's/rotational_acc_lim: 3.2/rotational_acc_lim: 1.2/' nav2_go2.yaml
```

## 4. footprint (수동 편집)

`local_costmap`, `global_costmap` 양쪽에:

```yaml
      footprint: "[ [0.45, 0.25], [0.45, -0.25], [-0.45, -0.25], [-0.45, 0.25] ]"
```

Go2 실제 0.70 x 0.31 m 에 각 방향 10cm 여유.
좁은 문을 못 지나가면 축소하세요.

## 5. planner 확인

```yaml
planner_server:
  ros__parameters:
    expected_planner_frequency: 2.0
    use_sim_time: False
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: "nav2_navfn_planner/NavfnPlanner"   # Foxy 표준. Smac 은 없음
      tolerance: 0.5
      use_astar: false
      allow_unknown: true
```

## 6. recovery — backup 제거 권장

```yaml
    recovery_plugins: ["spin", "wait"]
```

후진은 4족 로봇에서 뒤가 안 보여 위험합니다.

## 7. 최종 확인

```bash
grep -n "base_footprint\|max_vel\|max_speed_xy\|acc_lim\|alpha[1-4]\|particles\|inflation_radius\|allow_unknown\|laser_m\|tolerance" nav2_go2.yaml
```
