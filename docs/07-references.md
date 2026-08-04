# 참고 자료

## Unitree 공식

- unitree_ros2 (Go2/B2 ROS2 인터페이스): https://github.com/unitreerobotics/unitree_ros2
- point_lio_unilidar (L1 전용 Point-LIO): https://github.com/unitreerobotics/point_lio_unilidar
- Unitree GitHub: https://github.com/unitreerobotics
- 4D LiDAR L1 제품 페이지: https://www.unitree.com/mobile/LiDAR/
- L1 사용자 매뉴얼: https://oss-global-cdn.unitree.com/static/52b72f707b304d229d4321eea223738f.pdf

### L1 주요 사양

| 항목 | 값 |
|---|---|
| 유효 샘플링 | 21,600 points/sec |
| 방위각 스캔 | 11 Hz |
| 수직 스캔 | 180 Hz |
| FOV | 360° × 90° |
| 측정 거리 | 0.05 ~ 30 m (@90% 반사율) |
| 거리 정확도 | ±2.0 cm |
| 스캔 방식 | 비반복(non-repetitive) |

**비반복 스캔이라 단일 프레임만으로는 2D 슬라이스가 성립하지 않습니다.**
Velodyne 같은 반복 스캔은 매 프레임 같은 링을 훑지만, L1은 매번 다른 곳을
훑습니다. 그래서 누적이 필수입니다.

## Go2 커뮤니티 (동일 환경 검증됨)

- **go2_ros2_toolbox** — Go2 EDU 확장독 / Ubuntu 20.04 / Foxy / 펌웨어
  v1.1.7에서 테스트됨. 포인트클라우드 누적(`/trans_cloud`), SLAM Toolbox 연동,
  Nav2 통합, 맵 직렬화 포함. **누적이 필수라는 결론의 교차 검증**:
  https://github.com/andy-zhuo-02/go2_ros2_toolbox
- Go2_where_r_u — Go2 + Livox MID-360 2D SLAM. QoS 릴레이 유틸 참고:
  https://github.com/arpa-byte/Go2_where_r_u
- OpenMind Go2 토픽 목록: https://docs.openmind.org/robotics/unitree_go2_quadruped

## ROS 2 / SLAM

- slam_toolbox: https://github.com/SteveMacenski/slam_toolbox
  - 벤치마크: 30,000 sq.ft.까지 5배속, 60,000 sq.ft.까지 3배속.
    최대 사례는 200,000 sq.ft. 건물을 **동기 모드**로 매핑
- 기본 파라미터: https://github.com/SteveMacenski/slam_toolbox/blob/ros2/config/mapper_params_online_async.yaml
- Nav2 Mapping & Localization: https://docs.nav2.org/setup_guides/sensors/mapping_localization.html
- Husarion SLAM 튜토리얼: https://husarion.com/tutorials/ros2-tutorials/8-slam/

## 논문

- 실내 2D SLAM 루프 클로저 파라미터 최적화 (맵 품질 정량 평가 지표 3종:
  점유 격자 비율, 코너 개수, 폐쇄 영역): https://doi.org/10.3390/s20071906
- BIM/CAD 기반 위치추정 정량 비교 (AMCL vs 포즈그래프): https://arxiv.org/pdf/2308.05443
- Go2 EDU + ROS2 Foxy + RTAB-Map + Nav2 구축 사례: https://arxiv.org/pdf/2512.13974
- 4족 로봇 ROS2 항법 플랫폼 (Spot, SLAM Toolbox + Nav2 + frontier exploration,
  도달 불가능 frontier 처리): https://doi.org/10.3390/robotics15040070

## 자율 탐사 (선택 — Nav2 검증 후)

- nav2_wfd (Foxy 호환): https://github.com/SeanReg/nav2_wavefront_frontier_exploration
- m-explore-ros2 (Humble): https://github.com/robo-friends/m-explore-ros2

**주의**: 자율 탐사는 커버리지 속도를 최적화하며 맵 품질을 최적화하지 않습니다.
시작점 재방문, 양방향 루프, 코너 정지를 하지 않으므로 **최초 맵 제작에는
수동 조종이 낫습니다.** 탐사는 Nav2가 검증된 뒤에 시도하세요.

## 업그레이드 경로

| 방법 | 설명 |
|---|---|
| Point-LIO (Unitree 공식) | L1 + 내장 IMU만으로 SLAM. 드리프트가 내장 오도메트리보다 훨씬 작음. 이걸로 만든 odom을 slam_toolbox에 먹이면 2D 맵 품질 급상승 |
| RTAB-Map | Go2 EDU + Foxy 조합에서 논문 검증됨. icp_odometry + 카메라 RGB-D 융합 |
| 3D → 2D 투영 | 3D LIO로 3D 맵 후 특정 높이 슬라이스를 2D로 변환 |
