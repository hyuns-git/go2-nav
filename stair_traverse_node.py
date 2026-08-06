#!/usr/bin/env python3
"""
stair_traverse_node.py (v2 — 꺾인 계단 / 중간 착지참 대응)

go2-nav 확장: 9단 -> 평지참(유턴) -> 5단 처럼 중간에 방향전환 착지참이 있는
계단을 처리한다. 단순히 "pitch가 평평해지면 끝"으로 판단하면 중간 착지참에서
오탐(false completion)이 나므로, 계단 구조를 "구간(segment) 리스트"로 명시하고
각 구간을 순서대로 실행한다.

구간 종류:
  - climb : 전진하며 오름. pitch가 임계값 이상으로 올라갔다가 다시 임계값
            이하로 level_hold_sec 이상 유지되면 그 구간 종료.
  - turn  : 제자리에서 회전. yaw 변화량을 누적해서 목표 각도에 도달하면 종료.

전제 (이전 버전과 동일):
  - "/api/sport/request" 로 unitree_api/msg/Request 퍼블리시 (SportClient API 호출)
  - "/utlidar/robot_odom" (nav_msgs/Odometry) 에서 IMU 쿼터니언으로 pitch/yaw 계산

사용 예 (9단 -> 180도 유턴 -> 5단):
  ros2 run go2_nav_bridge stair_traverse_node --ros-args \
    -p flight_step_counts:="[9, 5]" \
    -p turn_angles_deg:="[180.0]" \
    -p turn_direction:=1.0 \
    -p seconds_per_step:=1.2 \
    -p forward_speed:=0.25 \
    -p turn_speed:=0.4

주의:
  - turn_direction: +1.0 = 반시계, -1.0 = 시계. 실제 계단에서 유턴 방향에 맞게 설정.
  - flight_step_counts와 turn_angles_deg의 길이 관계: len(turn_angles_deg) ==
    len(flight_step_counts) - 1 이어야 한다 (마지막 flight 뒤에는 turn 없음).
  - seconds_per_step은 실측(계단 한 단 오르는 데 forward_speed 기준 걸리는 시간)
    으로 채운다. 이 값 * 단수 로 각 flight의 최소 소요시간(min_climb_sec)을
    자동 계산해서 착지참에서의 조기 완료 오탐을 줄인다.
"""

import json
import time
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from unitree_api.msg import Request
from nav_msgs.msg import Odometry


# ---- SportClient API ID (Motion Services Interface V2.0 기준) ----
API_STOPMOVE = 1003
API_MOVE = 1008
API_SPEEDLEVEL = 1015
API_BALANCESTAND = 1002
API_FREEAVOID = 2048
API_CLASSICWALK = 2049


def normalize_angle(a: float) -> float:
    """[-pi, pi] 로 정규화"""
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


def quat_to_pitch_yaw(q):
    # pitch (y축 회전)
    sinp = 2.0 * (q.w * q.y - q.z * q.x)
    sinp = max(-1.0, min(1.0, sinp))
    pitch = math.asin(sinp)
    # yaw (z축 회전)
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return math.degrees(pitch), yaw


class StairTraverseNode(Node):
    def __init__(self):
        super().__init__("stair_traverse_node")

        # ---- 계단 구조 (실측 기반) ----
        self.declare_parameter("flight_step_counts", [9, 5])
        self.declare_parameter("turn_angles_deg", [180.0])
        self.declare_parameter("turn_direction", 1.0)  # +1 CCW, -1 CW

        # ---- 속도/임계값 (실측 후 보정) ----
        self.declare_parameter("forward_speed", 0.25)       # m/s
        self.declare_parameter("turn_speed", 0.4)            # rad/s
        self.declare_parameter("seconds_per_step", 1.2)      # 실측값으로 교체
        self.declare_parameter("pitch_climb_deg", 8.0)
        self.declare_parameter("pitch_level_deg", 3.0)
        self.declare_parameter("level_hold_sec", 2.0)
        self.declare_parameter("min_climb_margin", 0.5)      # min_climb_sec = steps*sec_per_step*margin
        self.declare_parameter("turn_tolerance_deg", 5.0)
        self.declare_parameter("max_duration_sec", 90.0)     # 전체 시퀀스 안전 타임아웃

        self.declare_parameter("odom_topic", "/utlidar/robot_odom")
        self.declare_parameter("request_topic", "/api/sport/request")

        g = self.get_parameter
        self.flight_step_counts = g("flight_step_counts").value
        self.turn_angles_deg = g("turn_angles_deg").value
        self.turn_direction = g("turn_direction").value
        self.forward_speed = g("forward_speed").value
        self.turn_speed = g("turn_speed").value
        self.seconds_per_step = g("seconds_per_step").value
        self.pitch_climb_deg = g("pitch_climb_deg").value
        self.pitch_level_deg = g("pitch_level_deg").value
        self.level_hold_sec = g("level_hold_sec").value
        self.min_climb_margin = g("min_climb_margin").value
        self.turn_tolerance_deg = g("turn_tolerance_deg").value
        self.max_duration_sec = g("max_duration_sec").value

        if len(self.turn_angles_deg) != len(self.flight_step_counts) - 1:
            raise ValueError(
                "turn_angles_deg 길이는 flight_step_counts 길이 - 1 이어야 합니다."
            )

        odom_topic = g("odom_topic").value
        request_topic = g("request_topic").value

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT,
                          history=HistoryPolicy.KEEP_LAST)

        self.req_pub = self.create_publisher(Request, request_topic, 10)
        self.odom_sub = self.create_subscription(Odometry, odom_topic, self._odom_cb, qos)

        self.pitch_deg = 0.0
        self.yaw_rad = 0.0
        self._got_odom = False

        self._seq_start_time = None

    # ---------- API 호출 ----------
    def publish_api(self, api_id: int, param: dict):
        msg = Request()
        msg.header.identity.api_id = api_id
        msg.parameter = json.dumps(param)
        self.req_pub.publish(msg)

    def _odom_cb(self, msg: Odometry):
        q = msg.pose.pose.orientation
        pitch_deg, yaw = quat_to_pitch_yaw(q)
        self.pitch_deg = pitch_deg
        self.yaw_rad = yaw
        self._got_odom = True

    def _wait_odom(self, timeout=5.0):
        t0 = time.time()
        while rclpy.ok() and not self._got_odom:
            rclpy.spin_once(self, timeout_sec=0.2)
            if time.time() - t0 > timeout:
                return False
        return True

    def _global_timeout_hit(self):
        return (time.time() - self._seq_start_time) > self.max_duration_sec

    # ---------- 구간 실행: 오름 ----------
    def _run_climb_segment(self, steps: int, idx: int):
        min_climb_sec = max(2.0, steps * self.seconds_per_step * self.min_climb_margin)
        self.get_logger().info(
            f"[flight {idx}] {steps}단 오름 시작 (min_climb_sec={min_climb_sec:.1f}s)"
        )

        start_time = time.time()
        climbing_detected = False
        level_since = None

        while rclpy.ok():
            self.publish_api(API_MOVE, {"x": self.forward_speed, "y": 0.0, "z": 0.0})
            rclpy.spin_once(self, timeout_sec=0.1)

            elapsed = time.time() - start_time
            abs_pitch = abs(self.pitch_deg)

            if abs_pitch > self.pitch_climb_deg:
                if not climbing_detected:
                    self.get_logger().info(f"[flight {idx}] 경사 감지 (pitch={self.pitch_deg:.1f}deg)")
                climbing_detected = True
                level_since = None
            elif climbing_detected and abs_pitch < self.pitch_level_deg:
                if level_since is None:
                    level_since = time.time()
                elif (time.time() - level_since) > self.level_hold_sec and elapsed > min_climb_sec:
                    self.get_logger().info(f"[flight {idx}] 평지 도달 - 구간 완료")
                    break

            if self._global_timeout_hit():
                self.get_logger().warn("전체 타임아웃 - 시퀀스 중단")
                return False

        self.publish_api(API_MOVE, {"x": 0.0, "y": 0.0, "z": 0.0})
        time.sleep(0.3)
        return True

    # ---------- 구간 실행: 회전 ----------
    def _run_turn_segment(self, target_deg: float, idx: int):
        self.get_logger().info(f"[turn {idx}] {target_deg:.0f}도 회전 시작")
        target_rad = math.radians(abs(target_deg))
        vyaw = self.turn_direction * self.turn_speed

        self._wait_odom()
        prev_yaw = self.yaw_rad
        turned = 0.0

        while rclpy.ok():
            self.publish_api(API_MOVE, {"x": 0.0, "y": 0.0, "z": vyaw})
            rclpy.spin_once(self, timeout_sec=0.1)

            delta = normalize_angle(self.yaw_rad - prev_yaw)
            turned += abs(delta)
            prev_yaw = self.yaw_rad

            if math.degrees(turned) >= target_deg - self.turn_tolerance_deg:
                self.get_logger().info(f"[turn {idx}] 회전 완료 (약 {math.degrees(turned):.0f}도)")
                break

            if self._global_timeout_hit():
                self.get_logger().warn("전체 타임아웃 - 시퀀스 중단")
                return False

        self.publish_api(API_MOVE, {"x": 0.0, "y": 0.0, "z": 0.0})
        time.sleep(0.5)
        return True

    # ---------- 메인 시퀀스 ----------
    def run(self):
        self._seq_start_time = time.time()

        self.get_logger().info("계단 진입 준비: SpeedLevel(-1), FreeAvoid(off), ClassicWalk(on)")
        self.publish_api(API_SPEEDLEVEL, {"data": -1})
        time.sleep(0.3)
        self.publish_api(API_FREEAVOID, {"data": False})
        time.sleep(0.3)
        self.publish_api(API_CLASSICWALK, {"data": True})
        time.sleep(1.0)

        if not self._wait_odom():
            self.get_logger().error("odom을 못 받음 - odom_topic 파라미터를 확인할 것")
            return False

        ok = True
        for i, steps in enumerate(self.flight_step_counts):
            ok = self._run_climb_segment(steps, i)
            if not ok:
                break
            if i < len(self.turn_angles_deg):
                ok = self._run_turn_segment(self.turn_angles_deg[i], i)
                if not ok:
                    break

        # 정지 및 원래 게이트로 복귀
        self.publish_api(API_MOVE, {"x": 0.0, "y": 0.0, "z": 0.0})
        time.sleep(0.2)
        self.publish_api(API_STOPMOVE, {})
        time.sleep(0.3)
        self.publish_api(API_CLASSICWALK, {"data": False})
        time.sleep(0.3)
        self.publish_api(API_BALANCESTAND, {})

        self.get_logger().info(f"계단 시퀀스 종료. 결과: {'성공' if ok else '중단/실패'}")
        return ok


def main():
    rclpy.init()
    node = StairTraverseNode()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
