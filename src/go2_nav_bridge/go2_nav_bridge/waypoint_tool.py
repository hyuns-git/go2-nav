#!/usr/bin/env python3
"""이름 붙인 지점 저장 / 이동 도구.

  ros2 run go2_nav_bridge waypoint_tool save 회의실A
  ros2 run go2_nav_bridge waypoint_tool list
  ros2 run go2_nav_bridge waypoint_tool go 회의실A
  ros2 run go2_nav_bridge waypoint_tool tour 엘리베이터 회의실A 탕비실

저장 파일: ~/maps/waypoints.yaml (직접 편집 가능)

등록 요령:
  - 벽에서 1m 이상 떨어진 곳 (inflation_radius 0.45 때문에 벽 근처는 도달 불가)
  - 최종 바라볼 방향까지 맞춘 상태에서 저장
"""
import math
import os
import sys

import rclpy
import yaml
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener

from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose

WP_FILE = os.path.expanduser('~/maps/waypoints.yaml')
MAP_FRAME = 'map'
BASE_FRAME = 'base_footprint'


def load_wp():
    if not os.path.exists(WP_FILE):
        return {}
    with open(WP_FILE, 'r') as f:
        return yaml.safe_load(f) or {}


def save_wp(d):
    os.makedirs(os.path.dirname(WP_FILE), exist_ok=True)
    with open(WP_FILE, 'w') as f:
        yaml.safe_dump(d, f, allow_unicode=True, sort_keys=True)


class WpTool(Node):
    def __init__(self):
        super().__init__('waypoint_tool')
        self.buf = Buffer()
        self.tfl = TransformListener(self.buf, self)
        self.ac = ActionClient(self, NavigateToPose, 'navigate_to_pose')

    def current_pose(self, timeout=10.0):
        end = self.get_clock().now() + Duration(seconds=timeout)
        while rclpy.ok() and self.get_clock().now() < end:
            rclpy.spin_once(self, timeout_sec=0.2)
            try:
                tr = self.buf.lookup_transform(MAP_FRAME, BASE_FRAME, rclpy.time.Time())
            except Exception:
                continue
            t, q = tr.transform.translation, tr.transform.rotation
            yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                             1.0 - 2.0 * (q.y * q.y + q.z * q.z))
            return {'x': round(float(t.x), 3),
                    'y': round(float(t.y), 3),
                    'yaw': round(float(yaw), 4)}
        return None

    def navigate(self, wp, name=''):
        if not self.ac.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('navigate_to_pose 서버 없음. Nav2 실행 확인.')
            return False
        goal = NavigateToPose.Goal()
        ps = PoseStamped()
        ps.header.frame_id = MAP_FRAME
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.pose.position.x = float(wp['x'])
        ps.pose.position.y = float(wp['y'])
        ps.pose.orientation.z = math.sin(float(wp['yaw']) * 0.5)
        ps.pose.orientation.w = math.cos(float(wp['yaw']) * 0.5)
        goal.pose = ps

        self.get_logger().info('이동 -> %s (%.2f, %.2f)' % (name, wp['x'], wp['y']))
        fut = self.ac.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, fut)
        gh = fut.result()
        if not gh.accepted:
            self.get_logger().error('목표 거부됨')
            return False
        res_fut = gh.get_result_async()
        rclpy.spin_until_future_complete(self, res_fut)
        status = res_fut.result().status
        ok = (status == 4)   # STATUS_SUCCEEDED
        self.get_logger().info('결과: %s (status=%d)' % ('성공' if ok else '실패', status))
        return ok


def main():
    args = sys.argv[1:]
    if not args:
        print(__doc__)
        return
    cmd = args[0]
    wps = load_wp()

    if cmd == 'list':
        if not wps:
            print('저장된 웨이포인트 없음:', WP_FILE)
        for k, v in sorted(wps.items()):
            print('  %-16s x=%7.2f  y=%7.2f  yaw=%6.2f rad' % (k, v['x'], v['y'], v['yaw']))
        return

    rclpy.init()
    node = WpTool()
    try:
        if cmd == 'save':
            if len(args) < 2:
                print('사용법: save <이름>')
                return
            p = node.current_pose()
            if p is None:
                node.get_logger().error(
                    'map -> base_footprint TF 없음. Nav2/AMCL 및 초기 위치 확인.')
                return
            wps[args[1]] = p
            save_wp(wps)
            print('저장됨: %s -> %s' % (args[1], p))
        elif cmd == 'go':
            if len(args) < 2 or args[1] not in wps:
                print('없는 이름. list 로 확인하세요.')
                return
            node.navigate(wps[args[1]], args[1])
        elif cmd == 'tour':
            names = args[1:]
            missing = [n for n in names if n not in wps]
            if missing:
                print('없는 이름:', missing)
                return
            for n in names:
                if not node.navigate(wps[n], n):
                    print('중단: %s 도달 실패' % n)
                    break
        else:
            print(__doc__)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
