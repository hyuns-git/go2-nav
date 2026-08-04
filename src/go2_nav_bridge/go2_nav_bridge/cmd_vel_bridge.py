#!/usr/bin/env python3
"""/cmd_vel -> Go2 sport mode (API 1008 Move) 브릿지.

min_vx / min_vyaw 부스트가 핵심:
  Nav2 DWB 는 목표 근처나 좁은 통로에서 0.02~0.05 m/s 같은 작은 속도를 낸다.
  Go2 sport mode 는 이 정도로는 발을 떼지 않아 "명령은 가는데 안 움직임" 현상이
  발생한다. 최소 속도로 끌어올려 해결.
"""
import json
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from unitree_api.msg import Request

API_ID_STOPMOVE = 1003
API_ID_MOVE = 1008


class CmdVelBridge(Node):
    def __init__(self):
        super().__init__('cmd_vel_bridge')
        p = self.declare_parameter
        p('max_vx', 0.36)        # DWB 0.30 보다 20% 크게 (클램프 빈발 방지)
        p('max_vy', 0.0)
        p('max_vyaw', 0.60)
        p('min_vx', 0.10)        # 부스트 하한
        p('min_vyaw', 0.18)
        p('deadband', 0.02)
        p('watchdog_sec', 0.4)
        p('rate_hz', 20.0)

        g = lambda k: self.get_parameter(k).value
        self.max_vx, self.max_vy, self.max_vyaw = g('max_vx'), g('max_vy'), g('max_vyaw')
        self.min_vx, self.min_vyaw = g('min_vx'), g('min_vyaw')
        self.deadband = g('deadband')
        self.watchdog = g('watchdog_sec')

        self.pub = self.create_publisher(Request, '/api/sport/request', 10)
        self.create_subscription(Twist, '/cmd_vel', self.on_cmd, 10)

        self.vx = self.vy = self.vyaw = 0.0
        self.last_rx = None
        self.stopped = True
        self._id = 0
        self.create_timer(1.0 / g('rate_hz'), self.tick)
        self.get_logger().info('cmd_vel_bridge -> /api/sport/request')

    @staticmethod
    def _clamp(v, lo, hi):
        return max(lo, min(hi, v))

    @staticmethod
    def _boost(v, floor):
        if v == 0.0:
            return 0.0
        if abs(v) < floor:
            return floor if v > 0 else -floor
        return v

    def on_cmd(self, msg):
        self.vx = self._clamp(msg.linear.x, -self.max_vx, self.max_vx)
        self.vy = self._clamp(msg.linear.y, -self.max_vy, self.max_vy)
        self.vyaw = self._clamp(msg.angular.z, -self.max_vyaw, self.max_vyaw)
        self.last_rx = self.get_clock().now()

    def _send(self, api_id, param=''):
        req = Request()
        self._id += 1
        req.header.identity.id = self._id
        req.header.identity.api_id = api_id
        req.header.lease.id = 0
        req.header.policy.priority = 0
        req.header.policy.noreply = True
        req.parameter = param
        self.pub.publish(req)

    def tick(self):
        now = self.get_clock().now()
        stale = (self.last_rx is None or
                 (now - self.last_rx).nanoseconds * 1e-9 > self.watchdog)
        mag = max(abs(self.vx), abs(self.vy), abs(self.vyaw))

        if stale or mag < self.deadband:
            if not self.stopped:
                self._send(API_ID_STOPMOVE)
                self.stopped = True
            return

        vx = self._boost(self.vx, self.min_vx) if abs(self.vx) > self.deadband else 0.0
        vy = self.vy if abs(self.vy) > self.deadband else 0.0
        vz = self._boost(self.vyaw, self.min_vyaw) if abs(self.vyaw) > self.deadband else 0.0

        self._send(API_ID_MOVE, json.dumps(
            {"x": float(vx), "y": float(vy), "z": float(vz)}))
        self.stopped = False


def main():
    rclpy.init()
    node = CmdVelBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node._send(API_ID_STOPMOVE)
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
