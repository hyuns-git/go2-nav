#!/usr/bin/env python3
"""메인 컨트롤 보드와 확장독 컴퓨터의 시계 차이 측정.

scan_maker 는 클라우드 원본 스탬프를 쓰지 않으므로 결과와 무관하게 동작하지만,
표준 파이프라인이 왜 실패하는지의 근거가 된다.
"""
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2

rclpy.init()
n = Node('clk')
s = []


def cb(m):
    ts = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
    s.append(ts - time.time())
    if len(s) == 10:
        avg = sum(s) / len(s)
        print("offset = %+.4f s   (min %+.4f / max %+.4f)" % (avg, min(s), max(s)))
        print("판정:", "OK" if abs(avg) < 0.05 else "오프셋 큼 (scan_maker 는 무관)")
        raise SystemExit


n.create_subscription(PointCloud2, '/utlidar/cloud_deskewed', cb,
                      QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT))
try:
    rclpy.spin(n)
except SystemExit:
    pass
