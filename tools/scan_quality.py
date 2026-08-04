#!/usr/bin/env python3
"""/scan 의 유효 빔 비율 측정. scan_maker 실행 중에 사용.

사용: python3 scan_quality.py [샘플수]
"""
import math
import sys
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

N = int(sys.argv[1]) if len(sys.argv) > 1 else 30
rclpy.init()
n = Node('sq')
s = []


def cb(m):
    v = [r for r in m.ranges if math.isfinite(r)]
    s.append((len(m.ranges), len(v), min(v) if v else 0, max(v) if v else 0))
    if len(s) >= N:
        b = s[0][0]
        a = sum(x[1] for x in s) / len(s)
        print("빔 %d / 유효 평균 %.0f (%.1f%%) / 거리 %.2f~%.2f m"
              % (b, a, 100 * a / b, min(x[2] for x in s), max(x[3] for x in s)))
        p = 100 * a / b
        print("판정:", "우수" if p >= 55 else "양호" if p >= 40 else "부족")
        raise SystemExit


n.create_subscription(LaserScan, '/scan', cb, 10)
try:
    rclpy.spin(n)
except SystemExit:
    pass
