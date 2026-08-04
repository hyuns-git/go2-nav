#!/usr/bin/env python3
"""포인트클라우드 구조 · 더미 비율 · Z/거리 분포 측정.

로봇을 평평한 바닥에 기립시키고, 주변 2~3m 에 벽이 보이는 곳에서 실행.
결과로 min_height / max_height 를 결정한다.
"""
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry

T = {1: np.int8, 2: np.uint8, 3: np.int16, 4: np.uint16,
     5: np.int32, 6: np.uint32, 7: np.float32, 8: np.float64}


def xyz(m):
    o = {f.name: (f.offset, T[f.datatype]) for f in m.fields if f.name in ('x', 'y', 'z')}
    dt = np.dtype({'names': ['x', 'y', 'z'],
                   'formats': [o['x'][1], o['y'][1], o['z'][1]],
                   'offsets': [o['x'][0], o['y'][0], o['z'][0]],
                   'itemsize': m.point_step})
    a = np.frombuffer(m.data, dtype=dt, count=m.width * m.height)
    return np.stack([a['x'], a['y'], a['z']], 1).astype(np.float32)


rclpy.init()
n = Node('inspect')
acc = []
od = [None]
hdr = [None]
q = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)


def co(m):
    od[0] = m


def cb(m):
    if hdr[0] is None:
        hdr[0] = 1
        print("=== PointCloud2 구조 ===")
        print("  frame_id  :", m.header.frame_id)
        print("  w x h     : %d x %d = %d" % (m.width, m.height, m.width * m.height))
        print("  point_step: %d   is_dense: %s" % (m.point_step, m.is_dense))
        for f in m.fields:
            print("  field %-12s offset=%2d type=%d" % (f.name, f.offset, f.datatype))
    acc.append(xyz(m))
    if len(acc) < 12:
        return

    P = np.concatenate(acc)
    tot = len(P)
    fin = np.isfinite(P).all(1)
    zero = (np.abs(P[:, 0]) < 1e-6) & (np.abs(P[:, 1]) < 1e-6) & (np.abs(P[:, 2]) < 1e-6)
    print("\n=== 포인트 분류 (%d 프레임, 총 %d점) ===" % (len(acc), tot))
    print("  NaN/Inf     : %7d (%.1f%%)" % ((~fin).sum(), 100 * (~fin).sum() / tot))
    print("  정확히 0,0,0: %7d (%.1f%%)  <- 더미" % (zero.sum(), 100 * zero.sum() / tot))
    V = P[fin & ~zero]
    print("  유효        : %7d (%.1f%%)  = 프레임당 %.0f점"
          % (len(V), 100 * len(V) / tot, len(V) / len(acc)))

    if len(V) < 100:
        print("!! 유효점 부족")
        raise SystemExit

    z = V[:, 2]
    r = np.hypot(V[:, 0], V[:, 1])
    print("\n=== 유효점 Z 분포 (odom frame) ===")
    for p in (0, 1, 2, 5, 10, 25, 50, 75, 90, 95, 99, 100):
        print("  p%-3d  %+.3f m" % (p, np.percentile(z, p)))
    print("\n=== 수평거리 분포 ===")
    for p in (0, 1, 5, 50, 95, 99, 100):
        print("  p%-3d  %.2f m" % (p, np.percentile(r, p)))

    near = z[(r > 0.5) & (r < 3.0)]
    if len(near) > 50:
        lo = float(np.percentile(near, 2))
        print("\n  바닥 추정 (0.5<r<3.0m, p2): %+.3f m" % lo)
        print(">>> min_height: %+.2f   max_height: %+.2f" % (lo + 0.20, lo + 0.95))
    if od[0]:
        p = od[0].pose.pose
        print("\n  robot_odom  x=%.3f y=%.3f z=%.3f"
              % (p.position.x, p.position.y, p.position.z))
        print("  (기립 시 z=0.32 부근이면 odom z=0 이 바닥면)")
    raise SystemExit


n.create_subscription(Odometry, '/utlidar/robot_odom', co, q)
n.create_subscription(PointCloud2, '/utlidar/cloud_deskewed', cb, q)
try:
    rclpy.spin(n)
except SystemExit:
    pass
