#!/usr/bin/env python3
"""높이 밴드 x 누적 프레임 x 빔폭 조합별 각도 커버리지 측정.

사용:
  AINC=1.0 python3 band_sweep.py
  AINC=1.5 python3 band_sweep.py
  AINC=2.0 python3 band_sweep.py

정지 상태, 기립, 벽이 사방에 보이는 곳에서 실행.
성격이 다른 위치 2~3곳에서 측정해 가장 낮은 값에 맞추는 것을 권장.
"""
import math
import os

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
    p = np.stack([a['x'], a['y'], a['z']], 1).astype(np.float32)
    ok = np.isfinite(p).all(1)
    ok &= ~((np.abs(p[:, 0]) < 1e-6) & (np.abs(p[:, 1]) < 1e-6) & (np.abs(p[:, 2]) < 1e-6))
    return p[ok]


NFRAME = 30
BANDS = [(-0.02, 0.60), (0.02, 0.60), (0.05, 0.60), (0.08, 0.60), (0.12, 0.60)]
ACCUM = [4, 6, 8, 10]
AINC_DEG = float(os.environ.get('AINC', '2.0'))
AINC = math.radians(AINC_DEG)
NB = int(round(2 * math.pi / AINC))

rclpy.init()
n = Node('sweep')
acc = []
od = [None]
q = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)


def co(m):
    od[0] = m


def cb(m):
    acc.append(xyz(m))
    if len(acc) < NFRAME:
        return
    if od[0] is None:
        return
    p = od[0].pose.pose
    x, y = p.position.x, p.position.y
    qq = p.orientation
    yaw = math.atan2(2 * (qq.w * qq.z + qq.x * qq.y), 1 - 2 * (qq.y * qq.y + qq.z * qq.z))
    c, s = math.cos(-yaw), math.sin(-yaw)

    print("\n빔 %d개 (%.1fdeg) 기준 각도 커버리지 [%%]" % (NB, AINC_DEG))
    print("  밴드(m)          " + "".join("  %2df" % a for a in ACCUM))
    for lo, hi in BANDS:
        row = "  [%+.2f, %.2f]   " % (lo, hi)
        for a in ACCUM:
            P = np.concatenate(acc[-a:])
            dx, dy = P[:, 0] - x, P[:, 1] - y
            bx, by = c * dx - s * dy, s * dx + c * dy
            mz = (P[:, 2] > lo) & (P[:, 2] < hi)
            r = np.hypot(bx, by)
            mm = mz & (r > 0.30) & (r < 15.0)
            if mm.sum() == 0:
                row += "   0"
                continue
            ang = np.arctan2(by[mm], bx[mm])
            idx = np.clip(((ang + math.pi) / AINC).astype(int), 0, NB - 1)
            row += " %3.0f" % (100 * len(np.unique(idx)) / NB)
        print(row)
    print("\n※ 40%% 이상 실용, 55%% 이상 우수")
    print("※ 같은 커버리지면 밴드 하한이 높고(바닥 여유) 누적이 짧은 쪽 선택")
    raise SystemExit


n.create_subscription(Odometry, '/utlidar/robot_odom', co, q)
n.create_subscription(PointCloud2, '/utlidar/cloud_deskewed', cb, q)
try:
    rclpy.spin(n)
except SystemExit:
    pass
