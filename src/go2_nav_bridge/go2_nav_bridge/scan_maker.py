#!/usr/bin/env python3
"""Go2 EDU 스캔 생성기 (실측 튜닝 반영판).

설계 근거는 docs/01-measurements.md 참조.

핵심 3가지:
  1. /utlidar/cloud_deskewed 의 86.5% 가 (0,0,0) 더미 -> 명시적 제거
  2. 클라우드가 이미 odom 프레임 -> odom 에서 누적 후 현재 yaw 로 일괄 투영
     (TF 조회가 없으므로 메인보드-확장독 시계 차이 49초가 무해)
  3. base_footprint (yaw only, z=0) 도입 -> 보행 pitch/roll 이 스캔에 안 실림

프레임:
  odom -> base_footprint : (x, y, 0, yaw)        중력 정렬. SLAM/Nav2 기준
  base_footprint -> base_link : (0, 0, z, r, p)  시각화용
"""
import math
from collections import deque

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from sensor_msgs.msg import PointCloud2, LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

_NP = {1: np.int8, 2: np.uint8, 3: np.int16, 4: np.uint16,
       5: np.int32, 6: np.uint32, 7: np.float32, 8: np.float64}


def cloud_to_xyz(msg):
    """PointCloud2 -> (N,3) float32. 필드 순서/패딩 무관. 더미 제거 포함."""
    o = {}
    for f in msg.fields:
        if f.name in ('x', 'y', 'z'):
            o[f.name] = (f.offset, _NP[f.datatype])
    if len(o) != 3:
        return np.empty((0, 3), dtype=np.float32)
    dt = np.dtype({
        'names': ['x', 'y', 'z'],
        'formats': [o['x'][1], o['y'][1], o['z'][1]],
        'offsets': [o['x'][0], o['y'][0], o['z'][0]],
        'itemsize': msg.point_step,
    })
    n = msg.width * msg.height
    a = np.frombuffer(msg.data, dtype=dt, count=n)
    p = np.empty((n, 3), dtype=np.float32)
    p[:, 0] = a['x']
    p[:, 1] = a['y']
    p[:, 2] = a['z']

    ok = np.isfinite(p).all(axis=1)
    # Go2 L1 은 무효 반환을 NaN 이 아니라 (0,0,0) 으로 채워 보낸다 (전체의 ~86%).
    # 이 점들은 odom 원점에 고정되므로 제거하지 않으면 출발점에 유령 장애물이 생긴다.
    ok &= ~((np.abs(p[:, 0]) < 1e-6) &
            (np.abs(p[:, 1]) < 1e-6) &
            (np.abs(p[:, 2]) < 1e-6))
    return p[ok]


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def rp_of(q):
    sr = 2.0 * (q.w * q.x + q.y * q.z)
    cr = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
    sp = max(-1.0, min(1.0, 2.0 * (q.w * q.y - q.z * q.x)))
    return math.atan2(sr, cr), math.asin(sp)


class ScanMaker(Node):
    def __init__(self):
        super().__init__('go2_scan_maker')
        p = self.declare_parameter
        p('cloud_topic', '/utlidar/cloud_deskewed')
        p('odom_topic', '/utlidar/robot_odom')
        p('odom_frame', 'odom')
        p('base_frame', 'base_footprint')
        p('body_frame', 'base_link')
        # 아래 기본값은 실측 스윕 결과 (docs/01-measurements.md 측정 5)
        p('accumulate_sec', 0.75)      # 8프레임 = 포화점
        p('max_frames', 10)
        p('min_height', 0.05)          # 실측 바닥(-0.116~-0.05)에서 10cm 여유
        p('max_height', 0.60)          # Z p90=0.364. 위에는 점 없음
        p('angle_increment', 0.03491)  # 2.0deg -> 180빔
        p('range_min', 0.30)           # 실측 최소거리
        p('range_max', 12.0)           # 실측 최대 7.83m
        p('scan_rate', 10.0)
        p('tf_rate', 50.0)
        p('min_points_per_scan', 150)

        g = lambda k: self.get_parameter(k).value
        self.odom_frame = g('odom_frame')
        self.base_frame = g('base_frame')
        self.body_frame = g('body_frame')
        self.acc_sec = float(g('accumulate_sec'))
        self.max_frames = int(g('max_frames'))
        self.min_h = float(g('min_height'))
        self.max_h = float(g('max_height'))
        self.ainc = float(g('angle_increment'))
        self.rmin = float(g('range_min'))
        self.rmax = float(g('range_max'))
        self.min_pts = int(g('min_points_per_scan'))
        self.nbeam = int(round(2.0 * math.pi / self.ainc))
        self.amin = -math.pi
        self.scan_period = 1.0 / float(g('scan_rate'))

        qos = QoSProfile(depth=10,
                         reliability=ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.VOLATILE,
                         history=HistoryPolicy.KEEP_LAST)

        self.pub_scan = self.create_publisher(LaserScan, '/scan', 10)
        self.pub_odom = self.create_publisher(Odometry, '/odom', 10)
        self.br = TransformBroadcaster(self)
        self.create_subscription(PointCloud2, g('cloud_topic'), self.on_cloud, qos)
        self.create_subscription(Odometry, g('odom_topic'), self.on_odom, qos)

        self.buf = deque()
        self.pose = None
        self.last_scan_t = 0.0
        self.warned = False
        self.create_timer(1.0 / float(g('tf_rate')), self.tick)
        self.get_logger().info(
            'scan_maker: %d beams (%.2f deg), accum %.2fs, z=[%.2f,%.2f]'
            % (self.nbeam, math.degrees(self.ainc), self.acc_sec,
               self.min_h, self.max_h))

    def on_cloud(self, msg):
        # 원본 스탬프를 쓰지 않는다. 메인보드-확장독 시계 차이(-49s)를 회피.
        t = self.get_clock().now().nanoseconds * 1e-9
        p = cloud_to_xyz(msg)
        if p.shape[0] == 0:
            return
        self.buf.append((t, p))
        while len(self.buf) > self.max_frames or \
                (self.buf and t - self.buf[0][0] > self.acc_sec):
            self.buf.popleft()

    def on_odom(self, msg):
        self.pose = msg

    def tick(self):
        if self.pose is None:
            if not self.warned:
                self.get_logger().warn('waiting for odometry...')
                self.warned = True
            return

        now = self.get_clock().now()
        stamp = now.to_msg()
        pp = self.pose.pose.pose
        x, y, z = pp.position.x, pp.position.y, pp.position.z
        yaw = yaw_of(pp.orientation)
        roll, pitch = rp_of(pp.orientation)

        # odom -> base_footprint : yaw 만, z=0 (odom z=0 이 바닥면임을 실측으로 확인)
        t1 = TransformStamped()
        t1.header.stamp = stamp
        t1.header.frame_id = self.odom_frame
        t1.child_frame_id = self.base_frame
        t1.transform.translation.x = float(x)
        t1.transform.translation.y = float(y)
        t1.transform.translation.z = 0.0
        t1.transform.rotation.z = math.sin(yaw * 0.5)
        t1.transform.rotation.w = math.cos(yaw * 0.5)

        # base_footprint -> base_link : z, roll, pitch
        cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
        cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
        t2 = TransformStamped()
        t2.header.stamp = stamp
        t2.header.frame_id = self.base_frame
        t2.child_frame_id = self.body_frame
        t2.transform.translation.z = float(z)
        t2.transform.rotation.w = cr * cp
        t2.transform.rotation.x = sr * cp
        t2.transform.rotation.y = cr * sp
        t2.transform.rotation.z = -sr * sp
        self.br.sendTransform([t1, t2])

        od = Odometry()
        od.header.stamp = stamp
        od.header.frame_id = self.odom_frame
        od.child_frame_id = self.base_frame
        od.pose.pose.position.x = float(x)
        od.pose.pose.position.y = float(y)
        od.pose.pose.orientation = t1.transform.rotation
        od.twist = self.pose.twist
        self.pub_odom.publish(od)

        tsec = now.nanoseconds * 1e-9
        if tsec - self.last_scan_t < self.scan_period:
            return
        self.last_scan_t = tsec
        self.publish_scan(stamp, x, y, yaw)

    def publish_scan(self, stamp, x, y, yaw):
        if not self.buf:
            return
        # odom 프레임에서 누적했으므로 점들이 월드에 고정되어 있다.
        # 따라서 회전해도 번지지 않는다 (센서 프레임 누적과의 결정적 차이).
        pts = np.concatenate([b[1] for b in self.buf], axis=0)

        m = (pts[:, 2] > self.min_h) & (pts[:, 2] < self.max_h)
        if not m.any():
            return
        dx = pts[m, 0] - x
        dy = pts[m, 1] - y
        c, s = math.cos(-yaw), math.sin(-yaw)
        bx = c * dx - s * dy
        by = s * dx + c * dy

        rng = np.hypot(bx, by)
        m2 = (rng > self.rmin) & (rng < self.rmax)
        if m2.sum() < self.min_pts:
            return
        bx, by, rng = bx[m2], by[m2], rng[m2]

        ang = np.arctan2(by, bx)
        idx = ((ang - self.amin) / self.ainc).astype(np.int32)
        np.clip(idx, 0, self.nbeam - 1, out=idx)
        ranges = np.full(self.nbeam, np.inf, dtype=np.float32)
        np.minimum.at(ranges, idx, rng.astype(np.float32))

        sc = LaserScan()
        sc.header.stamp = stamp     # TF 와 동일 스탬프. 시계 불일치 원천 차단.
        sc.header.frame_id = self.base_frame
        sc.angle_min = float(self.amin)
        sc.angle_max = float(self.amin + self.ainc * (self.nbeam - 1))
        sc.angle_increment = float(self.ainc)
        sc.time_increment = 0.0
        sc.scan_time = float(self.scan_period)
        sc.range_min = float(self.rmin)
        sc.range_max = float(self.rmax)
        sc.ranges = ranges.tolist()
        self.pub_scan.publish(sc)


def main():
    rclpy.init()
    node = ScanMaker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
