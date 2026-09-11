#!/usr/bin/env python3
"""Why is the robot not moving? Read-only, no continuous echo on sensor topics.

Measures: chassis safety flags, laser obstacle proximity in the direction of
travel, localization quality, and whether a nav goal is actually active.
"""
import math, sys, time
import rclpy
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)
from rclpy.qos import qos_profile_sensor_data

from std_msgs.msg import Bool, String
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from tf2_ros import Buffer, TransformListener

LATCH = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                   durability=DurabilityPolicy.TRANSIENT_LOCAL,
                   history=HistoryPolicy.KEEP_LAST)


class P(Node):
    def __init__(self):
        super().__init__('probe_blocked')
        self.v = {}
        self.scan = None
        self.amcl = None
        self.odom = None
        PQ = QoSProfile(depth=10)
        PQ.reliability = ReliabilityPolicy.RELIABLE
        PQ.durability = DurabilityPolicy.VOLATILE
        for name, typ in (('/xw/chassis/motor_disabled', Bool),
                          ('/xw/chassis/emergency_stop', Bool),
                          ('/xw/chassis/safety_ok', Bool),
                          ('/emergency_stop', Bool),
                          ('/xw/nav/goals_blocked', Bool),
                          ('/xw/localization_status', String),
                          ('/xw/localization/phase2c_state', String)):
            self.create_subscription(typ, name,
                                     lambda m, n=name: self.v.__setitem__(n, m),
                                     PQ)
        self.create_subscription(LaserScan, '/scan', self.on_scan,
                                 qos_profile_sensor_data)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose',
                                 lambda m: setattr(self, 'amcl', m), 10)
        self.create_subscription(Odometry, '/odom',
                                 lambda m: setattr(self, 'odom', m), 10)
        self.tf = Buffer()
        self.create_subscription  # noqa
        self.tfl = TransformListener(self.tf, self)

    def on_scan(self, m):
        self.scan = m


rclpy.init()
n = P()
t0 = time.time()
while time.time() - t0 < 8.0:
    rclpy.spin_once(n, timeout_sec=0.2)

print('=== chassis / latched ===')
for k, m in sorted(n.v.items()):
    val = m.data if hasattr(m, 'data') else m
    print(f'  {k:<40} {val!r}')

print('\n=== scan ===')
if n.scan is None:
    print('  /scan: NO MESSAGE')
else:
    s = n.scan
    n_valid = sum(1 for r in s.ranges if math.isfinite(r) and r > 0.01)
    finite = [r for r in s.ranges if math.isfinite(r) and r > 0.01]
    print(f'  frame={s.header.frame_id} rays={len(s.ranges)} valid={n_valid} '
          f'range_min={s.range_min:.3f} range_max={s.range_max:.3f}')
    if finite:
        i = min(range(len(s.ranges)), key=lambda j: (s.ranges[j] if math.isfinite(s.ranges[j]) and s.ranges[j] > 0.01 else 1e9))
        ang = math.degrees(s.angle_min + i * s.angle_increment)
        print(f'  global min range = {min(finite):.3f} m at {ang:+.1f} deg')
    # forward cone +-30 deg
    cone = []
    for j, r in enumerate(s.ranges):
        if not (math.isfinite(r) and r > 0.01):
            continue
        a = math.degrees(s.angle_min + j * s.angle_increment)
        if -30.0 <= a <= 30.0:
            cone.append((r, a))
    if cone:
        r, a = min(cone)
        print(f'  forward cone +-30deg: min = {r:.3f} m at {a:+.1f} deg  ({len(cone)} rays)')
    else:
        print('  forward cone: no valid rays')

print('\n=== localization ===')
if n.amcl is None:
    print('  /amcl_pose: NO MESSAGE')
else:
    p = n.amcl.pose.pose
    c = n.amcl.pose.covariance
    cov_xy = max(c[0], c[7])
    print(f'  amcl_pose = [{p.position.x:.3f}, {p.position.y:.3f}]  '
          f'cov_xy={cov_xy:.4f}  age={time.time()-n.amcl.header.stamp.sec - n.amcl.header.stamp.nanosec*1e-9 + 0:.1f}')
if n.odom is not None:
    p = n.odom.pose.pose
    print(f'  odom_pose = [{p.position.x:.3f}, {p.position.y:.3f}] frame={n.odom.header.frame_id}')

print('\n=== TF map->base_link ===')
try:
    import rclpy.time
    tr = n.tf.lookup_transform('map', 'base_link', rclpy.time.Time())
    t = tr.transform.translation
    print(f'  map->base_link = [{t.x:.3f}, {t.y:.3f}, {t.z:.3f}]')
except Exception as e:
    print(f'  TF FAILED: {type(e).__name__}: {e}')

rclpy.shutdown()
