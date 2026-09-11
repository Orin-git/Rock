#!/usr/bin/env python3
"""Disambiguate: did the robot move, or did AMCL drift?

Logs /odom pose (wheel+EKF, physical motion) and /amcl_pose (map frame) over the
same window. If odom moves, the robot moved and AMCL is fine. If odom is still
while amcl moves, AMCL is drifting -- and map->odom is absorbing the error.

Read-only.
"""
import math, time
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped

DUR = 60.0


class P(Node):
    def __init__(self):
        super().__init__('probe_drift')
        self.odom = []
        self.amcl = []
        self.create_subscription(Odometry, '/odom', self.on_odom, 10)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose',
                                 self.on_amcl, 10)

    def on_odom(self, m):
        p = m.pose.pose.position
        self.odom.append((time.time(), p.x, p.y))

    def on_amcl(self, m):
        p = m.pose.pose.position
        self.amcl.append((time.time(), p.x, p.y))


def span(seq, label):
    if len(seq) < 2:
        print(f'  {label}: {len(seq)} samples -- INSUFFICIENT')
        return None
    t0, x0, y0 = seq[0]
    t1, x1, y1 = seq[-1]
    d = math.hypot(x1 - x0, y1 - y0)
    xs = [s[1] for s in seq]; ys = [s[2] for s in seq]
    print(f'  {label}: {len(seq)} samples over {t1-t0:.1f} s')
    print(f'    first = ({x0:+.3f}, {y0:+.3f})   last = ({x1:+.3f}, {y1:+.3f})')
    print(f'    net displacement = {d:.3f} m')
    print(f'    bounding box     = {max(xs)-min(xs):.3f} x {max(ys)-min(ys):.3f} m')
    return d


rclpy.init(); n = P()
t0 = time.time()
while time.time() - t0 < DUR:
    rclpy.spin_once(n, timeout_sec=0.1)

print(f'=== {DUR:.0f} s ===')
d_odom = span(n.odom, '/odom  (physical, EKF)')
d_amcl = span(n.amcl, '/amcl_pose (map frame)')
print()
if d_odom is not None and d_amcl is not None:
    print(f'  VERDICT: odom moved {d_odom:.3f} m, amcl moved {d_amcl:.3f} m')
    if d_odom < 0.10 and d_amcl > 0.30:
        print('  -> ROBOT STATIONARY, AMCL DRIFTING. Localization is unstable.')
    elif d_odom > 0.30:
        print('  -> ROBOT PHYSICALLY MOVED. AMCL tracking it; no drift claim.')
    else:
        print('  -> NEITHER MOVED. Robot idle, localization quiet.')
rclpy.shutdown()
