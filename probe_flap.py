#!/usr/bin/env python3
"""Sample phase2c_state + amcl_pose for 90 s. Read-only.

Answers: how often does the state flap, and does the AMCL pose jump?
State transitions and pose deltas > 0.30 m are the only interesting events.
"""
import math, time
import rclpy
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)
from std_msgs.msg import String
from geometry_msgs.msg import PoseWithCovarianceStamped

LATCH = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                   durability=DurabilityPolicy.TRANSIENT_LOCAL,
                   history=HistoryPolicy.KEEP_LAST)
DUR = 90.0


class P(Node):
    def __init__(self):
        super().__init__('probe_flap')
        self.state = None
        self.events = []
        self.poses = []
        self.create_subscription(String, '/xw/localization/phase2c_state',
                                 self.on_state, LATCH)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose',
                                 self.on_pose, 10)

    def on_state(self, m):
        import json
        try:
            s = json.loads(m.data).get('state')
        except Exception:
            s = '?'
        if s != self.state:
            self.state = s
            self.events.append((time.time(), 'STATE', s))

    def on_pose(self, m):
        p = m.pose.pose.position
        c = m.pose.covariance
        self.poses.append((time.time(), p.x, p.y, max(c[0], c[7])))


rclpy.init(); n = P()
t0 = time.time()
last = None
while time.time() - t0 < DUR:
    rclpy.spin_once(n, timeout_sec=0.1)
    if n.poses:
        cur = n.poses[-1]
        if last and math.hypot(cur[1] - last[1], cur[2] - last[2]) > 0.30:
            n.events.append((cur[0], 'JUMP',
                             f'{math.hypot(cur[1]-last[1], cur[2]-last[2]):.2f} m '
                             f'({last[1]:.2f},{last[2]:.2f})->({cur[1]:.2f},{cur[2]:.2f})'))
        last = cur

print(f'=== {DUR:.0f} s observation ===')
print(f'amcl_pose samples = {len(n.poses)}  '
      f'({len(n.poses)/DUR:.1f} Hz)')
if n.poses:
    xs = [p[1] for p in n.poses]; ys = [p[2] for p in n.poses]; cs = [p[3] for p in n.poses]
    print(f'  x range [{min(xs):.3f}, {max(xs):.3f}]  span={max(xs)-min(xs):.3f}')
    print(f'  y range [{min(ys):.3f}, {max(ys):.3f}]  span={max(ys)-min(ys):.3f}')
    print(f'  cov_xy  [{min(cs):.4f}, {max(cs):.4f}]')
    print(f'  first=({n.poses[0][1]:.3f},{n.poses[0][2]:.3f})  '
          f'last=({n.poses[-1][1]:.3f},{n.poses[-1][2]:.3f})')
print()
print('=== events (state changes + pose jumps > 0.30 m) ===')
if not n.events:
    print('  (none)')
for t, kind, val in n.events:
    print(f'  t+{t-t0:6.1f}s  {kind:<6} {val}')
print()
tally = {}
for _, k, _ in n.events:
    tally[k] = tally.get(k, 0) + 1
print('tally =', tally, f'  -> state flaps per minute = '
      f'{tally.get("STATE", 0) / (DUR/60.0):.1f}')
rclpy.shutdown()
