#!/usr/bin/env python3
"""Step 0b acceptance: is localization actually settled, and does it STAY settled?

A single `ros2 topic echo --once` cannot answer "sustained >= 60 s" — it spawns a
node per sample and takes seconds each. This subscribes once and samples the
whole window, so the answer is a distribution, not a point.

Read-only.
"""
import sys
import time

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node
from std_msgs.msg import Bool, Int8, String

DUR = float(sys.argv[1]) if len(sys.argv) > 1 else 70.0

rclpy.init()
node = Node('loc_acceptance_probe')
rec = {'loc': [], 'blocked': [], 'phase2c': [], 'amcl': 0, 'amcl_last': None}

node.create_subscription(Int8, '/xw/localization_status',
                         lambda m: rec['loc'].append((time.time(), int(m.data))), 10)
node.create_subscription(Bool, '/xw/nav/goals_blocked',
                         lambda m: rec['blocked'].append((time.time(), bool(m.data))), 10)
node.create_subscription(String, '/xw/localization/phase2c_state',
                         lambda m: rec['phase2c'].append((time.time(), m.data)), 10)


def on_amcl(msg):
    rec['amcl'] += 1
    p = msg.pose.pose.position
    rec['amcl_last'] = (round(p.x, 3), round(p.y, 3))


node.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', on_amcl, 10)

t0 = time.time()
while time.time() - t0 < DUR:
    rclpy.spin_once(node, timeout_sec=0.2)
elapsed = time.time() - t0

loc = rec['loc']
print(f'--- window = {elapsed:.1f} s ---')
if loc:
    vals = [v for _, v in loc]
    span = loc[-1][0] - loc[0][0]
    print(f'  /xw/localization_status : {len(loc)} msgs, {len(loc)/max(span,1e-9):.2f} Hz, '
          f'values={sorted(set(vals))}, min={min(vals)}, max={max(vals)}')
    print(f'      == 0 for the whole window: {all(v == 0 for v in vals)}')
    print(f'      first={loc[0][1]} last={loc[-1][1]}')
    # longest unbroken run of 0
    best = cur = 0
    for v in vals:
        cur = cur + 1 if v == 0 else 0
        best = max(best, cur)
    print(f'      longest unbroken run of 0: {best} msgs')
else:
    print('  /xw/localization_status : NO MESSAGES')

print(f'  /amcl_pose              : {rec["amcl"]} msgs, '
      f'{rec["amcl"]/max(elapsed,1e-9):.2f} Hz, last={rec["amcl_last"]}')
b = [v for _, v in rec['blocked']]
print(f'  /xw/nav/goals_blocked   : {len(b)} msgs, values={sorted(set(b)) if b else "none"}')
p = [v for _, v in rec['phase2c']]
print(f'  /xw/localization/phase2c_state : {sorted(set(p)) if p else "NONE"}')
print(f'  phase2c last: {rec["phase2c"][-1][1] if rec["phase2c"] else "NONE"}')

node.destroy_node()
rclpy.shutdown()
