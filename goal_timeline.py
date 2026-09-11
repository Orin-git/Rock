#!/usr/bin/env python3
"""Full per-goal timeline from the latched /xw/visual_db/build_status.

Read-only. One line per goal *index*, so the last successful REACHED and the
run of failures after it are both visible in order.
"""
import json, sys
import rclpy
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)
from std_msgs.msg import String

QOS = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                 durability=DurabilityPolicy.TRANSIENT_LOCAL,
                 history=HistoryPolicy.KEEP_LAST)


class R(Node):
    def __init__(self):
        super().__init__('vdb_goal_timeline')
        self.msg = None
        self.create_subscription(String, '/xw/visual_db/build_status', self._cb, QOS)

    def _cb(self, m):
        self.msg = m


rclpy.init(); n = R()
for _ in range(60):
    rclpy.spin_once(n, timeout_sec=0.25)
    if n.msg is not None:
        break
if n.msg is None:
    print('NO STATUS'); sys.exit(2)

d = json.loads(n.msg.data)
goals = d.get('goals') or []
print(f'session={d.get("build_session_id")} state={d.get("state")} '
      f'stop={d.get("stop_reason")!r} goals={len(goals)}')
print(f'{"#":>3} {"cell":<12} {"yaw":>3} {"nav":<18} {"elapsed":>8} {"note":<28} cap')
for i, g in enumerate(goals):
    el = g.get('nav_elapsed_sec')
    el = f'{el:.2f}' if isinstance(el, (int, float)) else '-'
    print(f'{i:>3} {str(g.get("cell")):<12} {str(g.get("yaw_bin")):>3} '
          f'{str(g.get("nav")):<18} {el:>8} {str(g.get("note") or "")[:28]:<28} '
          f'{g.get("capture")}')
print()
print('nav tally =', json.dumps({k: sum(1 for g in goals if g.get("nav") == k)
                                 for k in {g.get("nav") for g in goals}}))
rclpy.shutdown()
