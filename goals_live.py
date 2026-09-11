#!/usr/bin/env python3
"""Live per-goal acceptance read from the latched /xw/visual_db/build_status.

Why not the session JSON: `_persist_session()` is called ONLY on exit paths
(measured 2026-09-11 — after 22 goals the live session's JSON did not exist on
disk at all). So the JSON can only be read after the session ends.

But `status_dict()` is literally `self._session.to_dict()`, so `goals[]` on the
latched topic IS live. Only `completion_summary` is stale, because that is
recomputed per round.

This prints the Fix-2 evidence for the goals completed SO FAR — the same fields
acceptance_report.py reads, just without waiting for the session to end.

Read-only. Needs TRANSIENT_LOCAL QoS (the topic is latched).
"""
import json
import sys
from collections import Counter

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

QOS = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                 durability=DurabilityPolicy.TRANSIENT_LOCAL,
                 history=HistoryPolicy.KEEP_LAST)


class Reader(Node):
    def __init__(self) -> None:
        super().__init__('vdb_goals_live_reader')
        self.msg = None
        self.create_subscription(String, '/xw/visual_db/build_status', self._cb, QOS)

    def _cb(self, m: String) -> None:
        self.msg = m


rclpy.init()
n = Reader()
for _ in range(60):
    rclpy.spin_once(n, timeout_sec=0.25)
    if n.msg is not None:
        break
if n.msg is None:
    print('NO STATUS RECEIVED (is the build node up?)')
    sys.exit(2)

d = json.loads(n.msg.data)
goals = d.get('goals') or []
print(f'session      = {d.get("build_session_id")}  state={d.get("state")}'
      f'  stop_reason={d.get("stop_reason")}')
print(f'goals live   = {len(goals)}   planned={d.get("planned_goals")}'
      f'   reached={d.get("reached_goals")}')
print()

# Fix-2: a cancelled goal must not be dressed up as a cell failure.
navc = Counter(g.get('nav') for g in goals)
print(f'[Fix-2 ] goals[].nav            = {dict(navc)}')
bad = [g for g in goals if g.get('nav') != 'REACHED'
       and str(g.get('nav_result_code', '')).startswith('CANCELLED')]
print(f'         cancelled-but-labelled-NAV_FAILED = {len(bad)}   (want 0)')

# Fix-2: elapsed must be on disk for every goal, so a 0.2 s abort is visible.
have = [g for g in goals if g.get('nav_elapsed_sec') is not None]
print(f'[Fix-2 ] nav_elapsed_sec present = {len(have)}/{len(goals)}')

# Fix-2b: the old failure mode was NAV2_STATUS:6 aborts at 0.16-0.95 s.
#
# Keyed on LABEL, this check misses the very case it looks for:
# build_orchestrator_node.py:810 sets rec['nav'] = nav_code on the
# cancel/instant-abort branch, so the label there is the RAW code
# ('NAV2_STATUS:6'), never 'NAV_FAILED'. Match on elapsed alone.
sub = [g for g in goals
       if g.get('nav') != 'REACHED'
       and g.get('nav_elapsed_sec') is not None
       and float(g['nav_elapsed_sec']) < 1.0]
labels = sorted({str(g.get('nav')) for g in sub})
print(f'[Fix-2b] ANY non-REACHED goal with elapsed < 1.0 s = {len(sub)}'
      f'   labels={labels}   (yesterday 14/14, all NAV2_STATUS:6)')
for g in sub:
    print(f'         {g.get("spatial_cell")} yaw={g.get("yaw_bin")} '
          f'elapsed={g.get("nav_elapsed_sec")}s nav={g.get("nav")} '
          f'note={g.get("nav_note")} requeued={g.get("nav_requeued")}')
if have:
    el = sorted(float(g['nav_elapsed_sec']) for g in have)
    print(f'         elapsed spread: min={el[0]:.1f}s  median={el[len(el)//2]:.1f}s'
          f'  max={el[-1]:.1f}s')

print()
print(f'[retry]  nav_retryable        = {dict(Counter(g.get("nav_retryable") for g in goals))}')
print(f'[ledger] permanent_unreachable= {dict(Counter(g.get("permanent_unreachable") for g in goals))}')

# Fix-1 evidence without the JSON: the planner logs its round number to the trace.
print()
# Raw "distinct vs total" is misleading here: Fix-2 deliberately REQUEUES a
# cancelled goal at the tail, so a (cell,yaw) pair legitimately appears twice.
# Separate that from a genuine planning duplicate — one whose earlier attempt
# already REACHED would be a real bug.
seen: dict = {}
dupes = []
for i, g in enumerate(goals):
    k = (g.get('spatial_cell'), g.get('yaw_bin'))
    if k in seen:
        dupes.append((i, seen[k], k, goals[seen[k]].get('nav')))
    else:
        seen[k] = i
real_dup = [d for d in dupes if d[3] == 'REACHED']
print(f'[Fix-1 ] distinct (cell,yaw) = {len(seen)} of {len(goals)} goals; '
      f'repeated appearances = {len(dupes)} (requeue), of which the earlier '
      f'attempt had already REACHED = {len(real_dup)}  <- want 0')
print(f'[caps  ] capture result codes = '
      f'{dict(Counter(g.get("capture") for g in goals))}')

print()
print('[per-goal] every goal whose capture is not ACCEPTED, plus ERROR clustering')
errs = [i for i, g in enumerate(goals) if g.get('capture') == 'ERROR']
for i, g in enumerate(goals):
    if g.get('capture') == 'ACCEPTED':
        continue
    # A goal appended but not yet filled in has spatial_cell=None — str() it,
    # an f-string format spec on None raises.
    print(f'  #{i:>3}  {str(g.get("spatial_cell")):<12} yaw={g.get("yaw_bin")}  '
          f'nav={str(g.get("nav")):<16} capture={g.get("capture")}')
if errs:
    # Only goals that actually went to COLLECTING carry a capture result; a
    # cancelled goal has capture=None and never knocked. Compare over those, or
    # a None in the middle makes a clean head-cluster look "scattered".
    attempted = [i for i, g in enumerate(goals) if g.get('capture') is not None]
    last_err = max(errs)
    before = [i for i in attempted if i <= last_err]
    print(f'  attempted captures = {len(attempted)}   ERROR goals = #{errs}')
    if sorted(before) == sorted(errs):
        print(f'  every capture attempted before #{last_err + 1} failed and none '
              f'after => CAPTURE NODE WAS NOT UP YET — operational, not a defect')
    else:
        good = [i for i in before if i not in errs]
        print(f'  ERRORs are interleaved with successes (#{good[:4]}) — investigate')

n.destroy_node()
rclpy.shutdown()
