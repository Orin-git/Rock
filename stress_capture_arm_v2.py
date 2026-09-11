#!/usr/bin/env python3
"""Instrumented re-run of stress_capture_arm.py.

The uninstrumented script reported

    RESULT: CRASH after 400 cycles
            arm did not create subscriptions

which its own accounting cannot produce. `done = i + 1` is the LAST statement of
the loop body, so it only executes on the path where the subscription check
PASSED; a `break` out of that check leaves `done <= CYCLES - 1`. With the default
CYCLES=400, `done == 400` therefore implies the loop ran to completion and
`state['error']` was never set — yet the crash branch printed. Two candidate
explanations, and this run separates them:

  (a) the 0.5 s timer re-entered `stress()` and a second invocation wrote
      `state['error']` while the first invocation's `state['cycles']=400` was
      still the most recent value for that key;
  (b) the check genuinely failed, i.e. `_scan_sub`/`_rgb_sub` was None right
      after `_arm_heavy()` returned — which the fixed code should make
      impossible (`_disarm_heavy` destroys nothing and never nulls them).

Differences from v1, all aimed at (a) vs (b):
  - results are APPENDED, one record per invocation — no cross-invocation
    overwrite, so re-entry becomes visible instead of corrupting the answer;
  - the failing iteration index and the live values of `_armed`, `_scan_sub`,
    `_rgb_sub`, `_map_sub` are captured at the moment of failure;
  - `_arm_heavy`/`_disarm_heavy` are wrapped, so an early return (already armed)
    or a silent exception is visible rather than inferred;
  - the main loop polls at 50 ms instead of 200 ms.

Usage:
    python3 /ros2_ws/stress_capture_arm_v2.py [cycles] [timeout_s]
"""
import sys
import threading
import time
import traceback

import rclpy
from rclpy.executors import MultiThreadedExecutor

from xw_global_reloc.phase2d.capture_candidate_node import VisualDbCaptureNode

CYCLES = int(sys.argv[1]) if len(sys.argv) > 1 else 400
TIMEOUT_S = float(sys.argv[2]) if len(sys.argv) > 2 else 120.0


class _NullPub:
    """Stand-in for the rgb_request publisher — counts, publishes nothing."""

    def __init__(self) -> None:
        self.count = 0

    def publish(self, _msg) -> None:
        self.count += 1


rclpy.init(args=['--ros-args', '-r', '__node:=xw_vdb_capture_stress'])
node = VisualDbCaptureNode()
node._rgb_req = _NullPub()  # must be replaced BEFORE the first arm

executor = MultiThreadedExecutor(num_threads=2)
executor.add_node(node)

runs = []
stop = threading.Event()
running = threading.Event()
lock = threading.Lock()

_real_arm = node._arm_heavy
_real_disarm = node._disarm_heavy


def _traced_arm():
    """Returns (early_return, exception_string)."""
    early = bool(node._armed)
    err = None
    try:
        _real_arm()
    except BaseException as exc:  # noqa: BLE001
        err = f'{type(exc).__name__}: {exc}'
        raise
    return early, err


def _traced_disarm():
    return _real_disarm()


node._arm_heavy = _traced_arm
node._disarm_heavy = _traced_disarm


def _snap(i, early):
    return {
        'i': i,
        'armed': bool(node._armed),
        'arm_early_return': early,
        'scan_sub': repr(node._scan_sub),
        'rgb_sub': repr(node._rgb_sub),
        'map_sub': repr(node._map_sub),
        'amcl_sub': repr(getattr(node, '_amcl_sub', '<absent>')),
        'odom_sub': repr(getattr(node, '_odom_sub', '<absent>')),
        'rgb_req_publish_count': node._rgb_req.count,
    }


def stress() -> None:
    """Runs on an executor thread via a _cb_svc timer — same context as the real
    capture command callback."""
    if running.is_set() or stop.is_set():
        return
    running.set()
    rec = {'inv': len(runs) + 1, 'done': 0, 'error': None, 'fail_i': None,
           'early_returns': 0, 'arm_exc': None, 'fail_state': None,
           'still_destroys': 0, 'cycles_arg': CYCLES}
    try:
        for i in range(CYCLES):
            early, arm_exc = _traced_arm()
            if early:
                rec['early_returns'] += 1
            if arm_exc:
                rec['arm_exc'] = arm_exc
            if node._scan_sub is None or node._rgb_sub is None:
                rec['error'] = 'arm did not create subscriptions'
                rec['fail_i'] = i
                rec['fail_state'] = _snap(i, early)
                break
            _traced_disarm()
            # Observation, not a failure: on the OLD code disarm nulls the attrs
            # (it destroyed them); on the FIXED code they survive.
            if not node._armed and node._scan_sub is None:
                rec['still_destroys'] += 1
            rec['done'] = i + 1
    except BaseException as exc:  # noqa: BLE001
        rec['error'] = f'{type(exc).__name__}: {exc}'
        rec['traceback'] = traceback.format_exc()
    with lock:
        runs.append(rec)
    running.clear()
    stop.set()


node.create_timer(0.5, stress, callback_group=node._cb_svc)

spin_thread = threading.Thread(target=executor.spin, daemon=True)
spin_thread.start()

t0 = time.time()
while not stop.is_set() and time.time() - t0 < TIMEOUT_S:
    time.sleep(0.05)

print(f'--- invocations of stress(): {len(runs)} ---')
for rec in runs:
    print(f"invocation {rec['inv']}: cycles_completed={rec['done']} "
          f"error={rec['error']!r} fail_i={rec['fail_i']} "
          f"early_returns={rec['early_returns']} "
          f"still_destroys={rec['still_destroys']}")
    if rec['fail_state']:
        for k, v in rec['fail_state'].items():
            print(f'    {k} = {v}')
    if rec.get('traceback'):
        print(rec['traceback'])

if not runs:
    print(f'RESULT: INCONCLUSIVE — stress() never ran within {TIMEOUT_S:.0f}s')
    sys.exit(3)

last = runs[-1]
if last['error']:
    print(f"RESULT: CRASH after {last['done']} cycles")
    print(f"        {last['error']}")
    rc = 1
else:
    print(f"RESULT: SURVIVED {last['done']} arm/disarm cycles, no crash")
    print(f"        disarm-destroyed the subscriptions {last['still_destroys']} times")
    rc = 0 if last['still_destroys'] == 0 else 2

sys.stdout.flush()
import os  # noqa: E402
os._exit(rc)
