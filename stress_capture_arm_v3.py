#!/usr/bin/env python3
"""v1 verbatim + a snapshot at the moment the subscription check fails.

v1 is deterministic against the deployed (fixed) code right now:

    RESULT: CRASH after 0 cycles
            arm did not create subscriptions

but v1's own accounting cannot explain a failure at i=0 on code where
`_disarm_heavy` destroys nothing and never nulls `_scan_sub`/`_rgb_sub`. v2 —
which added a re-entry guard, an append-only result list and a wrapper around
`_arm_heavy` — reported 400 clean cycles. v2's guard is therefore the prime
suspect: the node's service callback group is a ReentrantCallbackGroup
(capture_candidate_node.py:157) and the 0.5 s timer is registered on it, so
`stress()` can run concurrently on both executor threads.

This file changes NOTHING about v1's mechanism (no guard, no wrapper, same
module-level `state`, same loop). It only records, at the failure point, which
attribute was None and what the node's arm state actually was. That settles
whether the failure is a genuine defect or an artifact of concurrent re-entry.

Usage:
    python3 /ros2_ws/stress_capture_arm_v3.py [cycles]
"""
import sys
import threading
import time
import traceback

import rclpy
from rclpy.executors import MultiThreadedExecutor

from xw_global_reloc.phase2d.capture_candidate_node import VisualDbCaptureNode

CYCLES = int(sys.argv[1]) if len(sys.argv) > 1 else 400
TIMEOUT_S = 300.0


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

state = {'done': False, 'cycles': 0, 'error': None, 'still_destroys': 0}
stop = threading.Event()

# --- instrumentation (the ONLY delta from v1) ---
trace = {'entries': 0, 'in_flight': 0, 'max_in_flight': 0,
         'fail': None, 'arms': [], 'invocations': []}
_lock = threading.Lock()


def stress() -> None:
    """Runs on an executor thread via a _cb_svc timer — same context as the real
    capture command callback."""
    with _lock:
        trace['entries'] += 1
        trace['in_flight'] += 1
        trace['max_in_flight'] = max(trace['max_in_flight'], trace['in_flight'])
        my_inv = trace['entries']
    done = 0
    try:
        for i in range(CYCLES):
            armed_before = bool(node._armed)
            node._arm_heavy()
            if node._scan_sub is None or node._rgb_sub is None:
                with _lock:
                    trace['fail'] = {
                        'invocation': my_inv,
                        'thread': threading.current_thread().name,
                        'i': i,
                        'armed_before_arm': armed_before,
                        'armed_after_arm': bool(node._armed),
                        'in_flight_now': trace['in_flight'],
                        'scan_sub': repr(node._scan_sub),
                        'rgb_sub': repr(node._rgb_sub),
                        'map_sub': repr(node._map_sub),
                        'amcl_sub': repr(getattr(node, '_amcl_sub', '<absent>')),
                        'odom_sub': repr(getattr(node, '_odom_sub', '<absent>')),
                        'rgb_req_count': node._rgb_req.count,
                    }
                state['error'] = 'arm did not create subscriptions'
                break
            node._disarm_heavy()
            # Observation, not a failure: on the OLD code disarm nulls the attrs
            # (it destroyed them); on the FIXED code they survive.
            if not node._armed and node._scan_sub is None:
                state['still_destroys'] += 1
            done = i + 1
    except Exception as exc:  # noqa: BLE001
        state['error'] = f'{type(exc).__name__}: {exc}'
        with _lock:
            trace['tb'] = traceback.format_exc()
    else:
        with _lock:
            trace['invocations'].append({'inv': my_inv, 'completed': done,
                                         'thread': threading.current_thread().name})
    finally:
        with _lock:
            trace['in_flight'] -= 1
    state['cycles'] = done
    state['done'] = True
    stop.set()


node.create_timer(0.5, stress, callback_group=node._cb_svc)

spin_thread = threading.Thread(target=executor.spin, daemon=True)
spin_thread.start()

t0 = time.time()
while not stop.is_set() and time.time() - t0 < TIMEOUT_S:
    time.sleep(0.2)

n = state['cycles']
destroys = state['still_destroys']

print(f"--- trace: entries={trace['entries']} max_concurrent={trace['max_in_flight']} ---")
for inv in trace['invocations']:
    print(f"    invocation {inv['inv']} on {inv['thread']}: completed={inv['completed']} (no error)")
if trace.get('tb'):
    print(trace['tb'])
if trace['fail']:
    print('--- failure snapshot ---')
    for k, v in trace['fail'].items():
        print(f'    {k} = {v}')

if not state['done']:
    print(f'RESULT: TIMEOUT after {n} cycles — race did not fire (INCONCLUSIVE)')
    rc = 3
elif state['error']:
    print(f'RESULT: CRASH after {n} cycles')
    print(f'        {state["error"]}')
    rc = 1
else:
    print(f'RESULT: SURVIVED {n} arm/disarm cycles, no crash')
    print(f'        disarm-destroyed the subscriptions {destroys} times '
          f'({"OLD behaviour — race still live" if destroys else "FIXED — arm/disarm no longer destroys"})')
    rc = 0 if destroys == 0 else 2

sys.stdout.flush()
import os  # noqa: E402
os._exit(rc)
