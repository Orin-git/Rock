#!/usr/bin/env python3
"""Reproduce the capture-node arm/disarm race, then prove the fix closes it.

Run against the OLD code: the process dies with
    InvalidHandle: cannot use Destroyable because destruction was requested
Run against the FIXED code: it finishes clean.

Why this shape. The race needs (a) a subscription being destroyed and (b) the
executor concurrently taking from it. `_on_capture_cmd` runs `capture_once`
inline on a `_cb_svc` callback — i.e. on an executor thread — and capture_once's
`finally` calls `_disarm_heavy()`. Driving arm/disarm from a timer bound to the
SAME callback group therefore reproduces the exact context, at a rate the real
capture path cannot reach: a real cycle costs ~5 s of pipeline, here it is a
tight loop.

The capture pipeline itself is deliberately NOT run. It is not part of the racing
mechanism, it needs live sensor data, and 5 s/cycle would make the reproduction
take hours.

Two side effects are suppressed so this can run without touching the robot's
behaviour — neither is part of the mechanism under test:
  - /xw/reloc/rgb_request is stubbed (xw_depth_topic_bridge starts/stops the RGB
    stream on it; hammering it 400x would disturb a live build session);
  - the node is remapped so it does not answer to the real node's name.

Usage:
    python3 /ros2_ws/stress_capture_arm.py [cycles]      (default 400)
"""
import sys
import threading
import time

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


def stress() -> None:
    """Runs on an executor thread via a _cb_svc timer — same context as the real
    capture command callback."""
    done = 0
    try:
        for i in range(CYCLES):
            node._arm_heavy()
            if node._scan_sub is None or node._rgb_sub is None:
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

executor.shutdown()
node.destroy_node()
rclpy.shutdown()
sys.exit(rc)
