#!/usr/bin/env python3
"""Prove _disarm_motion_subs() no longer tears down live subscriptions.

OLD code: every disarm calls destroy_subscription() on the amcl/odom
subscriptions and nulls the handles, from a callback context (stop_build runs on
the executor thread via _on_build_cmd; the worker's `finally` is the other
caller). rclpy does not synchronise entity destruction against a spinning
executor, so this is the same race that killed capture_candidate_node with

    InvalidHandle: cannot use Destroyable because destruction was requested

and lost_recovery_node twice (02:02:26, 03:17:35).

NEW code: created once, never destroyed. Disarm clears the retained payloads and
flips `_motion_armed`, so `_on_amcl`/`_on_odom` drop messages while disarmed.

Two checks:

  1. Guard (no executor needed): a pose delivered while DISARMED must not be
     retained; the same pose after ARM must be.
  2. Teardown: N arm/disarm cycles driven from an executor callback must not
     crash, and the handles must survive every disarm.

The cycles deliberately run on the node's own `_state_cb` group — the same
context stop_build() runs in — at a rate the real build path cannot reach.

rc: 0 = survived, handles survive ... FIXED
    2 = survived, disarm still destroys ... OLD behaviour, race still live
    1 = crashed / raised
    3 = timed out

Usage:
    python3 /ros2_ws/stress_motion_subs.py [cycles] [timeout_s]
"""
import sys
import threading
import time
import traceback

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.executors import MultiThreadedExecutor

from xw_global_reloc.phase2d.build_orchestrator_node import VisualDbBuildOrchestrator

CYCLES = int(sys.argv[1]) if len(sys.argv) > 1 else 400
TIMEOUT_S = float(sys.argv[2]) if len(sys.argv) > 2 else 120.0

rclpy.init(args=['--ros-args', '-r', '__node:=xw_vdb_motion_stress'])
node = VisualDbBuildOrchestrator()

# --- check 1: the callback guard, no executor involved ---------------------
guard = {}
try:
    fake = PoseWithCovarianceStamped()
    node._disarm_motion_subs()
    node._on_amcl(fake)
    guard['retained_while_disarmed'] = node._amcl is not None
    node._arm_motion_subs()
    node._on_amcl(fake)
    guard['retained_while_armed'] = node._amcl is not None
    node._disarm_motion_subs()
except BaseException as exc:  # noqa: BLE001
    guard['error'] = f'{type(exc).__name__}: {exc}'
    guard['traceback'] = traceback.format_exc()

executor = MultiThreadedExecutor(num_threads=2)
executor.add_node(node)

stop = threading.Event()
running = threading.Event()
state = {'done': 0, 'error': None, 'traceback': None, 'still_destroys': 0,
         'subs_missing_after_arm': 0, 'entries': 0}


def stress() -> None:
    """Runs on an executor thread via a _state_cb timer — the same context as
    stop_build()'s caller."""
    if running.is_set() or stop.is_set():
        return
    running.set()
    state['entries'] += 1
    done = 0
    try:
        for i in range(CYCLES):
            node._arm_motion_subs()
            if node._amcl_sub is None or node._odom_sub is None:
                state['subs_missing_after_arm'] += 1
            node._disarm_motion_subs()
            # Observation, not a failure: the OLD code nulls the handles here
            # because it destroyed them; the FIXED code leaves them in place.
            if node._amcl_sub is None or node._odom_sub is None:
                state['still_destroys'] += 1
            done = i + 1
    except BaseException as exc:  # noqa: BLE001
        state['error'] = f'{type(exc).__name__}: {exc}'
        state['traceback'] = traceback.format_exc()
    state['done'] = done
    running.clear()
    stop.set()


node.create_timer(0.5, stress, callback_group=node._state_cb)

spin_thread = threading.Thread(target=executor.spin, daemon=True)
spin_thread.start()

t0 = time.time()
while not stop.is_set() and time.time() - t0 < TIMEOUT_S:
    time.sleep(0.05)

print('--- check 1: callback guard ---')
for k, v in guard.items():
    print(f'    {k} = {v}')

print(f"--- check 2: {state['done']} arm/disarm cycles "
      f"(entries={state['entries']}, still_destroys={state['still_destroys']}, "
      f"subs_missing_after_arm={state['subs_missing_after_arm']}) ---")
if state['traceback']:
    print(state['traceback'])

if not state['done']:
    print('RESULT: TIMEOUT — race did not fire (INCONCLUSIVE)')
    rc = 3
elif state['error']:
    print(f"RESULT: CRASH after {state['done']} cycles")
    print(f"        {state['error']}")
    rc = 1
else:
    print(f"RESULT: SURVIVED {state['done']} arm/disarm cycles, no crash")
    if state['still_destroys']:
        print(f"        disarm destroyed+nulled the subscriptions "
              f"{state['still_destroys']} times — OLD behaviour, race still live")
        rc = 2
    else:
        print('        disarm destroyed nothing; handles survive every cycle — FIXED')
        rc = 0

sys.stdout.flush()
import os  # noqa: E402
os._exit(rc)
