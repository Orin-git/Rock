#!/usr/bin/env python3
"""Prove the build_busy branch of start_build() cannot self-deadlock.

`self._lock` is a plain (non-reentrant) threading.Lock. When the build_busy
branch called the public status_dict() -- which itself does `with self._lock` --
the nested acquire wedged the node forever: it stopped answering every command
while still looking alive from the outside.

The test drives exactly that branch: make `_worker` a live thread, then call
start_build() from a SECOND thread and join with a timeout.

    OLD code: join times out -> the calling thread is stuck inside the lock.
    NEW code: returns {'ok': False, 'message': 'build_busy', ...} at once.

No build is started either way: a live worker short-circuits before any state is
touched. The node is name-remapped so it cannot be mistaken for the real one.
"""
import sys
import threading
import time

import rclpy

from xw_global_reloc.phase2d.build_orchestrator_node import VisualDbBuildOrchestrator

TIMEOUT = 5.0

rclpy.init(args=['--ros-args', '-r', '__node:=xw_vdb_build_deadlock_probe'])
node = VisualDbBuildOrchestrator()

# A live, never-finishing worker: this is the "build_busy" precondition.
release = threading.Event()


def _forever() -> None:
    release.wait(60.0)


busy = threading.Thread(target=_forever, daemon=True)
busy.start()
node._worker = busy

result = {}
returned = threading.Event()


def call_start() -> None:
    try:
        result['value'] = node.start_build(mode='RESUME_BUILD', patrol_mode='full')
    except Exception as exc:  # noqa: BLE001
        result['error'] = f'{type(exc).__name__}: {exc}'
    returned.set()


t0 = time.time()
caller = threading.Thread(target=call_start, daemon=True)
caller.start()
ok = returned.wait(TIMEOUT)
elapsed = time.time() - t0

release.set()

if not ok:
    print(f'RESULT: HUNG — start_build() did not return within {TIMEOUT:.0f}s.')
    print('        The build_busy branch deadlocked on a nested acquire of the')
    print('        non-reentrant self._lock. This is the DEFECT.')
    rc = 1
elif 'error' in result:
    print(f'RESULT: RAISED after {elapsed:.3f}s — {result["error"]}')
    rc = 2
else:
    v = result['value']
    good = (isinstance(v, dict) and v.get('ok') is False
            and v.get('message') == 'build_busy')
    print(f'RESULT: RETURNED after {elapsed:.3f}s -> ok={v.get("ok")!r} '
          f'message={v.get("message")!r}')
    if good:
        print('        Correct: build_busy reported without blocking. FIX CONFIRMED.')
        rc = 0
    else:
        print('        Returned, but not the expected build_busy payload.')
        rc = 3

node.destroy_node()
rclpy.shutdown()
sys.exit(rc)
