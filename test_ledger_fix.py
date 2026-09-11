#!/usr/bin/env python3
"""Unit + replay tests for the Phase2D nav-fail ledger fix.

Run inside the container:
  python3 /ros2_ws/test_ledger_fix.py
"""
import json
import sys
import tempfile
from pathlib import Path

from xw_global_reloc.phase2d.build_completion import (
    is_cell_attributable_failure,
    update_nav_fail_history,
)

FAILED = 0


def check(label, got, want):
    global FAILED
    ok = got == want
    if not ok:
        FAILED += 1
    print(f"  [{'PASS' if ok else 'FAIL'}] {label}: got={got!r} want={want!r}")


print("=== 1. is_cell_attributable_failure matrix ===")
cases = [
    # (code, elapsed, expected, why)
    ('CANCELLED', 43.0, False, 'interlock cancel, real duration'),
    ('ABORTED', 1.0, False, 'operator stop'),
    ('INTERLOCK_PAUSED', 40.0, False, 'interlock cancel'),
    ('NAV2_UNAVAILABLE', 5.0, False, 'Nav2 server absent -> system, not cell'),
    ('NAV2_STATUS:6', 0.17, False, 'MEASURED bogus abort (session A)'),
    ('NAV2_STATUS:6', 0.95, False, 'MEASURED bogus abort (session A, slowest)'),
    ('NAV2_STATUS:6', 45.0, True, 'real Nav2 abort after driving'),
    ('TIMEOUT', 120.0, True, 'ran the full budget, never arrived'),
    ('TIMEOUT', 1.0, False, 'sub-floor timeout is a preamble failure'),
    ('GOAL_REJECTED', 0.1, True, 'preflight: goal itself is invalid'),
    ('SEND_TIMEOUT', 10.0, True, 'action server accepted nothing in 10 s'),
    (None, 30.0, False, 'no code -> ambiguous -> stay retryable'),
    ('NAV2_STATUS:6', None, False, 'no duration -> ambiguous -> stay retryable'),
    ('WHAT_IS_THIS', 30.0, False, 'unknown code -> stay retryable'),
]
for code, elapsed, want, why in cases:
    check(f'{code!r}@{elapsed}s ({why})', is_cell_attributable_failure(code, elapsed), want)

print("\n=== 2. update_nav_fail_history synthetic session ===")
with tempfile.TemporaryDirectory() as td:
    root = Path(td)
    goals = [
        {'spatial_cell': 'cell_A', 'nav': 'NAV_FAILED', 'nav_result_code': 'CANCELLED', 'nav_elapsed_sec': 43.0},
        {'spatial_cell': 'cell_B', 'nav': 'NAV_FAILED', 'nav_result_code': 'NAV2_STATUS:6', 'nav_elapsed_sec': 0.2},
        {'spatial_cell': 'cell_C', 'nav': 'NAV_FAILED', 'nav_result_code': 'NAV2_STATUS:6', 'nav_elapsed_sec': 45.0},
        {'spatial_cell': 'cell_D', 'nav': 'NAV_FAILED', 'nav_result_code': 'TIMEOUT', 'nav_elapsed_sec': 120.0},
        {'spatial_cell': 'cell_E', 'nav': 'NAV_FAILED', 'nav_result_code': 'CANCELLED', 'nav_elapsed_sec': 30.0},
        {'spatial_cell': 'cell_E', 'nav': 'REACHED', 'nav_result_code': 'SUCCEEDED:4', 'nav_elapsed_sec': 22.0},
        {'spatial_cell': 'cell_F', 'nav': 'NAV_FAILED', 'nav_result_code': 'GOAL_REJECTED', 'nav_elapsed_sec': 0.1},
    ]
    got = update_nav_fail_history(root, goals)
    check('session 1 ledger', got, {'cell_C': 1, 'cell_D': 1, 'cell_F': 1})

    # session 2: cell_C fails again, cell_D is reached -> clears
    goals2 = [
        {'spatial_cell': 'cell_C', 'nav': 'NAV_FAILED', 'nav_result_code': 'NAV2_STATUS:6', 'nav_elapsed_sec': 60.0},
        {'spatial_cell': 'cell_D', 'nav': 'REACHED', 'nav_result_code': 'SUCCEEDED:4', 'nav_elapsed_sec': 30.0},
    ]
    got2 = update_nav_fail_history(root, goals2)
    check('session 2 ledger (C ratchets, D cleared)', got2, {'cell_C': 2, 'cell_F': 1})

print("\n=== 3. REPLAY: real session A goals under the new accounting ===")
sess = Path('/ros2_ws/maps/vp/visual/state/build_20260910_075732_dacbb5.json')
if not sess.is_file():
    print(f"  [SKIP] {sess} not found")
else:
    data = json.loads(sess.read_text(encoding='utf-8'))
    with tempfile.TemporaryDirectory() as td:
        got = update_nav_fail_history(Path(td), data.get('goals') or [])
    recorded = json.loads(
        Path('/ros2_ws/maps/vp/visual/state/nav_fail_history.json').read_text(encoding='utf-8')
    ).get('cells', {})
    print(f"  recorded ledger (polluted): {json.dumps(recorded, sort_keys=True)}")
    print(f"  new accounting would produce: {json.dumps(got, sort_keys=True)}")
    check('every recorded cell is now rejected as non-attributable', got, {})

print("\n=== 4. session B replay ===")
sessb = Path('/ros2_ws/maps/vp/visual/state/build_20260910_091307_f2974f.json')
if sessb.is_file():
    data = json.loads(sessb.read_text(encoding='utf-8'))
    with tempfile.TemporaryDirectory() as td:
        got = update_nav_fail_history(Path(td), data.get('goals') or [])
    check('session B also yields an empty ledger', got, {})

print()
print(f"RESULT: {'ALL PASS' if FAILED == 0 else str(FAILED) + ' FAILURE(S)'}")
sys.exit(1 if FAILED else 0)
