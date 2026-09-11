#!/usr/bin/env python3
"""Compact one-shot report on the running Phase2D build session.

Prints only what the Step 2 acceptance table needs, so polling stays cheap.
Read-only.
"""
import json
from collections import Counter
from pathlib import Path

STATE = Path('/ros2_ws/maps/vp/visual/state')
LOG = Path('/ros2_ws/log/phase2d_c2/node_stdout.log')

sessions = sorted(STATE.glob('build_2026*.json'), key=lambda p: p.stat().st_mtime)
if not sessions:
    print('no session files')
    raise SystemExit(0)
f = sessions[-1]
d = json.loads(f.read_text(encoding='utf-8'))

print(f'=== {f.name}  ({f.stat().st_size} B, mtime {f.stat().st_mtime:.0f}) ===')
for k in ('state', 'stop_reason', 'mode', 'patrol_mode', 'build_kind', 'start_time',
          'end_time', 'coverage_before', 'coverage_after', 'active_version'):
    if k in d:
        print(f'  {k:18s} = {json.dumps(d[k], ensure_ascii=False)[:150]}')

goals = d.get('goals') or []
print(f'  goals              = {len(goals)}')
if goals:
    print(f'  goal nav counters  = {dict(Counter(str(g.get("nav")) for g in goals))}')
    print(f'  yaw bins per cell  = {len({(g.get("spatial_cell"), g.get("yaw_bin")) for g in goals})} pairs')
    el = [g.get('nav_elapsed_sec') for g in goals if 'nav_elapsed_sec' in g]
    print(f'  nav_elapsed_sec    = {len(el)}/{len(goals)} present'
          + (f'  min={min(el):.2f} max={max(el):.2f}' if el else ''))
    fast6 = [g for g in goals
             if str(g.get('nav')) == 'NAV_FAILED'
             and isinstance(g.get('nav_elapsed_sec'), (int, float))
             and g['nav_elapsed_sec'] < 1.0]
    print(f'  NAV_FAILED <1s     = {len(fast6)}   <- Fix-2b: was 14/14 yesterday')
    codes = Counter(str(g.get('nav_result_code')) for g in goals)
    print(f'  nav_result_code    = {dict(codes)}')

print()
print('--- log: planning rounds / states (tail 400 lines) ---')
if LOG.is_file():
    lines = LOG.read_text(encoding='utf-8', errors='replace').splitlines()
    rounds = [ln for ln in lines if 'PLANNING' in ln or 'stop_reason' in ln or 'STOPPED' in ln]
    for ln in rounds[-10:]:
        print('  ' + ln.split('] ', 1)[-1][:170])
    print(f'  (total log lines: {len(lines)})')
    print('  last 3:')
    for ln in lines[-3:]:
        print('    ' + ln.split('] ', 1)[-1][:170])
else:
    print('  (no node_stdout.log)')
