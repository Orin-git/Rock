#!/usr/bin/env python3
"""Offline reproduction of the Phase2D session deadlock (D1) and its fix (Fix-1).

Replays the real session-A goal set against the production coverage model and
compares planner output with and without `exclude_pairs`.

Pure python — builds no ROS nodes.
"""
import json
from pathlib import Path

from xw_global_reloc.phase2d.build_completion import (
    evaluate_coverage_completion,
    load_nav_fail_history,
    remaining_gap_cells,
)
from xw_global_reloc.phase2d.config_loader import load_phase2d_config, production_visual_root
from xw_global_reloc.phase2d.coverage_model import build_coverage_model
from xw_global_reloc.phase2d.patrol_planner import plan_patrol_goals

cfg = load_phase2d_config()
vroot = production_visual_root(cfg)
maps_dir = Path(str(cfg.get('maps_dir') or '/ros2_ws/maps'))
map_yaml = maps_dir / 'vp.yaml'
print(f'map_yaml   = {map_yaml}  exists={map_yaml.is_file()}')
print(f'visual_root= {vroot}')

sess_path = vroot / 'state' / 'build_20260910_075732_dacbb5.json'
session = json.loads(sess_path.read_text(encoding='utf-8'))
attempted = set()
for g in session.get('goals') or []:
    attempted.add((str(g.get('spatial_cell')), int(g.get('yaw_bin'))))
print(f'session A attempted (cell, yaw_bin) pairs: {len(attempted)}')

model = build_coverage_model(cfg, load_descriptors=False)
hist = load_nav_fail_history(vroot)
session_fail_ids = {
    str(g.get('spatial_cell')) for g in session.get('goals') or []
    if g.get('nav') == 'NAV_FAILED'
}
print(f'nav_fail_history cells = {len(hist)}  session_fail_ids = {len(session_fail_ids)}')

completion = evaluate_coverage_completion(
    model,
    map_yaml,
    cfg,
    seed_xy=None,
    nav_fail_history=hist,
    session_nav_failed_cells=session_fail_ids,
    patrol_mode='full',
    build_kind='RESUME_BUILD',
)
gaps = remaining_gap_cells(completion)
unreachable = set(completion.unreachable)
print(f'gaps = {len(gaps)}   unreachable = {len(unreachable)}   '
      f'actionable = {len(gaps - unreachable)}')
print(f'coverage: {completion.as_dict().get("spatial_coverage_ratio"):.3f} '
      f'gate_pass={completion.gate_pass}')

round_cfg = dict(cfg)
round_cfg['patrol'] = dict(cfg.get('patrol') or {})


def run(label, **kw):
    goals = plan_patrol_goals(
        model, map_yaml=map_yaml, cfg=round_cfg, mode='full',
        seed_xy=None, should_stop=None,
        only_cells=gaps, exclude_cells=unreachable, **kw,
    )
    pairs = [(g.spatial_cell, g.yaw_bin) for g in goals]
    stale = [p for p in pairs if p in attempted]
    fresh = [p for p in pairs if p not in attempted]
    print(f'\n--- {label}')
    print(f'    goals returned      : {len(goals)}')
    print(f'    already attempted   : {len(stale)}   <- these would be dropped by the freshness filter')
    print(f'    NEW (never tried)   : {len(fresh)}')
    if len(goals) == 0:
        print('    => session stops with stop_reason=planner_no_new_goals')
    elif not fresh:
        print('    => the whole batch is stale: session stops with stop_reason=planner_no_new_goals')
    else:
        print(f'    => {len(fresh)} usable goals this round')
    return goals, fresh


print('\n================ BEFORE (planner as it was) ================')
_, fresh_before = run('no exclude_pairs  (pre-fix behaviour)')

print('\n================ AFTER  (Fix-1) ================')
_, fresh_after = run('exclude_pairs = session-attempted  (Fix-1)', exclude_pairs=attempted)

print('\n================ VERDICT ================')
ok_stale_repro = len(fresh_before) == 0 or len(fresh_before) < len(fresh_after)
ok_fix = len(fresh_after) > 0 and all(
    (g.spatial_cell, g.yaw_bin) not in attempted
    for g in plan_patrol_goals(
        model, map_yaml=map_yaml, cfg=round_cfg, mode='full', seed_xy=None,
        should_stop=None, only_cells=gaps, exclude_cells=unreachable,
        exclude_pairs=attempted)
)
print(f'  pre-fix batch is starved of fresh goals : {ok_stale_repro}  (fresh={len(fresh_before)})')
print(f'  post-fix batch is entirely fresh        : {ok_fix}  (fresh={len(fresh_after)})')
print(f'  RESULT: {"PASS" if (ok_stale_repro and ok_fix) else "FAIL"}')
