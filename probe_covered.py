#!/usr/bin/env python3
"""Why does the web report covered_eligible=30 while the model has 32 covered?

`covered_eligible = covered & eligible` silently drops any covered cell that is
neither eligible nor unreachable. That is the report black hole the new
`covered_outside_eligible` count is meant to expose — this measures it on the
real DB instead of trusting the arithmetic.

Read-only.
"""
import json
from pathlib import Path

import yaml

from xw_global_reloc.phase2d.build_completion import (
    _active_covered_and_yaw, compute_eligible_area, evaluate_coverage_completion,
)
from xw_global_reloc.phase2d.config_loader import load_phase2d_config, production_visual_root
from xw_global_reloc.phase2d.coverage_model import build_coverage_model

cfg = load_phase2d_config()
vroot = production_visual_root(cfg)
map_yaml = Path(str(cfg.get('maps_dir') or '/ros2_ws/maps')) / 'vp.yaml'
pose = yaml.safe_load(Path('/ros2_ws/maps/vp/state/last_good_pose.yaml').read_text()) or {}
seed_pose = (float(pose['x']), float(pose['y'])) if pose else None


def cid(c):
    return f'cell_{c[0]}_{c[1]}'


model = build_coverage_model(cfg, load_descriptors=False)
covered, yaw_map = _active_covered_and_yaw(model)
area = compute_eligible_area(map_yaml, cfg, seed_xy=seed_pose)

eligible, unreachable = set(area.eligible), set(area.unreachable)
outside = covered - eligible - unreachable
print(f'covered (from the model)      = {len(covered)}')
print(f'eligible                      = {len(eligible)}')
print(f'unreachable                   = {len(unreachable)}')
print(f'covered & eligible            = {len(covered & eligible)}   <- what the web shows')
print(f'covered OUTSIDE eligible      = {len(outside)}   <- the black hole')
print(f'  {sorted(map(cid, outside))}')
print(f'covered & unreachable         = {len(covered & unreachable)}')
print()
print('classification of each covered-outside cell:')
for c in sorted(outside):
    tags = []
    if c in area.clearance_fail:
        tags.append('clearance_fail')
    if c in area.structure_excluded:
        tags.append('structure_excluded')
    if c in area.unknown_cells:
        tags.append('unknown')
    if c in area.free_cells:
        tags.append('free')
    print(f'  {cid(c):14s}  ' + (', '.join(tags) or 'NOT CLASSIFIED AT ALL'))

print()
comp = evaluate_coverage_completion(
    model, map_yaml, cfg, seed_xy=seed_pose,
    nav_fail_history={}, session_nav_failed_cells=set(),
    patrol_mode='full', build_kind='RESUME_BUILD')
d = comp.as_dict()
print('=== new counts (needs the deployed module) ===')
print(json.dumps(d.get('counts'), indent=2, sort_keys=True))
print('covered_eligible   =', d.get('covered_eligible'))
print('unvisited/under/yaw=', len(comp.unvisited), len(comp.under_covered), len(comp.yaw_insufficient))
print('spatial_ratio      =', d.get('spatial_coverage_ratio'))
print('eligible block     =', json.dumps(d['eligible'], sort_keys=True)[:400])
