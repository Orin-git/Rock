#!/usr/bin/env python3
"""Step 0a forensic: where does 'unreachable' come from, and is it stable?

Read-only. Writes a report; never edits the ledger, the DB or the unreachable set.
"""
import json
from collections import deque
from pathlib import Path

import yaml

from xw_global_reloc.phase2d.build_completion import (
    _active_covered_and_yaw, compute_eligible_area, load_nav_fail_history,
    remaining_gap_cells, evaluate_coverage_completion,
)
from xw_global_reloc.phase2d.config_loader import load_phase2d_config, production_visual_root
from xw_global_reloc.phase2d.coverage_model import build_coverage_model

cfg = load_phase2d_config()
vroot = production_visual_root(cfg)
map_yaml = Path(str(cfg.get('maps_dir') or '/ros2_ws/maps')) / 'vp.yaml'
cell_size = float((cfg.get('coverage') or {}).get('cell_size_m', 1.0))

pose = yaml.safe_load((Path('/ros2_ws/maps/vp/state/last_good_pose.yaml')).read_text()) or {}
seed_pose = (float(pose['x']), float(pose['y'])) if pose else None
model = build_coverage_model(cfg, load_descriptors=False)
covered, yaw_map = _active_covered_and_yaw(model)

print('=' * 78)
print('A. WHICH CELL IS THE ROBOT IN?')
print('=' * 78)
import math
pose_cell = (int(math.floor(seed_pose[0] / cell_size)), int(math.floor(seed_pose[1] / cell_size)))
print(f'  pose            = {seed_pose}')
print(f'  pose cell       = {pose_cell}')
print(f'  covered cells   = {len(covered)}')
cov_comps = {}
seen = set()
for start in covered:
    if start in seen:
        continue
    comp, st = set(), [start]
    while st:
        c = st.pop()
        if c in comp:
            continue
        comp.add(c)
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                n = (c[0] + dx, c[1] + dy)
                if n in covered and n not in comp:
                    st.append(n)
    seen |= comp
    cov_comps[tuple(sorted(comp)[0])] = comp
print(f'  covered cells form {len(cov_comps)} connected group(s); '
      f'sizes={sorted((len(v) for v in cov_comps.values()), reverse=True)[:8]}')

print()
print('=' * 78)
print('B. THE THREE CLASSIFICATIONS OF THE SAME MAP')
print('=' * 78)
prod = compute_eligible_area(map_yaml, cfg, seed_xy=seed_pose, seed_cells=covered or None)
empty_db = compute_eligible_area(map_yaml, cfg, seed_xy=seed_pose, seed_cells=None)
none_seed = compute_eligible_area(map_yaml, cfg, seed_cells=None)

def show(tag, a):
    print(f'  [{tag}] eligible={a.eligible_visual_cells:4d}  unreachable={len(a.unreachable):4d} '
          f'clearance_fail={len(a.clearance_fail):3d}  structure_excl={len(a.structure_excluded):3d} '
          f'unknown={len(a.unknown_cells):3d}  free={a.total_free_cells}')

show('P  production (seed=covered)', prod)
show('E  empty DB   (seed=pose)   ', empty_db)
show('N  no seed at all           ', none_seed)
print(f'  pose cell {pose_cell} membership: '
      f'candidate={pose_cell in (prod.eligible | prod.unreachable)}  '
      f'clearance_fail={pose_cell in prod.clearance_fail}  '
      f'structure_excluded={pose_cell in prod.structure_excluded}  '
      f'unknown={pose_cell in prod.unknown_cells}  '
      f'free={pose_cell in prod.free_cells}')

# nearest candidate to the pose cell — how far is the robot from anything drivable?
cand = prod.eligible | prod.unreachable
near = sorted(cand, key=lambda c: (c[0]-pose_cell[0])**2 + (c[1]-pose_cell[1])**2)[:3]
print(f'  nearest candidate cells to pose cell: '
      f'{[(c, round(math.hypot(c[0]-pose_cell[0], c[1]-pose_cell[1]), 1)) for c in near]}')

print()
print('=' * 78)
print('C. IS "UNREACHABLE" SEED-DEPENDENT?  (components on the candidate set)')
print('=' * 78)
comps = []
unseen = set(cand)
while unseen:
    start = next(iter(unseen))
    comp, st = set(), [start]
    while st:
        c = st.pop()
        if c in comp:
            continue
        comp.add(c)
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                n = (c[0] + dx, c[1] + dy)
                if n in cand and n not in comp:
                    st.append(n)
    unseen -= comp
    comps.append(comp)
comps.sort(key=len, reverse=True)
print(f'  candidates={len(cand)}  connected components={len(comps)}  '
      f'sizes={[len(c) for c in comps[:8]]}')
geo_unreach = set().union(*comps[1:]) if len(comps) > 1 else set()
print(f'  geometric unreachable (all but largest component) = {len(geo_unreach)}')
print(f'       {sorted(geo_unreach)[:12]}')
print(f'  production unreachable                            = {len(prod.unreachable)}')
print(f'       {sorted(prod.unreachable)[:12]}')
print(f'  => production == geometric? {set(prod.unreachable) == geo_unreach}')

print()
print('=' * 78)
print('D. EVIDENCE CHECK: what does the robot\'s own history say?')
print('=' * 78)
reached, attempted = set(), set()
for f in sorted((vroot / 'state').glob('build_2026*.json')):
    try:
        d = json.loads(f.read_text())
    except Exception:
        continue
    for g in d.get('goals') or []:
        cid = str(g.get('spatial_cell') or '')
        if not cid.startswith('cell_'):
            continue
        attempted.add(cid)
        if g.get('nav') == 'REACHED':
            reached.add(cid)
print(f'  history: {len(attempted)} cells attempted, {len(reached)} REACHED')
for tag, s in (('production unreachable', {f'cell_{c[0]}_{c[1]}' for c in prod.unreachable}),
               ('geometric  unreachable', {f'cell_{c[0]}_{c[1]}' for c in geo_unreach})):
    bad = s & reached
    print(f'  {tag}: {len(s)} cells -> contradicted by history (REACHED before): {len(bad)}  {sorted(bad)[:8]}')

print()
print('=' * 78)
print('E. WHAT THE ORCHESTRATOR SEES ON AN EMPTY DB')
print('=' * 78)
class _EmptyModel:
    def cell_summaries(self):
        return []
comp_e = evaluate_coverage_completion(
    _EmptyModel(), map_yaml, cfg, seed_xy=seed_pose, nav_fail_history={},
    session_nav_failed_cells=set(), patrol_mode='full', build_kind='RESUME_BUILD')
d = comp_e.as_dict()
print(f'  eligible            = {comp_e.eligible.eligible_visual_cells}')
print(f'  unreachable         = {len(comp_e.unreachable)}   -> orchestrator exclude_cells')
g = remaining_gap_cells(comp_e)
print(f'  gaps                = {len(g)}')
print(f'  unvisited/under/yaw = {len(comp_e.unvisited)}/{len(comp_e.under_covered)}/{len(comp_e.yaw_insufficient)}')
print(f'  map_complete_claim  = {d.get("map_complete_claim_allowed")}')
print(f'  gate_reasons        = {d.get("gate_reasons")}')
print(f'  => planner gets only_cells=set() -> returns [] -> fresh empty ->')
print(f'     stop_reason=planner_no_new_goals  (SILENT STOP, nothing built)')

print()
print('=' * 78)
print('F. EVERY SESSION WITH goals=0, IN FULL')
print('=' * 78)
for f in sorted((vroot / 'state').glob('build_2026*.json')):
    try:
        d = json.loads(f.read_text())
    except Exception:
        continue
    if len(d.get('goals') or []) != 0:
        continue
    print(f'  --- {f.name}  ({f.stat().st_size} B)')
    for k, v in sorted(d.items()):
        if k in ('goals', 'candidate_frames', 'descriptors'):
            continue
        s = json.dumps(v, ensure_ascii=False)
        print(f'      {k} = {s[:190]}')
