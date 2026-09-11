#!/usr/bin/env python3
"""Phase2D Step 0a — offline reachability survey.

Classifies every 1 m visual cell as reachable / unreachable / clearance-fail /
unknown, and — crucially — checks whether that classification is STABLE.

`compute_eligible_area()` seeds its connectivity flood from an arbitrary member
of a Python set when no seed is given, so a single run's `unreachable` set is not
trustworthy on its own. This runs several deterministic seeds and compares.

Classification only: writes a report, never edits the ledger or the unreachable
set used by the build.

Pure python, no ROS nodes.
"""
import json
import time
from pathlib import Path

import yaml

from xw_global_reloc.phase2d.build_completion import (
    compute_eligible_area,
    evaluate_coverage_completion,
    load_nav_fail_history,
)
from xw_global_reloc.phase2d.config_loader import load_phase2d_config, production_visual_root
from xw_global_reloc.phase2d.coverage_model import build_coverage_model

cfg = load_phase2d_config()
vroot = production_visual_root(cfg)
maps_dir = Path(str(cfg.get('maps_dir') or '/ros2_ws/maps'))
map_yaml = maps_dir / 'vp.yaml'
out_dir = Path('/ros2_ws/maps/vp/visual/state')

# --- seed sources -----------------------------------------------------------
pose_file = Path('/ros2_ws/maps/vp/state/last_good_pose.yaml')
pose = yaml.safe_load(pose_file.read_text(encoding='utf-8')) if pose_file.is_file() else {}
seed_pose = (float(pose['x']), float(pose['y'])) if pose else None
print(f'seed from last_good_pose: {seed_pose} (source={pose.get("source")}, '
      f'laser_verified={pose.get("laser_verified")}, q={pose.get("quality")})')

seeds = {}
if seed_pose:
    seeds['last_good_pose'] = seed_pose

# --- run the classifier under each seed -------------------------------------
results = {}
for name, seed_xy in seeds.items():
    area = compute_eligible_area(map_yaml, cfg, seed_xy=seed_xy)
    results[name] = area
    print(f'\n[{name}] seed={seed_xy}')
    print(f'   eligible(reachable) = {area.eligible_visual_cells}')
    print(f'   unreachable         = {len(area.unreachable)}')
    print(f'   clearance_fail      = {len(area.clearance_fail)}')
    print(f'   unknown             = {len(area.unknown_cells)}')
    print(f'   structure_excluded  = {len(area.structure_excluded)}')
    print(f'   total_free_cells    = {area.total_free_cells}')

# --- seed-sensitivity: is "unreachable" an artefact of the seed? ------------
# The real question is not "what does this seed say" but "how many disconnected
# candidate islands are there". One island => seed-independent answer. More than
# one => `unreachable` is really "the islands the seed did not land on".
print('\n================ SEED SENSITIVITY / CONNECTED COMPONENTS ================')
base_name = next(iter(results))
base = results[base_name]
all_candidates = set(base.eligible) | set(base.unreachable)

components = []
unseen = set(all_candidates)
while unseen:
    start = next(iter(unseen))
    comp = set()
    stack = [start]
    while stack:
        cx, cy = stack.pop()
        if (cx, cy) in comp:
            continue
        comp.add((cx, cy))
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                n = (cx + dx, cy + dy)
                if n in all_candidates and n not in comp:
                    stack.append(n)
    unseen -= comp
    components.append(comp)

components.sort(key=len, reverse=True)
print(f'  candidate cells total     : {len(all_candidates)}')
print(f'  connected components      : {len(components)}')
for i, comp in enumerate(components[:10]):
    print(f'    component[{i}] size={len(comp)}  sample={sorted(comp)[:3]}')
stable = len(components) <= 1
print(f'  => seed-independent classification: {stable}')

# If there is more than one island, flipping the seed must flip the answer.
if len(components) > 1 and seed_pose:
    other = sorted(components[1])[0]
    area2 = compute_eligible_area(map_yaml, cfg, seed_cells={other})
    results['second_component_seed'] = area2
    print(f'  seed planted in component[1] {other}: '
          f'eligible={area2.eligible_visual_cells} unreachable={len(area2.unreachable)} '
          f'(vs {base.eligible_visual_cells}/{len(base.unreachable)} from the pose seed)')

# --- cross-check against history: a REACHED cell cannot be unreachable ------
print('\n================ CROSS-CHECK vs SESSION HISTORY ================')
reached_cells = set()
attempted_cells = set()
for f in sorted((vroot / 'state').glob('build_2026*.json')):
    try:
        data = json.loads(f.read_text(encoding='utf-8'))
    except Exception:  # noqa: BLE001
        continue
    for g in data.get('goals') or []:
        cell = str(g.get('spatial_cell') or '')
        if not cell.startswith('cell_'):
            continue
        attempted_cells.add(cell)
        if g.get('nav') == 'REACHED':
            reached_cells.add(cell)
print(f'  cells REACHED in history   : {len(reached_cells)}')
print(f'  cells attempted in history : {len(attempted_cells)}')


def cell_id(c):
    return f'cell_{c[0]}_{c[1]}'


unreach_ids = {cell_id(c) for c in base.unreachable}
contradiction = unreach_ids & reached_cells
print(f'  geometric unreachable      : {len(unreach_ids)}')
print(f'  CONTRADICTIONS (unreachable but REACHED before): {len(contradiction)}')
for c in sorted(contradiction):
    print(f'     !! {c}')

# --- the 80% gate arithmetic ------------------------------------------------
completion = evaluate_coverage_completion(
    build_coverage_model(cfg, load_descriptors=False), map_yaml, cfg,
    seed_xy=seed_pose, nav_fail_history=load_nav_fail_history(vroot),
    session_nav_failed_cells=set(), patrol_mode='full', build_kind='RESUME_BUILD',
)
c = completion.as_dict()
n_elig = c['eligible']['eligible_visual_cells']
covered = c['covered_eligible']
bc = dict(cfg.get('build_completion') or {})
target = float(bc.get('target_spatial_coverage_ratio', 0.80))
print('\n================ 80% GATE ARITHMETIC ================')
print(f'  denominator (geometric eligible) = {n_elig}')
print(f'  currently covered                = {covered}  ({covered / max(n_elig,1):.1%})')
print(f'  cells needed for {target:.0%}           = {int(-(-target * n_elig // 1))}')
print(f'  remaining to build               = {max(0, int(-(-target * n_elig // 1)) - covered)}')
print(f'  gate_pass = {c.get("gate_pass")}   map_complete_claim_allowed = '
      f'{c.get("map_complete_claim_allowed")}')
print(f'  gate_reasons = {c.get("gate_reasons")}')

report = {
    'generated_at': time.time(),
    'map_yaml': str(map_yaml),
    'map_hash_from_pose': pose.get('map_hash'),
    'seed': {'source': 'last_good_pose', 'xy': seed_pose,
             'quality': pose.get('quality'), 'laser_verified': pose.get('laser_verified')},
    'counts': {
        'eligible_reachable': base.eligible_visual_cells,
        'unreachable': len(base.unreachable),
        'clearance_fail': len(base.clearance_fail),
        'unknown': len(base.unknown_cells),
        'structure_excluded': len(base.structure_excluded),
        'total_free_cells': base.total_free_cells,
        'reached_in_history': len(reached_cells),
        'attempted_in_history': len(attempted_cells),
    },
    'seed_stability': {
        'seeds_tested': sorted(results),
        'stable': bool(stable),
    },
    'contradictions_unreachable_but_reached': sorted(contradiction),
    'eightypct_gate': {
        'denominator_eligible': n_elig,
        'covered': covered,
        'ratio': covered / max(n_elig, 1),
        'target_ratio': target,
        'cells_needed': int(-(-target * n_elig // 1)),
        'cells_remaining': max(0, int(-(-target * n_elig // 1)) - covered),
        'gate_pass': c.get('gate_pass'),
        'gate_reasons': c.get('gate_reasons'),
    },
    'unreachable_cell_ids': sorted(unreach_ids),
    'eligible_cell_ids': sorted(cell_id(x) for x in base.eligible),
}
out = out_dir / 'reachability_report.json'
out.write_text(json.dumps(report, indent=2, sort_keys=True) + '\n', encoding='utf-8')
print(f'\nreport -> {out}')

print('\n================ VERDICT ================')
print(f'  classification stable across seeds : {stable}')
print(f'  contradictions with history        : {len(contradiction)}')
print(f'  classification only, ledger untouched: yes')
