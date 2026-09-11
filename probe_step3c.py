#!/usr/bin/env python3
"""Step 3c — freeze the fixture facts and record BEFORE/AFTER numbers.

Read-only: never edits the ledger, the DB or the unreachable set. Prints one
JSON blob so the before and after runs can be diffed field by field.

The component analysis below is written inline on purpose — it must produce the
same answer against the OLD module (which has no connectivity helpers) and the
NEW one, otherwise the "before" baseline would not exist.
"""
import hashlib
import json
import math
import sys
from pathlib import Path

import yaml

from xw_global_reloc.phase2d.build_completion import (
    _active_covered_and_yaw, compute_eligible_area,
)
from xw_global_reloc.phase2d.config_loader import load_phase2d_config, production_visual_root
from xw_global_reloc.phase2d.coverage_model import build_coverage_model

LABEL = sys.argv[1] if len(sys.argv) > 1 else 'run'
NEIGH = [(dx, dy) for dx in (-1, 0, 1) for dy in (-1, 0, 1) if (dx, dy) != (0, 0)]

cfg = load_phase2d_config()
vroot = production_visual_root(cfg)
map_yaml = Path(str(cfg.get('maps_dir') or '/ros2_ws/maps')) / 'vp.yaml'
cell_size = float((cfg.get('coverage') or {}).get('cell_size_m', 1.0))
pose = yaml.safe_load(Path('/ros2_ws/maps/vp/state/last_good_pose.yaml').read_text()) or {}
seed_pose = (float(pose['x']), float(pose['y'])) if pose else None


def digest(cells):
    return hashlib.sha256(repr(sorted(cells)).encode()).hexdigest()[:16]


def comps_of(cand):
    """8-neighbour components, deterministic order (-size, min(cell))."""
    unseen, comps = set(cand), []
    while unseen:
        start = min(unseen)
        comp, stack = {start}, [start]
        while stack:
            cx, cy = stack.pop()
            for dx, dy in NEIGH:
                n = (cx + dx, cy + dy)
                if n in unseen and n not in comp:
                    comp.add(n)
                    stack.append(n)
        unseen -= comp
        comps.append(comp)
    comps.sort(key=lambda c: (-len(c), min(c)))
    return comps


model = build_coverage_model(cfg, load_descriptors=False)
covered, _ = _active_covered_and_yaw(model)
pose_cell = (int(math.floor(seed_pose[0] / cell_size)), int(math.floor(seed_pose[1] / cell_size)))

prod = compute_eligible_area(map_yaml, cfg, seed_cells=covered or None)
cand = set(prod.eligible) | set(prod.unreachable)
comps = comps_of(cand)
geo_unreach = set().union(*comps[1:]) if len(comps) > 1 else set()

out = {
    'label': LABEL,
    'map_yaml': str(map_yaml),
    'cell_size': cell_size,
    'covered_cells': len(covered),
    'pose_xy': seed_pose,
    'pose_cell': list(pose_cell),
    'facts': {
        'candidates': len(cand),
        'min_candidate': list(min(cand)),
        'min_is_orphan_(-13,-6)': min(cand) == (-13, -6),
        'min_in_largest_component': min(cand) in comps[0],
        'largest_component_anchor[min(comps[0])]': list(min(comps[0])),
        'component_count': len(comps),
        'component_sizes': [len(c) for c in comps[:10]],
        'geometric_unreachable': sorted(geo_unreach),
        'pose_cell_is_candidate': pose_cell in cand,
        'pose_cell_in_clearance_fail': pose_cell in prod.clearance_fail,
        'pose_cell_in_structure_excluded': pose_cell in prod.structure_excluded,
        'nearest_candidate_to_pose': list(min(
            cand, key=lambda c: ((c[0] - pose_cell[0]) ** 2 + (c[1] - pose_cell[1]) ** 2, c[0], c[1]))),
    },
    'runs': {},
}

cases = {
    'production_seed_cells': dict(seed_cells=covered or None),
    'empty_db_seed_xy': dict(seed_xy=seed_pose),
    'no_seed_at_all': dict(),
}
for tag, kw in cases.items():
    try:
        a = compute_eligible_area(map_yaml, cfg, **kw)
        sr = getattr(a, 'seed_resolution', None)
        out['runs'][tag] = {
            'eligible': a.eligible_visual_cells,
            'unreachable_count': len(a.unreachable),
            'unreachable_sorted': sorted(a.unreachable),
            'eligible_digest': digest(a.eligible),
            'unreachable_digest': digest(a.unreachable),
            'matches_geometric': set(a.unreachable) == geo_unreach,
            'seed_source': (sr or {}).get('seed_source', 'N/A(no field)'),
            'snapped_from': (sr or {}).get('snapped_from', 'N/A(no field)'),
            'internal_error': (sr or {}).get('internal_error', 'N/A(no field)'),
        }
    except Exception as exc:  # noqa: BLE001
        out['runs'][tag] = {'EXCEPTION': f'{type(exc).__name__}: {exc}'}

print(json.dumps(out, indent=2, sort_keys=True, default=str))
