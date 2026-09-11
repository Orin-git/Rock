#!/usr/bin/env python3
"""Tests for the Phase2D seed-resolution / connectivity fix (2026-09-11).

The defect being pinned: `compute_eligible_area()` used to enqueue a seed that
was not a candidate cell and drop it at pop time (`continue`), leaving no trace.
An empty flood then meant `unreachable = candidates` — every cell on the map
condemned, permanently, because the orchestrator forwards `unreachable` as
`exclude_cells`. On an empty visual DB that is the normal path (the AMCL pose
sits in a clearance-fail cell next to the charger), so the build session would
stop silently having built nothing.

Run inside the container (fixtures present, sections A-D all run):
  python3 /ros2_ws/test_seed_resolution.py
Locally via the /tmp/p2d_shim package shim (section D self-skips).
"""
import hashlib
import json
import math
import os
import random
import subprocess
import sys
from collections import deque
from pathlib import Path

from xw_global_reloc.phase2d.build_completion import (
    _connected_components,
    _flood,
    _largest_component_anchor,
    _nearest_candidate,
    _resolve_seeds,
)

FAILED = 0


def check(label, got, want):
    global FAILED
    ok = got == want
    if not ok:
        FAILED += 1
    print(f"  [{'PASS' if ok else 'FAIL'}] {label}: got={got!r} want={want!r}")


# ---------------------------------------------------------------------------
# The pre-fix implementation, replicated verbatim (build_completion.py:206-238
# before 2026-09-11). Kept in executable form so the defect — and the proof
# that the new code does not reproduce it — cannot rot into a comment.
# ---------------------------------------------------------------------------
def _legacy_resolve(candidates, seed_cells, seed_xy, cell_size):
    seeds = []
    if seed_cells:
        seeds = [c for c in seed_cells if c in candidates] or list(seed_cells)[:1]
    if not seeds and seed_xy is not None:
        sx, sy = float(seed_xy[0]), float(seed_xy[1])
        seeds = [(int(math.floor(sx / cell_size)), int(math.floor(sy / cell_size)))]
    if not seeds and candidates:
        seeds = [next(iter(candidates))]
    return seeds


def _legacy_flood(candidates, seeds):
    reachable = set()
    if seeds:
        q = deque()
        seen = set()
        for s in seeds:
            if s not in seen:
                seen.add(s)
                q.append(s)
        while q:
            cx, cy = q.popleft()
            if (cx, cy) not in candidates:
                continue
            reachable.add((cx, cy))
            for dx in (-1, 0, 1):
                for dy in (-1, 0, 1):
                    if dx == 0 and dy == 0:
                        continue
                    n = (cx + dx, cy + dy)
                    if n in candidates and n not in seen:
                        seen.add(n)
                        q.append(n)
    return reachable


# A counterexample map: a 5-cell corridor plus one orphan pocket whose x is the
# smallest in the whole set. This is deliberately NOT how the production map
# looks — measured on 189 on 2026-09-11, min(candidates) is (-24,-8), which sits
# inside the 402-cell main component, so `min()` happens to agree with the anchor
# there. The anchor is still the rule: agreement on one map is a coincidence, not
# a property, and the map below is a legal map on which the two diverge.
BIG = {(x, 0) for x in range(5)}
ORPHAN = (-13, -6)
MINI = BIG | {ORPHAN}

print("=== A. connectivity helpers ===")
check('components of empty set', _connected_components(set()), [])
check('single cell -> one component', _connected_components({(2, 3)}), [{(2, 3)}])
check('diagonal counts as adjacent', len(_connected_components({(0, 0), (1, 1)})), 1)
check('gap of one cell splits', len(_connected_components({(0, 0), (2, 0)})), 2)

check('min(MINI) is the orphan', min(MINI), ORPHAN)
check('anchor avoids the orphan', _largest_component_anchor(MINI), (0, 0))
check('flood from min(MINI) finds only the orphan', _flood(MINI, [min(MINI)]), {ORPHAN})
check('flood from the anchor finds the corridor', len(_flood(MINI, [_largest_component_anchor(MINI)])), 5)
check('component order: largest first', [len(c) for c in _connected_components(MINI)], [5, 1])
check('empty candidates -> no anchor', _largest_component_anchor(set()), None)

check('nearest: equidistant breaks by (x, y)',
      _nearest_candidate((0, 0), {(-1, 0), (1, 0), (0, -1), (0, 1)}), (-1, 0))
check('nearest: the production pose cell -2,2 -> -1,2',
      _nearest_candidate((-2, 2), {(-1, 2), ORPHAN, (5, 5)}), (-1, 2))
check('nearest: empty -> None', _nearest_candidate((0, 0), set()), None)

check('flood with no seeds', _flood(MINI, []), set())
check('flood ignores a non-candidate seed', _flood(MINI, [(99, 99)]), set())
check('flood seed order is irrelevant',
      _flood(MINI, [(0, 0), ORPHAN]), _flood(MINI, [ORPHAN, (0, 0)]))

print("\n=== A2. determinism under input reordering (50 shuffles) ===")
rng = random.Random(0)
base = (sorted(_connected_components(MINI), key=lambda c: min(c)),
        _largest_component_anchor(MINI),
        sorted(_flood(MINI, _resolve_seeds(MINI, None, None, 1.0)[0])))
stable = True
for _ in range(50):
    sh = list(MINI)
    rng.shuffle(sh)
    shuffled = set(sh)
    got = (sorted(_connected_components(shuffled), key=lambda c: min(c)),
           _largest_component_anchor(shuffled),
           sorted(_flood(shuffled, _resolve_seeds(shuffled, None, None, 1.0)[0])))
    if got != base:
        stable = False
        break
check('50 shuffled input orders give identical results', stable, True)

print("\n=== B. _resolve_seeds policy ===")
s, a = _resolve_seeds(MINI, {(0, 0), (3, 0)}, None, 1.0)
check('all seeds candidates -> seeds kept', s, [(0, 0), (3, 0)])
check('  seed_source', a['seed_source'], 'seed_cells')
check('  nothing dropped', a['dropped_non_candidate'], [])
check('  nothing snapped', a['snapped_from'], [])

# NEUTRALITY: the mixed case must equal the old filtered expression exactly.
s, a = _resolve_seeds(MINI, {(0, 0), (99, 99), (3, 0)}, None, 1.0)
legacy = [c for c in sorted({(0, 0), (99, 99), (3, 0)}) if c in MINI]
check('mixed: seeds == legacy filtered set', s, legacy)
check('mixed: flood == legacy flood', _flood(MINI, s), _legacy_flood(MINI, legacy))
check('mixed: the off-map seed is recorded, not hidden', a['dropped_non_candidate'], [[99, 99]])

# The shape of the production defect: an off-map cell next to the corridor.
# (On the real map the pose cell is (-2,2) and the nearest candidate (-1,2);
# in this miniature the nearest is (0,0) — same defect, different geometry.)
s, a = _resolve_seeds(MINI, {(-2, 2)}, None, 1.0)
check('no candidate seed -> snapped (the empty-DB case)', s, [(0, 0)])
check('  seed_source marked', a['seed_source'], 'seed_cells_snapped')
check('  snap recorded as from/to', a['snapped_from'], [{'from': [-2, 2], 'to': [0, 0]}])
check('  contradiction noted', 'seed_map_contradiction' in a['notes'], True)

s, a = _resolve_seeds(MINI, None, (-2.4, 2.4), 1.0)
check('pose path: cell (-3, 2) snaps to a candidate', len(s), 1)
check('  every snapped seed is a candidate', all(c in MINI for c in s), True)
check('  source says seed_xy_snapped', a['seed_source'], 'seed_xy_snapped')

s, a = _resolve_seeds(MINI, None, None, 1.0)
check('no evidence at all -> largest-component anchor', s, [(0, 0)])
check('  marked no_seed_evidence', a['notes'], ['no_seed_evidence'])

s, a = _resolve_seeds(set(), {(0, 0)}, None, 1.0)
check('empty candidates -> no seeds, no exception', s, [])
check('  seed_unresolved flag', a['seed_unresolved'], True)
check('  note', a['notes'], ['no_candidates'])

# A valid but non-largest seed must be honoured: the old code did this, and it
# is how `unreachable` stays a real geometric fact rather than "everything but
# the biggest blob".
s, a = _resolve_seeds(MINI, {ORPHAN}, None, 1.0)
check('a valid non-largest seed is still honoured', s, [ORPHAN])
check('  flood from it is the orphan alone', _flood(MINI, s), {ORPHAN})

# Q3 neutrality: the pose must NOT be unioned into a seed set that already has
# valid cells, or `eligible` would move with the parking spot.
s, a = _resolve_seeds(MINI, {(0, 0)}, (-13.4, -6.4), 1.0)
check('pose near the orphan does not join the seed set', s, [(0, 0)])
check('  so the orphan stays unreachable', MINI - _flood(MINI, s), {ORPHAN})

print("\n=== C. the defect, in executable form ===")
old_seeds = _legacy_resolve(MINI, None, (-2.4, 2.4), 1.0)
check('OLD: pose seed is not a candidate', old_seeds, [(-3, 2)])
check('OLD: flood collapses to nothing', _legacy_flood(MINI, old_seeds), set())
check('OLD: the whole map becomes unreachable', MINI - _legacy_flood(MINI, old_seeds), MINI)
new_seeds, _ = _resolve_seeds(MINI, None, (-2.4, 2.4), 1.0)
check('NEW: the flood reaches the corridor', len(MINI - _flood(MINI, new_seeds)), 1)
check('NEW: only the genuine orphan stays unreachable', MINI - _flood(MINI, new_seeds), {ORPHAN})

print("\n=== D. PYTHONHASHSEED independence ===")
if sys.argv[1:2] == ['--hashdump']:
    # Child mode: emit one digest and exit. Must not fall through — the parent
    # calls _hashdump() at module level, so a child that kept going would fork
    # copies of itself forever.
    print(json.dumps({'digest': hashlib.sha256(
        repr(sorted(_flood(MINI, _resolve_seeds(MINI, None, None, 1.0)[0]))).encode()
    ).hexdigest()}))
    sys.exit(0)


def _hashdump(seed):
    # Keep the existing PYTHONPATH (the local shim / the installed package) and
    # add this file's directory, so the child process imports what we imported.
    pp = os.pathsep.join(p for p in (str(Path(__file__).parent),
                                     os.environ.get('PYTHONPATH', '')) if p)
    env = {**os.environ, 'PYTHONHASHSEED': str(seed), 'PYTHONPATH': pp}
    out = subprocess.run([sys.executable, __file__, '--hashdump'],
                         capture_output=True, text=True, env=env)
    for line in out.stdout.splitlines():
        if line.startswith('{'):
            return json.loads(line)['digest']
    return f'ERROR:{out.stderr.strip()[:120]}'


digests = [_hashdump(s) for s in (0, 1, 2)]
check('identical results under PYTHONHASHSEED 0/1/2', len(set(digests)), 1)
for s, d in zip((0, 1, 2), digests):
    print(f'       PYTHONHASHSEED={s} -> {d[:16]}')

print("\n=== E. real map / DB fixtures ===")
MAP_YAML = Path('/ros2_ws/maps/vp.yaml')
try:
    from xw_global_reloc.phase2d.config_loader import load_phase2d_config, production_visual_root
    from xw_global_reloc.phase2d.coverage_model import build_coverage_model
    from xw_global_reloc.phase2d.build_completion import (
        _active_covered_and_yaw, evaluate_coverage_completion, remaining_gap_cells,
    )
    import xw_global_reloc.phase2d.build_completion as bc
    HAVE_PKG = True
except Exception as exc:  # noqa: BLE001
    HAVE_PKG = False
    print(f'  [SKIP] package not importable: {exc!r}')

if not HAVE_PKG or not MAP_YAML.is_file():
    print(f'  [SKIP] fixtures absent ({MAP_YAML}) — run this inside the container')
else:
    cfg = load_phase2d_config()
    vroot = production_visual_root(cfg)
    covered, _ = _active_covered_and_yaw(build_coverage_model(cfg, load_descriptors=False))
    print(f'  fixtures: covered cells = {len(covered)}')

    prod = bc.compute_eligible_area(MAP_YAML, cfg, seed_cells=covered or None)
    print(f'  production: eligible={prod.eligible_visual_cells} unreachable={len(prod.unreachable)} '
          f'source={prod.seed_resolution.get("seed_source")}')
    if len(covered) == 30:
        check('production eligible is unchanged (402)', prod.eligible_visual_cells, 402)
        check('production unreachable is unchanged (1)', len(prod.unreachable), 1)
        check('production: unreachable == {(-13,-6)}', prod.unreachable, {(-13, -6)})
    else:
        print(f'  [SKIP] covered={len(covered)} != 30 — the DB has moved on, so freezing '
              f'402/1 would be wrong. Asserting the DB-independent invariant instead.')
    check('invariant: every candidate is eligible or unreachable (403)',
          prod.eligible_visual_cells + len(prod.unreachable), 403)

    # Pin the MEASUREMENT, not the coincidence. `min == anchor` on this map is a
    # fact worth printing (it is what falsified the "min always picks the orphan"
    # argument), but asserting it would turn a coincidence into a spec — the DB
    # grows and it will legitimately stop holding.
    cand_ids = set(prod.eligible) | set(prod.unreachable)
    measured_min = min(cand_ids)
    measured_anchor = bc._largest_component_anchor(cand_ids)
    print(f'  measured: min(candidates)={measured_min}  anchor={measured_anchor}  '
          f'agree={measured_min == measured_anchor}')
    check('anchor is one of the candidates', measured_anchor in cand_ids, True)
    check('anchor lies in the largest component',
          measured_anchor in bc._connected_components(cand_ids)[0], True)

    empty = bc.compute_eligible_area(MAP_YAML, cfg, seed_xy=(-1.5737449332173814, 2.479320081004552))
    print(f'  empty DB : eligible={empty.eligible_visual_cells} unreachable={len(empty.unreachable)} '
          f'source={empty.seed_resolution.get("seed_source")} '
          f'snapped_from={empty.seed_resolution.get("snapped_from")}')
    check('empty DB eligible == production eligible', empty.eligible_visual_cells, 402)
    check('empty DB unreachable == production unreachable', empty.unreachable, {(-13, -6)})
    check('empty DB snapped the pose cell', empty.seed_resolution.get('seed_source'), 'seed_xy_snapped')
    check('empty DB audit has no internal error',
          empty.seed_resolution.get('internal_error', False), False)

    class _EmptyModel:
        def cell_summaries(self):
            return []

    comp = evaluate_coverage_completion(
        _EmptyModel(), MAP_YAML, cfg, seed_xy=(-1.5737449332173814, 2.479320081004552),
        nav_fail_history={}, session_nav_failed_cells=set(),
        patrol_mode='full', build_kind='RESUME_BUILD')
    gaps = remaining_gap_cells(comp)
    print(f'  empty DB completion: gaps={len(gaps)} gate_pass={comp.gate_pass} '
          f'claim={comp.map_complete_claim_allowed}')
    check('an empty DB yields actionable gaps (was 0)', len(gaps) > 300, True)
    check('an empty DB never claims completion', comp.map_complete_claim_allowed, False)
    check('counts carry candidate_cells', comp.counts.get('candidate_cells'), 403)
    check('counts carry covered_outside_eligible', comp.counts.get('covered_outside_eligible'), 0)

    d = comp.as_dict()
    check('as_dict exposes seed_resolution', 'seed_resolution' in d['eligible'], True)
    check('as_dict exposes connectivity', 'connectivity' in d['eligible'], True)
    check('as_dict round-trips through JSON', json.loads(json.dumps(d)) == d, True)

    # The guard: break the flood outright and require a loud, recorded fallback
    # that does NOT condemn the map. A broken flood means reachability is
    # UNKNOWN — and "unknown" must never be spent as an irreversible
    # exclude-lock on every cell on the map.
    real_flood, bc._flood = bc._flood, lambda *a, **k: set()
    try:
        forced = bc.compute_eligible_area(MAP_YAML, cfg, seed_cells=covered or None)
    finally:
        bc._flood = real_flood
    check('guard: internal_error is flagged', forced.seed_resolution.get('internal_error'), True)
    check('guard: resolver note recorded',
          'INTERNAL_seed_resolution_failed' in forced.seed_resolution.get('notes', []), True)
    check('guard: flood note recorded',
          'INTERNAL_flood_unavailable' in forced.seed_resolution.get('notes', []), True)
    check('guard: nothing is excluded on a broken flood', forced.unreachable, set())
    check('guard: every candidate stays eligible', forced.eligible_visual_cells, 403)
    check('guard: reachability marked unknown',
          forced.connectivity.get('reachability_unknown'), True)
    check('guard: real components still reported', forced.connectivity.get('component_count'), 2)

print()
print(f"RESULT: {'ALL PASS' if FAILED == 0 else str(FAILED) + ' FAILURE(S)'}")
sys.exit(1 if FAILED else 0)
