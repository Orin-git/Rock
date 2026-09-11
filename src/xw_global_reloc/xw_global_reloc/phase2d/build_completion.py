"""Eligible visual coverage + Full-Build completion gate (Phase2D-C2.1).

Does NOT treat raw free-space pixel count as the build target.
Eligible = known-free + clearance + connected + (optional) near structure.
"""

from __future__ import annotations

import json
import logging
import math
from collections import deque
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional, Set, Tuple

import cv2
import numpy as np
import yaml

from xw_global_reloc.phase2d.coverage_model import CoverageModel
from xw_global_reloc.phase2d.patrol_planner import (
    _cell_center,
    _is_clear_goal,
    erode_free,
    load_free_mask,
    map_to_world,
    world_to_map,
)
from xw_global_reloc.phase2d.spatial import parse_spatial_cell

Cell = Tuple[int, int]

LOGGER = logging.getLogger(__name__)

# Gap / cell classification labels (V1)
COVERED = 'COVERED'
UNDER_COVERED = 'UNDER_COVERED'
UNVISITED = 'UNVISITED'
NAV_FAILED = 'NAV_FAILED'
NAV_FAILED_RETRYABLE = 'NAV_FAILED_RETRYABLE'
UNREACHABLE = 'UNREACHABLE'
UNSAFE_CLEARANCE = 'UNSAFE_CLEARANCE'
UNKNOWN_MAP = 'UNKNOWN_MAP'
YAW_INSUFFICIENT = 'YAW_INSUFFICIENT'


@dataclass
class EligibleArea:
    cell_size_m: float
    total_free_cells: int
    eligible_visual_cells: int
    excluded_unknown: int
    excluded_clearance: int
    excluded_unreachable: int
    excluded_structure: int
    eligible: Set[Cell] = field(default_factory=set)
    free_cells: Set[Cell] = field(default_factory=set)
    clearance_fail: Set[Cell] = field(default_factory=set)
    unreachable: Set[Cell] = field(default_factory=set)
    unknown_cells: Set[Cell] = field(default_factory=set)
    structure_excluded: Set[Cell] = field(default_factory=set)
    # Audit trail for the connectivity flood. `unreachable` is an irreversible
    # lock (the orchestrator passes it as exclude_cells forever), so how it was
    # derived must be inspectable after the fact.
    seed_resolution: Dict[str, Any] = field(default_factory=dict)
    connectivity: Dict[str, Any] = field(default_factory=dict)

    def as_dict(self) -> Dict[str, Any]:
        return {
            'cell_size_m': self.cell_size_m,
            'total_free_cells': self.total_free_cells,
            'eligible_visual_cells': self.eligible_visual_cells,
            'excluded_unknown': self.excluded_unknown,
            'excluded_clearance': self.excluded_clearance,
            'excluded_unreachable': self.excluded_unreachable,
            'excluded_structure': self.excluded_structure,
            'eligible_cell_ids': sorted(f'cell_{x}_{y}' for x, y in self.eligible),
            'seed_resolution': dict(self.seed_resolution),
            'connectivity': dict(self.connectivity),
        }


@dataclass
class CoverageCompletion:
    eligible: EligibleArea
    covered_cells: Set[Cell]
    covered_eligible: Set[Cell]
    under_covered: Set[Cell]
    unvisited: Set[Cell]
    yaw_sufficient: Set[Cell]
    yaw_insufficient: Set[Cell]
    nav_failed_retryable: Set[Cell]
    unreachable: Set[Cell]
    unsafe_clearance: Set[Cell]
    unknown_map: Set[Cell]
    spatial_coverage_ratio: float
    yaw_completeness_ratio: float
    unresolved_nav_fail_ratio: float
    gate_pass: bool
    gate_reasons: List[str]
    map_complete_claim_allowed: bool
    cell_status: Dict[str, str] = field(default_factory=dict)
    counts: Dict[str, int] = field(default_factory=dict)

    def as_dict(self) -> Dict[str, Any]:
        return {
            'eligible': {
                'total_free_cells': self.eligible.total_free_cells,
                'eligible_visual_cells': self.eligible.eligible_visual_cells,
                'excluded_unknown': self.eligible.excluded_unknown,
                'excluded_clearance': self.eligible.excluded_clearance,
                'excluded_unreachable': self.eligible.excluded_unreachable,
                'excluded_structure': self.eligible.excluded_structure,
                # Hand-written subset of EligibleArea.as_dict() — new audit
                # fields must be repeated here or they never reach the
                # orchestrator, which persists exactly this dict.
                'seed_resolution': dict(self.eligible.seed_resolution),
                'connectivity': dict(self.eligible.connectivity),
            },
            'covered_eligible': len(self.covered_eligible),
            'spatial_coverage_ratio': self.spatial_coverage_ratio,
            'yaw_sufficient': len(self.yaw_sufficient),
            'yaw_insufficient': len(self.yaw_insufficient),
            'yaw_completeness_ratio': self.yaw_completeness_ratio,
            'unvisited': len(self.unvisited),
            'under_covered': len(self.under_covered),
            'nav_failed_retryable': len(self.nav_failed_retryable),
            'unreachable': len(self.unreachable),
            'unsafe_clearance': len(self.unsafe_clearance),
            'unknown_map': len(self.unknown_map),
            'unresolved_nav_fail_ratio': self.unresolved_nav_fail_ratio,
            'gate_pass': self.gate_pass,
            'gate_reasons': list(self.gate_reasons),
            'map_complete_claim_allowed': self.map_complete_claim_allowed,
            'counts': dict(self.counts),
            'cell_status': dict(self.cell_status),
        }


def _completion_cfg(cfg: Dict[str, Any]) -> Dict[str, Any]:
    return dict(cfg.get('build_completion') or {})


# ---------------------------------------------------------------------------
# Connectivity helpers. Pure functions — no map, no config — so the seed and
# flood logic can be unit-tested directly.
# ---------------------------------------------------------------------------

# Single source of truth for adjacency: the flood and the component pass must
# agree, or "largest component" could name a region the flood treats
# differently and the audit would lie.
_NEIGHBORS8: Tuple[Tuple[int, int], ...] = tuple(
    (dx, dy) for dx in (-1, 0, 1) for dy in (-1, 0, 1) if (dx, dy) != (0, 0)
)


def _connected_components(candidates: Set[Cell]) -> List[Set[Cell]]:
    """8-neighbour connected components, in a deterministic total order.

    Ordered by size descending, ties broken by the component's smallest cell,
    so the result never depends on set iteration order or PYTHONHASHSEED.
    Iterative on purpose: a 400-cell snake corridor would blow the recursion
    limit, which would be a new failure class of its own.
    """
    unseen = set(candidates)
    comps: List[Set[Cell]] = []
    while unseen:
        start = min(unseen)
        comp: Set[Cell] = {start}
        stack = [start]
        while stack:
            cx, cy = stack.pop()
            for dx, dy in _NEIGHBORS8:
                n = (cx + dx, cy + dy)
                if n in unseen and n not in comp:
                    comp.add(n)
                    stack.append(n)
        unseen -= comp
        comps.append(comp)
    comps.sort(key=lambda c: (-len(c), min(c)))
    return comps


def _nearest_candidate(cell: Cell, candidates: Set[Cell]) -> Optional[Cell]:
    """Nearest candidate by integer squared cell distance; None when empty.

    Geometric proximity in cell-index space, NOT path distance: the nearest
    candidate can sit on the far side of a wall. Ties are broken by (x, y) so
    the answer never depends on set iteration order — without that, this is
    just next(iter(set)) wearing a disguise.
    """
    if not candidates:
        return None
    cx, cy = int(cell[0]), int(cell[1])
    return min(
        candidates,
        key=lambda c: ((c[0] - cx) ** 2 + (c[1] - cy) ** 2, c[0], c[1]),
    )


def _largest_component_anchor(candidates: Set[Cell]) -> Optional[Cell]:
    """Smallest cell of the largest connected component — the no-evidence anchor.

    Used only when there is no positional evidence at all. Deliberately NOT
    min(candidates): that is a pure lexicographic extreme with zero
    connectivity content, and a map-edge orphan pocket (small x, small y) would
    win it every time — deterministically condemning the whole map.
    """
    comps = _connected_components(candidates)
    return min(comps[0]) if comps else None


def _flood(candidates: Set[Cell], seeds: Iterable[Cell]) -> Set[Cell]:
    """8-neighbour BFS over `candidates` starting from `seeds`.

    Non-candidate seeds are filtered AT ENQUEUE. The previous version enqueued
    them and dropped them at pop — which is precisely how an entire map could
    collapse to UNREACHABLE without leaving a single trace in the output.
    """
    reachable: Set[Cell] = set()
    q: deque = deque()
    for s in seeds:
        if s in candidates and s not in reachable:
            reachable.add(s)
            q.append(s)
    while q:
        cx, cy = q.popleft()
        for dx, dy in _NEIGHBORS8:
            n = (cx + dx, cy + dy)
            if n in candidates and n not in reachable:
                reachable.add(n)
                q.append(n)
    return reachable


def _resolve_seeds(
    candidates: Set[Cell],
    seed_cells: Optional[Set[Cell]],
    seed_xy: Optional[Tuple[float, float]],
    cell_size: float,
) -> Tuple[List[Cell], Dict[str, Any]]:
    """Turn whatever evidence the caller has into candidate seeds, plus an audit.

    Priority chain — seed_cells, then seed_xy, then a geometric anchor. NOT a
    union: `eligible` must be a function of the database and the map, never of
    where the robot happens to be parked, or the coverage denominator (and the
    gate built on it) would move with the parking spot.

    Neutrality rule: as soon as ANY requested seed is a candidate, the result is
    exactly the old expression [c for c in requested if c in candidates].
    Snapping is a repair for the no-usable-seed case ONLY — snapping
    unconditionally would widen `eligible` into regions nothing actually
    reaches, weakening the irreversible `unreachable` lock.
    """
    audit: Dict[str, Any] = {
        'seed_source': 'none',
        'seeds': [],
        'requested': [],
        'dropped_non_candidate': [],
        'snapped_from': [],
        'notes': [],
        'seed_unresolved': False,
    }
    if not candidates:
        audit['seed_unresolved'] = True
        audit['notes'].append('no_candidates')
        return [], audit

    requested: List[Cell] = []
    source = ''
    if seed_cells:
        requested = sorted(set(seed_cells))
        source = 'seed_cells'
    elif seed_xy is not None:
        sx, sy = float(seed_xy[0]), float(seed_xy[1])
        requested = [(int(math.floor(sx / cell_size)), int(math.floor(sy / cell_size)))]
        source = 'seed_xy'
    audit['requested'] = [[int(x), int(y)] for x, y in requested]

    if not requested:
        anchor = _largest_component_anchor(candidates)
        seeds = [anchor] if anchor is not None else []
        audit.update(
            seed_source='anchor',
            seeds=[[int(x), int(y)] for x, y in seeds],
            notes=['no_seed_evidence'],
        )
        return seeds, audit

    valid = sorted(c for c in requested if c in candidates)
    if valid:
        # Production branch: identical to the pre-fix seed set, by construction.
        # (Keep `c` whole here: unpacking `for x, y in requested` would rebind
        # `x` to the coordinate and the membership test would silently pass.)
        audit.update(
            seed_source=source,
            seeds=[[int(c[0]), int(c[1])] for c in valid],
            dropped_non_candidate=[
                [int(c[0]), int(c[1])] for c in requested if c not in candidates
            ],
        )
        return valid, audit

    # No requested seed is drivable. Snap each onto the map instead of dropping
    # it — the old code silently produced an empty flood here, and the entire
    # candidate set came back as UNREACHABLE (irreversible: exclude_cells).
    snapped: List[Cell] = []
    for c in requested:
        t = _nearest_candidate(c, candidates)
        if t is None:
            continue
        if t not in snapped:
            snapped.append(t)
        if t != c:
            audit['snapped_from'].append(
                {'from': [int(c[0]), int(c[1])], 'to': [int(t[0]), int(t[1])]}
            )
    audit.update(
        seed_source=f'{source}_snapped',
        seeds=[[int(x), int(y)] for x, y in snapped],
        notes=['seed_map_contradiction'],
    )
    return snapped, audit


def compute_eligible_area(
    map_yaml: Path,
    cfg: Dict[str, Any],
    *,
    seed_xy: Optional[Tuple[float, float]] = None,
    seed_cells: Optional[Set[Cell]] = None,
) -> EligibleArea:
    """Compute eligible 1 m visual cells from the 2D occupancy map."""
    cov = dict(cfg.get('coverage') or {})
    patrol = dict(cfg.get('patrol') or {})
    bc = _completion_cfg(cfg)
    cell_size = float(cov.get('cell_size_m', 1.0))
    clearance = float(patrol.get('goal_clearance_m', 0.45))
    kernel = int(patrol.get('free_kernel_cells', 2))
    structure_m = float(bc.get('structure_proximity_m', 3.0))
    use_structure = bool(bc.get('require_near_structure', True))

    free, info = load_free_mask(map_yaml)
    res = float(info['resolution'])
    k_clear = max(kernel, int(math.ceil(clearance / max(res, 1e-3))))
    free_safe = erode_free(free, k_clear)

    meta = yaml.safe_load(map_yaml.read_text(encoding='utf-8')) or {}
    img = map_yaml.parent / meta['image']
    pgm = cv2.imread(str(img), cv2.IMREAD_UNCHANGED)
    if pgm is None:
        raise FileNotFoundError(img)
    if pgm.ndim == 3:
        pgm = cv2.cvtColor(pgm, cv2.COLOR_BGR2GRAY)
    img_u8 = np.flipud(np.asarray(pgm, dtype=np.uint8))
    negate = int(meta.get('negate', 0))
    occ_t = float(meta.get('occupied_thresh', 0.65))
    free_t = float(meta.get('free_thresh', 0.25))
    pix = img_u8.astype(np.float64)
    occ = (pix / 255.0) if negate else ((255.0 - pix) / 255.0)
    free_raw = occ < free_t
    occupied = occ > occ_t
    unknown = ~(free_raw | occupied)
    dist_to_occ = cv2.distanceTransform((~occupied).astype(np.uint8), cv2.DIST_L2, 5) * res

    h, w = free.shape
    stride = max(1, int(cell_size / max(res, 1e-3)))

    free_cells: Set[Cell] = set()
    clear_cells: Set[Cell] = set()
    clearance_fail: Set[Cell] = set()
    unknown_cells: Set[Cell] = set()
    structure_ok: Set[Cell] = set()

    # Dense pass: mark free / unknown cells from any pixel
    for iy in range(0, h, stride):
        for ix in range(0, w, stride):
            x, y = map_to_world(ix, iy, info)
            cx, cy = int(math.floor(x / cell_size)), int(math.floor(y / cell_size))
            ccx, ccy = _cell_center(cx, cy, cell_size)
            mix, miy = world_to_map(ccx, ccy, info)
            if mix < 0 or miy < 0 or mix >= w or miy >= h:
                continue
            if unknown[miy, mix] and not free_raw[miy, mix] and not occupied[miy, mix]:
                unknown_cells.add((cx, cy))
                continue
            if free_raw[miy, mix]:
                free_cells.add((cx, cy))
            if _is_clear_goal(ccx, ccy, free_safe, info):
                clear_cells.add((cx, cy))
                if (not use_structure) or float(dist_to_occ[miy, mix]) <= structure_m:
                    structure_ok.add((cx, cy))
            elif free_raw[miy, mix]:
                clearance_fail.add((cx, cy))

    clearance_fail -= clear_cells
    structure_excluded = clear_cells - structure_ok
    candidates = set(structure_ok)

    # Connectivity: 8-neighbour flood from resolved seeds. A seed must never be
    # dropped silently — an empty flood condemns every candidate to
    # UNREACHABLE, which the orchestrator then passes as exclude_cells forever.
    seeds, seed_audit = _resolve_seeds(candidates, seed_cells, seed_xy, cell_size)
    reachable = _flood(candidates, seeds)
    flood_unknown = False

    if candidates and not reachable:
        # _resolve_seeds guarantees a candidate seed whenever candidates is
        # non-empty, so this is unreachable by construction. If it ever fires,
        # the resolver or the flood is broken — and condemning every cell is
        # irreversible. Keep the session alive, but make the fault LOUD rather
        # than plausible.
        LOGGER.error(
            'seed resolution produced an empty flood over %d candidates '
            '(seed_source=%s); falling back to the largest component',
            len(candidates), seed_audit.get('seed_source'),
        )
        seed_audit['notes'].append('INTERNAL_seed_resolution_failed')
        seed_audit['internal_error'] = True
        anchor = _largest_component_anchor(candidates)
        seeds = [anchor] if anchor is not None else []
        seed_audit['seed_source'] = 'anchor_recovery'
        seed_audit['seeds'] = [[int(x), int(y)] for x, y in seeds]
        reachable = _flood(candidates, seeds)

        if not reachable:
            # The recovery flood failed too, so reachability is UNKNOWN, not
            # empty. `unreachable` is an irreversible exclude-lock, and the safe
            # answer to "I cannot tell" is to condemn NOTHING: the evidence-based
            # ledger closes bad cells on three nav failures, whereas geometry
            # guessing here would silently kill the whole map forever.
            LOGGER.error(
                'recovery flood over %d candidates is still empty; treating '
                'reachability as UNKNOWN and excluding nothing', len(candidates),
            )
            seed_audit['notes'].append('INTERNAL_flood_unavailable')
            seed_audit['connectivity_unknown'] = True
            flood_unknown = True
            reachable = set(candidates)

    unreachable = candidates - reachable
    eligible = reachable

    # Audit. The component pass is only paid for when something came back
    # unreachable; otherwise "the flood covered every candidate" already
    # implies a single component.
    connectivity: Dict[str, Any]
    if unreachable or flood_unknown:
        comps = _connected_components(candidates)
        connectivity = {
            'component_count': len(comps),
            'component_sizes': [len(c) for c in comps[:10]],
            'largest_component_size': len(comps[0]) if comps else 0,
            'unreachable_component_sizes': [
                len(c) for c in comps if not (c & reachable)
            ],
        }
        if flood_unknown:
            # Real components, but no claim about which ones are reachable.
            connectivity['reachability_unknown'] = True
    else:
        connectivity = {
            'component_count': 1 if candidates else 0,
            'component_sizes': [len(candidates)] if candidates else [],
            'largest_component_size': len(candidates),
            'unreachable_component_sizes': [],
        }

    return EligibleArea(
        cell_size_m=cell_size,
        total_free_cells=len(free_cells),
        eligible_visual_cells=len(eligible),
        excluded_unknown=len(unknown_cells - free_cells),
        excluded_clearance=len(clearance_fail),
        excluded_unreachable=len(unreachable),
        excluded_structure=len(structure_excluded),
        eligible=eligible,
        free_cells=free_cells,
        clearance_fail=clearance_fail,
        unreachable=unreachable,
        unknown_cells=unknown_cells,
        structure_excluded=structure_excluded,
        seed_resolution=seed_audit,
        connectivity=connectivity,
    )


def _active_covered_and_yaw(
    model: CoverageModel,
) -> Tuple[Set[Cell], Dict[Cell, Set[int]]]:
    covered: Set[Cell] = set()
    yaw: Dict[Cell, Set[int]] = {}
    for s in model.cell_summaries():
        if s.active_count <= 0 and s.candidate_count <= 0:
            continue
        # Spatial coverage counts Active OR Candidate (resume must see pending)
        cell = (s.cell_x, s.cell_y)
        if s.active_count > 0 or s.candidate_count > 0:
            covered.add(cell)
            bins = set(s.covered_yaw_bins)
            yaw[cell] = bins
    return covered, yaw


def load_nav_fail_history(visual_root: Path) -> Dict[str, int]:
    """Persist counts of independent build sessions that nav-failed a cell."""
    path = visual_root / 'state' / 'nav_fail_history.json'
    if not path.is_file():
        return {}
    try:
        data = json.loads(path.read_text(encoding='utf-8')) or {}
        return {str(k): int(v) for k, v in (data.get('cells') or {}).items()}
    except Exception:  # noqa: BLE001
        return {}


# Failures owned by our own safety interlock / operator / the Nav2 server's
# absence. They say nothing about the cell. Never accumulate toward UNREACHABLE.
NON_ATTRIBUTABLE_FAILURE_CODES = frozenset({
    'CANCELLED',
    'ABORTED',
    'INTERLOCK_PAUSED',
    'NAV2_UNAVAILABLE',
})
# Decided by the goal itself before any motion; meaningful even when instant.
PREFLIGHT_FAILURE_CODES = frozenset({'GOAL_REJECTED'})
# Codes that only blame the cell once navigation actually ran for a while.
CELL_ATTRIBUTABLE_FAILURE_CODES = frozenset({'TIMEOUT', 'SEND_TIMEOUT'})

# Measured on the 2026-09-10 session: every bogus abort (NAV2_STATUS:6 caused by
# a goal fired into a momentary localization "clear" window) finished in
# 0.16-0.95 s, while every real attempt — 34 succeeded, 20 interlock-cancelled —
# ran >= 5 s. Nav2 cannot plan, drive and give up in under a second.
MIN_NAV_ATTEMPT_SEC = 2.0


def is_cell_attributable_failure(code: Any, elapsed_sec: Any = None) -> bool:
    """True when a nav_result_code blames the cell rather than the moment.

    Unknown / missing codes return False on purpose: the UNREACHABLE tally is
    an irreversible lock (excluded from gaps forever), so an ambiguous record
    must leave the cell retryable instead of condemning it.

    elapsed_sec: how long the navigation attempt actually ran. A NAV2_STATUS:*
    abort below MIN_NAV_ATTEMPT_SEC is a preamble failure (bad start pose),
    not evidence that the cell cannot be driven to. When elapsed_sec is None
    the duration is unknown, and the cell is left retryable.
    """
    c = str(code or '')
    if not c or c in NON_ATTRIBUTABLE_FAILURE_CODES:
        return False
    if c in PREFLIGHT_FAILURE_CODES:
        return True
    if not (c in CELL_ATTRIBUTABLE_FAILURE_CODES or c.startswith('NAV2_STATUS:')
            or c.startswith('ERROR:')):
        return False
    if elapsed_sec is None:
        return False
    try:
        return float(elapsed_sec) >= MIN_NAV_ATTEMPT_SEC
    except (TypeError, ValueError):
        return False


def update_nav_fail_history(
    visual_root: Path,
    session_goals: List[Dict[str, Any]],
) -> Dict[str, int]:
    """Increment per-cell fail count when a cell had NAV_FAILED and never REACHED in session.

    A NAV_FAILED row only counts when its nav_result_code is attributable to the
    cell (see is_cell_attributable_failure). Interlock cancellations are retried
    in-session and must not ratchet a reachable cell toward permanent UNREACHABLE.
    """
    hist = load_nav_fail_history(visual_root)
    failed: Set[str] = set()
    reached: Set[str] = set()
    for g in session_goals or []:
        cell = str(g.get('spatial_cell') or '')
        if not cell:
            continue
        nav = str(g.get('nav') or '')
        if nav == 'NAV_FAILED':
            if is_cell_attributable_failure(
                g.get('nav_result_code'), g.get('nav_elapsed_sec')
            ):
                failed.add(cell)
        elif nav == 'REACHED':
            reached.add(cell)
    only_fail = failed - reached
    for cell in only_fail:
        hist[cell] = int(hist.get(cell, 0)) + 1
    # Clear history when reached successfully
    for cell in reached:
        if cell in hist:
            del hist[cell]
    path = visual_root / 'state' / 'nav_fail_history.json'
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        json.dumps({'cells': hist}, indent=2, sort_keys=True) + '\n',
        encoding='utf-8',
    )
    return hist


def evaluate_coverage_completion(
    model: CoverageModel,
    map_yaml: Path,
    cfg: Dict[str, Any],
    *,
    seed_xy: Optional[Tuple[float, float]] = None,
    nav_fail_history: Optional[Dict[str, int]] = None,
    session_nav_failed_cells: Optional[Set[str]] = None,
    patrol_mode: str = 'full',
    build_kind: str = 'AUTO_BUILD',
) -> CoverageCompletion:
    """Evaluate spatial/yaw coverage against configurable completion gate."""
    bc = _completion_cfg(cfg)
    target_spatial = float(bc.get('target_spatial_coverage_ratio', 0.80))
    min_yaw = int(bc.get('min_useful_yaw_bins_per_cell', 2))
    max_nav_fail = float(bc.get('max_unresolved_nav_fail_ratio', 0.10))
    fail_to_unreachable = int(bc.get('nav_fail_sessions_to_unreachable', 3))
    micro_can_claim = bool(bc.get('micro_may_claim_full_complete', False))

    covered, yaw_map = _active_covered_and_yaw(model)
    eligible = compute_eligible_area(
        map_yaml,
        cfg,
        seed_xy=seed_xy,
        seed_cells=covered or None,
    )

    hist = nav_fail_history or {}
    session_fails = session_nav_failed_cells or set()

    covered_eligible = covered & eligible.eligible
    unvisited = eligible.eligible - covered
    under_covered: Set[Cell] = set()
    yaw_ok: Set[Cell] = set()
    yaw_bad: Set[Cell] = set()

    for cell in covered_eligible:
        bins = yaw_map.get(cell) or set()
        if len(bins) >= min_yaw:
            yaw_ok.add(cell)
        else:
            yaw_bad.add(cell)
            under_covered.add(cell)

    nav_retry: Set[Cell] = set()
    unreachable: Set[Cell] = set(eligible.unreachable)
    for cell_id, n in hist.items():
        try:
            cx, cy = parse_spatial_cell(cell_id)
        except ValueError:
            continue
        c = (cx, cy)
        if c not in eligible.eligible and c not in eligible.unreachable:
            continue
        if int(n) >= fail_to_unreachable:
            unreachable.add(c)
            unvisited.discard(c)
        elif cell_id in session_fails or int(n) > 0:
            if c in unvisited or c in covered_eligible:
                nav_retry.add(c)

    # Session fails that never reached → retryable (not permanent)
    for cell_id in session_fails:
        try:
            cx, cy = parse_spatial_cell(cell_id)
        except ValueError:
            continue
        c = (cx, cy)
        if c in unreachable:
            continue
        if c not in eligible.eligible:
            continue
        # Spatially covered cells keep COVERED / YAW_INSUFFICIENT — do not
        # re-label the whole cell NAV_FAILED just because one yaw goal failed.
        if c in covered_eligible:
            continue
        nav_retry.add(c)

    n_elig = max(len(eligible.eligible), 1)
    spatial_ratio = float(len(covered_eligible)) / float(n_elig)
    yaw_ratio = float(len(yaw_ok)) / float(max(len(covered_eligible), 1)) if covered_eligible else 0.0
    unresolved = len(nav_retry)
    unresolved_ratio = float(unresolved) / float(n_elig)

    remaining_actionable = unvisited | under_covered | nav_retry
    # Exhaustion: no actionable gaps left (all remaining are unreachable/unsafe/unknown)
    exhausted = len(remaining_actionable - unreachable) == 0 and len(eligible.eligible) > 0

    gate_reasons: List[str] = []
    spatial_ok = spatial_ratio >= target_spatial
    yaw_ok_gate = True
    if covered_eligible:
        # Among covered cells, enough share have useful yaw
        yaw_ok_gate = yaw_ratio >= float(bc.get('min_yaw_completeness_ratio', 0.70))
    nav_ok = unresolved_ratio <= max_nav_fail

    if not spatial_ok:
        gate_reasons.append(
            f'spatial={spatial_ratio:.3f}<target={target_spatial:.3f}'
        )
    if not yaw_ok_gate:
        gate_reasons.append(f'yaw_completeness={yaw_ratio:.3f}')
    if not nav_ok:
        gate_reasons.append(
            f'nav_fail_ratio={unresolved_ratio:.3f}>max={max_nav_fail:.3f}'
        )
    if not eligible.eligible:
        # Without this the report reads "you covered 0 of a real target" when
        # the truth is "there is no target at all".
        gate_reasons.append(
            'eligible_cells=0 (no candidates: check map/structure config)'
        )

    coverage_gate = (spatial_ok and yaw_ok_gate and nav_ok) or exhausted
    if exhausted and not (spatial_ok and yaw_ok_gate and nav_ok):
        gate_reasons.append('exhausted_actionable_gaps')

    mode = str(patrol_mode or '').lower()
    claim_ok = bool(coverage_gate)
    if mode == 'micro' and not micro_can_claim:
        claim_ok = False
        gate_reasons.append('micro_mode_cannot_claim_full_map_complete')

    # Cell status map (eligible + known exclusions of interest)
    status: Dict[str, str] = {}
    for c in eligible.eligible:
        cid = f'cell_{c[0]}_{c[1]}'
        if c in unreachable:
            status[cid] = UNREACHABLE
        elif c in nav_retry and c not in covered_eligible:
            status[cid] = NAV_FAILED_RETRYABLE
        elif c in yaw_bad:
            status[cid] = YAW_INSUFFICIENT
        elif c in under_covered:
            status[cid] = UNDER_COVERED
        elif c in covered_eligible:
            status[cid] = COVERED
        else:
            status[cid] = UNVISITED
    for c in eligible.clearance_fail:
        status.setdefault(f'cell_{c[0]}_{c[1]}', UNSAFE_CLEARANCE)
    for c in eligible.unknown_cells:
        status.setdefault(f'cell_{c[0]}_{c[1]}', UNKNOWN_MAP)
    for c in unreachable:
        status[f'cell_{c[0]}_{c[1]}'] = UNREACHABLE

    # Covered cells that are neither eligible nor unreachable — i.e. the DB and
    # the map disagree (clearance / structure / unknown). `covered_eligible`
    # alone drops these from every count and from `status`, so a non-zero value
    # here is the only alarm that the two have drifted apart.
    covered_outside_eligible = covered - eligible.eligible - unreachable

    counts = {
        'eligible': len(eligible.eligible),
        'candidate_cells': len(eligible.eligible) + len(eligible.unreachable),
        'covered_outside_eligible': len(covered_outside_eligible),
        'covered': len(covered_eligible),
        'under_covered': len(under_covered),
        'unvisited': len(unvisited - nav_retry),
        'nav_failed_retryable': len(nav_retry),
        'unreachable': len(unreachable),
        'yaw_sufficient': len(yaw_ok),
        'yaw_insufficient': len(yaw_bad),
        'unsafe_clearance': len(eligible.clearance_fail),
        'unknown_map': len(eligible.unknown_cells - eligible.free_cells),
    }

    return CoverageCompletion(
        eligible=eligible,
        covered_cells=covered,
        covered_eligible=covered_eligible,
        under_covered=under_covered,
        unvisited=unvisited,
        yaw_sufficient=yaw_ok,
        yaw_insufficient=yaw_bad,
        nav_failed_retryable=nav_retry,
        unreachable=unreachable,
        unsafe_clearance=eligible.clearance_fail,
        unknown_map=eligible.unknown_cells,
        spatial_coverage_ratio=spatial_ratio,
        yaw_completeness_ratio=yaw_ratio,
        unresolved_nav_fail_ratio=unresolved_ratio,
        gate_pass=bool(coverage_gate),
        gate_reasons=gate_reasons,
        map_complete_claim_allowed=claim_ok,
        cell_status=status,
        counts=counts,
    )


def remaining_gap_cells(completion: CoverageCompletion) -> Set[Cell]:
    """Cells resume/full build should still plan for."""
    out: Set[Cell] = set()
    out |= completion.unvisited
    out |= completion.under_covered
    out |= completion.yaw_insufficient
    out |= completion.nav_failed_retryable
    out -= completion.unreachable
    return out


def format_completion_status_text(completion: CoverageCompletion, *, mode: str = '') -> str:
    c = completion.counts
    lines = [
        '视觉地图建库',
        f"Eligible区域：{c.get('eligible', 0)}",
        f"已覆盖：{c.get('covered', 0)}",
        f"空间覆盖率：{completion.spatial_coverage_ratio * 100:.1f}%",
        '',
        f"朝向充分：{c.get('yaw_sufficient', 0)}",
        f"待补朝向：{c.get('yaw_insufficient', 0)}",
        '',
        f"导航失败：{c.get('nav_failed_retryable', 0)}",
        f"不可达：{c.get('unreachable', 0)}",
        '',
    ]
    if completion.map_complete_claim_allowed:
        lines.append('状态：【视觉定位库建库完成】')
    else:
        reason = '；'.join(completion.gate_reasons[:3]) if completion.gate_reasons else '未达Coverage Gate'
        if str(mode).lower() == 'micro':
            lines.append('状态：【尚未完成】（micro测试模式，不可宣称全图建完）')
        else:
            lines.append(f'状态：【尚未完成】{reason}')
    return '\n'.join(lines)
