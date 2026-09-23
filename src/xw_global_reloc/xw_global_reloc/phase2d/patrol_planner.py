"""Coverage-driven patrol goal planner (known free space only; no frontier)."""

from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Callable, Dict, List, Optional, Sequence, Set, Tuple

import cv2
import numpy as np
import yaml

from xw_global_reloc.phase2d.coverage_model import CoverageModel, appearance_id_for_slot
from xw_global_reloc.phase2d.spatial import parse_spatial_cell, spatial_cell_id, yaw_bin_index


@dataclass
class PatrolGoal:
    x: float
    y: float
    yaw: float
    spatial_cell: str
    yaw_bin: int
    reason: str  # missing_cell | missing_yaw | sparse
    # ---- A2 diagnostics (2026-09-22). Purely additive: every field has a
    # default, so existing construction sites and consumers are unaffected.
    # These exist because a goal used to be logged as `reason='missing_cell'`
    # and nothing else -- you could not tell a 0.1 m hop from a 40 m trek, nor
    # whether the per-round budget was the binding constraint. A2's hard `cap`
    # was deliberately NOT implemented (it needs a NEW threshold); this is the
    # data that has to come first.
    score: float = 0.0
    dist_cov_m: float = float('inf')
    miss_n: int = 0
    n_candidates: int = 0
    # ---- D1 Multi-Appearance (2026-09-23). Purely additive (defaults to None).
    # For a `reason='forced_appearance'` goal it carries the id a NEW distinct
    # appearance captured at this slot would get -- the SAME derivation the
    # capture writer uses (appearance_id_for_slot), so the two agree by
    # construction and a DB replay reproduces identical ids.
    appearance_id: Optional[str] = None

    def as_dict(self) -> Dict[str, Any]:
        return {
            'x': self.x,
            'y': self.y,
            'yaw': self.yaw,
            'spatial_cell': self.spatial_cell,
            'yaw_bin': self.yaw_bin,
            'reason': self.reason,
            'score': self.score,
            'dist_cov_m': self.dist_cov_m,
            'miss_n': self.miss_n,
            'n_candidates': self.n_candidates,
            'appearance_id': self.appearance_id,
        }


def load_free_mask(map_yaml: Path) -> Tuple[np.ndarray, Dict[str, float]]:
    """Return free boolean mask (H,W) in OccupancyGrid orientation + meta."""
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
    free = occ < free_t
    occupied = occ > occ_t
    unknown = ~(free | occupied)
    info = {
        'resolution': float(meta['resolution']),
        'origin_x': float(meta['origin'][0]),
        'origin_y': float(meta['origin'][1]),
        'width': float(img_u8.shape[1]),
        'height': float(img_u8.shape[0]),
    }
    return free & (~unknown), info


def world_to_map(x: float, y: float, info: Dict[str, float]) -> Tuple[int, int]:
    res = float(info['resolution'])
    ix = int(math.floor((x - float(info['origin_x'])) / res))
    iy = int(math.floor((y - float(info['origin_y'])) / res))
    return ix, iy


def map_to_world(ix: int, iy: int, info: Dict[str, float]) -> Tuple[float, float]:
    res = float(info['resolution'])
    x = float(info['origin_x']) + (ix + 0.5) * res
    y = float(info['origin_y']) + (iy + 0.5) * res
    return x, y


def erode_free(free: np.ndarray, kernel_cells: int) -> np.ndarray:
    k = max(0, int(kernel_cells))
    if k <= 0:
        return free
    kernel = np.ones((2 * k + 1, 2 * k + 1), dtype=np.uint8)
    return cv2.erode(free.astype(np.uint8), kernel, iterations=1).astype(bool)


def _cell_center(cx: int, cy: int, cell_size: float) -> Tuple[float, float]:
    return (cx + 0.5) * cell_size, (cy + 0.5) * cell_size


def _is_clear_goal(
    x: float,
    y: float,
    free_safe: np.ndarray,
    info: Dict[str, float],
) -> bool:
    ix, iy = world_to_map(x, y, info)
    h, w = free_safe.shape
    if ix < 0 or iy < 0 or ix >= w or iy >= h:
        return False
    return bool(free_safe[iy, ix])


def _covered_cells(model: CoverageModel) -> Set[Tuple[int, int]]:
    out: Set[Tuple[int, int]] = set()
    for s in model.cell_summaries():
        if s.active_count > 0 or s.candidate_count > 0:
            out.add((s.cell_x, s.cell_y))
    return out


def _yaw_bins_for_cell(model: CoverageModel, cell: str) -> Set[int]:
    bins: Set[int] = set()
    for (c, yb), slot in model.slots.items():
        if c == cell and (slot.active_ids or slot.candidate_ids):
            bins.add(int(yb))
    return bins


def revisit_targets(
    model: CoverageModel,
    *,
    attempted_pairs: Optional[Set[Tuple[str, int]]] = None,
    exclude_cells: Optional[Set[Tuple[int, int]]] = None,
    max_per_slot: int = 5,
) -> Tuple[Set[Tuple[int, int]], Set[Tuple[str, int]]]:
    """D1 revisit round: covered slots that can still take another appearance.

    Returns ``(cells, pairs)``. ``cells`` (parsed, the same form `only_cells`
    takes) narrows the round's search domain; ``pairs`` (string cell id, the
    same form `force_pairs` takes) is what actually gets re-visited. Pure
    function, no rclpy, no new thresholds -- the caller owns the round budget.

    "Not attempted this session" is a hard requirement, not tidiness: TWO
    independent gates silently drop a re-visited pair -- the planner's own
    `exclude_pairs` check below, and the caller's freshness filter on
    `self._session.goals`. Handing an attempted pair to this function would
    produce a batch that plans and then vanishes.

    Guards, in order:
      * the slot must already hold a frame (`active_ids or candidate_ids` --
        the same predicate `_yaw_bins_for_cell` uses); nothing to re-visit
        otherwise, and a frame-less slot would not be in `covered` either;
      * the pair must not be in `attempted_pairs`;
      * the slot must still be under `max_per_slot` (pass the caller's
        `candidate_limits.max_per_cell_yaw`, i.e. the same key D2-b's capture
        gate uses) -- this can never plan a goal the capture policy would then
        refuse to fill;
      * the cell must not be in `exclude_cells` (UNREACHABLE and friends).
    """
    attempted = attempted_pairs or set()
    excl = exclude_cells or set()
    cells: Set[Tuple[int, int]] = set()
    pairs: Set[Tuple[str, int]] = set()
    for (cell, yb), slot in model.slots.items():
        if not (slot.active_ids or slot.candidate_ids):
            continue
        pair = (str(cell), int(yb))
        if pair in attempted:
            continue
        if model.appearance_count(pair[0], pair[1]) >= int(max_per_slot):
            continue
        try:
            key = parse_spatial_cell(pair[0])
        except ValueError:
            continue
        if key in excl:
            continue
        pairs.add(pair)
        cells.add(key)
    return cells, pairs


def _nearest_covered_dist(cx: int, cy: int, covered: Set[Tuple[int, int]], cell_size: float) -> float:
    if not covered:
        return 0.0
    best = float('inf')
    for ax, ay in covered:
        d = math.hypot((cx - ax) * cell_size, (cy - ay) * cell_size)
        if d < best:
            best = d
    return best


def plan_patrol_goals(
    model: CoverageModel,
    *,
    map_yaml: Path,
    cfg: Dict[str, Any],
    mode: str = 'micro',
    seed_xy: Optional[Tuple[float, float]] = None,
    should_stop: Optional[Callable[[], bool]] = None,
    only_cells: Optional[Set[Tuple[int, int]]] = None,
    exclude_cells: Optional[Set[Tuple[int, int]]] = None,
    exclude_pairs: Optional[Set[Tuple[str, int]]] = None,
    force_pairs: Optional[Set[Tuple[str, int]]] = None,
    diag: Optional[Dict[str, Any]] = None,
) -> List[PatrolGoal]:
    """Generate NavigateToPose goals in known free space from coverage gaps.

    should_stop: optional zero-arg callable; when truthy, abort planning early
    so user stop can interrupt long dense-coverage loops.
    only_cells: if set, only plan these spatial cells (resume / gap fill).
    exclude_cells: permanently skip (e.g. UNREACHABLE).
    exclude_pairs: skip these (spatial_cell, yaw_bin) pairs. Session-scoped
        already-attempted targets; keeps the per-round goal budget from being
        consumed by cells this session already tried (which would otherwise
        crowd out untouched gaps and starve the session).
    force_pairs: D1 Multi-Appearance -- (spatial_cell, yaw_bin) slots to REVISIT
        even though they are already covered. Such a bin is by construction
        already in `have`, so the ordinary `want_bins` channel can never emit it;
        these are emitted separately with reason 'forced_appearance' (which the
        `b in have and reason == 'missing_yaw'` guard lets through untouched).
        They consume the per-round budget (`max_total_goals`) but deliberately
        NOT `max_yaw_targets_per_cell` (that would silently drop them). Unknown
        or unparsable cell ids are ignored. Empty/None => every path below is
        bit-for-bit the pre-D1 code.
    diag: optional dict, filled in-place with selection diagnostics (candidate
        counts, score/dist_cov percentiles, which constraint bound). Read-only:
        it never influences which goals are produced.
    """
    patrol = dict(cfg.get('patrol') or {})
    cov = dict(cfg.get('coverage') or {})
    cell_size = float(cov.get('cell_size_m', model.cell_size_m))
    yaw_bins = int(cov.get('yaw_bins', model.yaw_bins))
    max_yaw_per = int(patrol.get('max_yaw_targets_per_cell', 2))
    max_total = int(patrol.get('max_total_goals', 40))
    clearance = float(patrol.get('goal_clearance_m', 0.45))
    kernel = int(patrol.get('free_kernel_cells', 2))
    near_m = float(patrol.get('prefer_near_existing_m', 8.0))
    micro_max = int(patrol.get('micro_max_goals', 6))
    min_yaw_useful = int((cfg.get('build_completion') or {}).get('min_useful_yaw_bins_per_cell', 2))

    def _stopped() -> bool:
        return bool(should_stop()) if callable(should_stop) else False

    def _mark_stopped() -> None:
        if diag is not None:
            diag['stopped_early'] = True

    if mode == 'micro':
        max_total = min(max_total, micro_max)
    elif mode == 'partial':
        max_total = min(max_total, max(12, micro_max * 3))
    # full: keep max_total_goals as session/round budget (orchestrator loops)

    if _stopped():
        _mark_stopped()
        return []

    free, info = load_free_mask(map_yaml)
    # Extra clearance via erosion using clearance distance
    res = float(info['resolution'])
    k_clear = max(kernel, int(math.ceil(clearance / max(res, 1e-3))))
    free_safe = erode_free(free, k_clear)

    covered = _covered_cells(model)
    h, w = free_safe.shape
    only = only_cells
    excl = exclude_cells or set()
    excl_pairs = exclude_pairs or set()
    # ---- D1 Multi-Appearance (2026-09-23): forced revisits -------------------
    forced_by_cell: Dict[Tuple[int, int], List[int]] = {}
    for _cell_s, _bin in sorted(force_pairs or set()):
        try:
            _fc = parse_spatial_cell(str(_cell_s))
        except ValueError:
            continue
        if 0 <= int(_bin) < yaw_bins:
            forced_by_cell.setdefault(_fc, []).append(int(_bin))
    forced_cells = set(forced_by_cell)
    # A forced pair is ALWAYS an already-covered cell => never in the gap set =>
    # the `only` filter below would silently drop the whole batch. Union (over a
    # copy -- the caller's set is never mutated) and only when `only` is already
    # active, so the `only is None` branch stays exactly as it was.
    if only is not None and forced_cells:
        only = set(only) | forced_cells

    # Enumerate free coverage-cells
    candidates: List[Tuple[float, int, int, float, float, float, int]] = []
    # score, cx, cy, x, y, dist_cov, miss_n
    n_missing_cand = 0
    n_covered_cand = 0
    n_yaw_sufficient_skipped = 0
    stride = max(1, int(cell_size / res))
    for iy in range(0, h, stride):
        if _stopped():
            _mark_stopped()
            return []
        for ix in range(0, w, stride):
            if not free_safe[iy, ix]:
                continue
            x, y = map_to_world(ix, iy, info)
            if not _is_clear_goal(x, y, free_safe, info):
                continue
            cx, cy = int(math.floor(x / cell_size)), int(math.floor(y / cell_size))
            if (cx, cy) in excl:
                continue
            if only is not None and (cx, cy) not in only:
                continue
            # Prefer cell centers that are free
            ccx, ccy = _cell_center(cx, cy, cell_size)
            if _is_clear_goal(ccx, ccy, free_safe, info):
                x, y = ccx, ccy
            dist_cov = _nearest_covered_dist(cx, cy, covered, cell_size)
            missing = (cx, cy) not in covered
            miss_n = 0
            # Prefer missing cells near existing coverage (expand frontier of known DB)
            if missing:
                n_missing_cand += 1
                if dist_cov <= near_m or not covered:
                    score = 1000.0 - dist_cov
                else:
                    score = 100.0 - min(dist_cov, 50.0)
            else:
                # yaw-deficient cells
                cell = spatial_cell_id(x, y, cell_size)
                have = _yaw_bins_for_cell(model, cell)
                miss_n = yaw_bins - len(have)
                # D1: this gate is cell-level and does NOT look at `only`, so a
                # forced cell (yaw-sufficient by construction -- that is exactly
                # why it is being revisited) must be let through explicitly.
                if (miss_n <= 0 or len(have) >= min_yaw_useful) and (cx, cy) not in forced_cells:
                    # already has enough useful yaw unless only_cells forces revisit
                    if only is None:
                        n_yaw_sufficient_skipped += 1
                        continue
                    if len(have) >= min_yaw_useful:
                        n_yaw_sufficient_skipped += 1
                        continue
                score = 50.0 + miss_n
                n_covered_cand += 1
            if seed_xy is not None:
                score -= 0.05 * math.hypot(x - seed_xy[0], y - seed_xy[1])
            candidates.append((score, cx, cy, x, y, dist_cov, miss_n))

    if _stopped():
        _mark_stopped()
        return []

    candidates.sort(key=lambda t: -t[0])
    n_candidates = len(candidates)
    goals: List[PatrolGoal] = []
    used_cells: Set[Tuple[int, int]] = set()
    # D1: forced pairs already emitted. Needed because a yaw-saturated forced
    # cell never reaches the `if picked:` below, so without this every sampled
    # pixel of that cell would re-emit the same pair.
    emitted_forced: Set[Tuple[str, int]] = set()

    for score, cx, cy, x, y, dist_cov, miss_n in candidates:
        if _stopped():
            break
        if len(goals) >= max_total:
            break
        if (cx, cy) in used_cells:
            continue
        cell = f'cell_{cx}_{cy}'
        have = _yaw_bins_for_cell(model, cell)
        if (cx, cy) not in covered:
            reason = 'missing_cell'
            # Pick a few principal yaw bins (0 and 90/180 style)
            want_bins = [0, yaw_bins // 4, yaw_bins // 2, (3 * yaw_bins) // 4]
        else:
            reason = 'missing_yaw'
            want_bins = [b for b in range(yaw_bins) if b not in have]
            # ---- D1 Multi-Appearance: emit this cell's forced revisits --------
            # Placed BEFORE the `if not want_bins: continue` below (a yaw-
            # saturated cell has no want_bins at all) and BEFORE the
            # exclude_pairs stripping, which never sees these bins anyway.
            # Nothing here touches `used_cells`: that set exists to stop a cell
            # from consuming several candidate slots, not to gate revisits.
            for b in forced_by_cell.get((cx, cy), ()):
                if len(goals) >= max_total:
                    break
                if (cell, int(b)) in excl_pairs or (cell, int(b)) in emitted_forced:
                    continue
                emitted_forced.add((cell, int(b)))
                goals.append(
                    PatrolGoal(
                        x=x,
                        y=y,
                        yaw=float((b + 0.5) * (2.0 * math.pi / yaw_bins)),
                        spatial_cell=cell,
                        yaw_bin=int(b),
                        reason='forced_appearance',
                        appearance_id=appearance_id_for_slot(
                            cell, int(b), model.appearance_count(cell, int(b))
                        ),
                        score=float(score),
                        dist_cov_m=float(dist_cov),
                        miss_n=int(miss_n),
                        n_candidates=int(n_candidates),
                    )
                )
            if not want_bins:
                continue

        # Drop yaw bins this session already attempted for this cell; a cell
        # with nothing new must not consume a candidate slot (it would also be
        # re-visited from every other sampled pixel of the same cell).
        if excl_pairs:
            want_bins = [b for b in want_bins if (cell, int(b)) not in excl_pairs]
            if not want_bins:
                used_cells.add((cx, cy))
                continue

        picked = 0
        for b in want_bins:
            if picked >= max_yaw_per or len(goals) >= max_total:
                break
            if b in have and reason == 'missing_yaw':
                continue
            yaw = (b + 0.5) * (2.0 * math.pi / yaw_bins)
            goals.append(
                PatrolGoal(
                    x=x, y=y, yaw=float(yaw), spatial_cell=cell, yaw_bin=int(b), reason=reason,
                    score=float(score), dist_cov_m=float(dist_cov), miss_n=int(miss_n),
                    n_candidates=int(n_candidates),
                )
            )
            picked += 1
        if picked:
            used_cells.add((cx, cy))

    if diag is not None:
        dists = sorted(float(c[5]) for c in candidates)
        scores = sorted(float(c[0]) for c in candidates)

        def _pct(xs: List[float], q: float) -> Optional[float]:
            if not xs:
                return None
            i = min(len(xs) - 1, max(0, int(round(q * (len(xs) - 1)))))
            return float(xs[i])

        reason_counts: Dict[str, int] = {}
        for g in goals:
            reason_counts[g.reason] = reason_counts.get(g.reason, 0) + 1
        diag.update({
            'mode': mode,
            'n_candidates': n_candidates,
            'n_goals': len(goals),
            'max_total': int(max_total),
            'max_yaw_targets_per_cell': int(max_yaw_per),
            # `near_m` is the EXISTING prefer_near_existing_m, not a new threshold;
            # it is reported so the score split can be read without a lookup.
            'near_m': float(near_m),
            'only_cells_n': (len(only) if only is not None else None),
            'exclude_cells_n': len(excl),
            'exclude_pairs_n': len(excl_pairs),
            # D1: requested vs emitted, so a forced batch that dies on the way
            # (budget / excl_pairs / unparsable cell) cannot do so silently.
            'force_pairs_n': len(forced_cells),
            'n_forced_appearance_goals': int(sum(1 for g in goals if g.reason == 'forced_appearance')),
            'n_missing_cell_candidates': n_missing_cand,
            'n_missing_yaw_candidates': n_covered_cand,
            'n_yaw_sufficient_skipped': n_yaw_sufficient_skipped,
            'n_cells_used': len(used_cells),
            'reason_counts': reason_counts,
            'dist_cov_p50': _pct(dists, 0.50),
            'dist_cov_p90': _pct(dists, 0.90),
            'dist_cov_max': (dists[-1] if dists else None),
            'n_dist_cov_over_near_m': int(sum(1 for d in dists if d > near_m)),
            'score_p50': _pct(scores, 0.50),
            'score_p90': _pct(scores, 0.90),
            'hit_max_total': bool(len(goals) >= max_total),
            'stopped_early': bool(diag.get('stopped_early', False)),
        })

    return goals
