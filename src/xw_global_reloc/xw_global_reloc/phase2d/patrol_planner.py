"""Coverage-driven patrol goal planner (known free space only; no frontier)."""

from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Callable, Dict, List, Optional, Sequence, Set, Tuple

import cv2
import numpy as np
import yaml

from xw_global_reloc.phase2d.coverage_model import CoverageModel
from xw_global_reloc.phase2d.spatial import parse_spatial_cell, spatial_cell_id, yaw_bin_index


@dataclass
class PatrolGoal:
    x: float
    y: float
    yaw: float
    spatial_cell: str
    yaw_bin: int
    reason: str  # missing_cell | missing_yaw | sparse

    def as_dict(self) -> Dict[str, Any]:
        return {
            'x': self.x,
            'y': self.y,
            'yaw': self.yaw,
            'spatial_cell': self.spatial_cell,
            'yaw_bin': self.yaw_bin,
            'reason': self.reason,
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
) -> List[PatrolGoal]:
    """Generate NavigateToPose goals in known free space from coverage gaps.

    should_stop: optional zero-arg callable; when truthy, abort planning early
    so user stop can interrupt long dense-coverage loops.
    only_cells: if set, only plan these spatial cells (resume / gap fill).
    exclude_cells: permanently skip (e.g. UNREACHABLE).
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

    if mode == 'micro':
        max_total = min(max_total, micro_max)
    elif mode == 'partial':
        max_total = min(max_total, max(12, micro_max * 3))
    # full: keep max_total_goals as session/round budget (orchestrator loops)

    if _stopped():
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

    # Enumerate free coverage-cells
    candidates: List[Tuple[float, int, int, float, float]] = []
    # score, cx, cy, x, y
    stride = max(1, int(cell_size / res))
    for iy in range(0, h, stride):
        if _stopped():
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
            # Prefer missing cells near existing coverage (expand frontier of known DB)
            if missing:
                if dist_cov <= near_m or not covered:
                    score = 1000.0 - dist_cov
                else:
                    score = 100.0 - min(dist_cov, 50.0)
            else:
                # yaw-deficient cells
                cell = spatial_cell_id(x, y, cell_size)
                have = _yaw_bins_for_cell(model, cell)
                miss_n = yaw_bins - len(have)
                if miss_n <= 0 or len(have) >= min_yaw_useful:
                    # already has enough useful yaw unless only_cells forces revisit
                    if only is None:
                        continue
                    if len(have) >= min_yaw_useful:
                        continue
                score = 50.0 + miss_n
            if seed_xy is not None:
                score -= 0.05 * math.hypot(x - seed_xy[0], y - seed_xy[1])
            candidates.append((score, cx, cy, x, y))

    if _stopped():
        return []

    candidates.sort(key=lambda t: -t[0])
    goals: List[PatrolGoal] = []
    used_cells: Set[Tuple[int, int]] = set()

    for score, cx, cy, x, y in candidates:
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
            if not want_bins:
                continue

        picked = 0
        for b in want_bins:
            if picked >= max_yaw_per or len(goals) >= max_total:
                break
            if b in have and reason == 'missing_yaw':
                continue
            yaw = (b + 0.5) * (2.0 * math.pi / yaw_bins)
            goals.append(
                PatrolGoal(x=x, y=y, yaw=float(yaw), spatial_cell=cell, yaw_bin=int(b), reason=reason)
            )
            picked += 1
        if picked:
            used_cells.add((cx, cy))

    return goals
