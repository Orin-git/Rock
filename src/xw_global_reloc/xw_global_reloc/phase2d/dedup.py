"""Candidate / Active dedup for Phase2D-A3 (local neighborhood only)."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional

import numpy as np

from xw_global_reloc.orb_utils import visual_difference
from xw_global_reloc.phase2d.coverage_model import CoverageModel, FrameRef


@dataclass
class DedupResult:
    is_duplicate: bool
    reason: str
    nearest_keyframe_id: Optional[str] = None
    nearest_distance_m: float = float('inf')
    nearest_yaw_delta_deg: float = float('inf')
    visual_difference: Optional[float] = None
    compared_ids: List[str] = field(default_factory=list)


def _load_desc(fr: FrameRef) -> Optional[np.ndarray]:
    if fr.descriptors is not None:
        return fr.descriptors
    if fr.descriptors_path is None:
        return None
    try:
        if fr.descriptors_path.is_file():
            fr.descriptors = np.load(str(fr.descriptors_path))
            return fr.descriptors
    except Exception:  # noqa: BLE001
        return None
    return None


def evaluate_dedup(
    model: CoverageModel,
    *,
    x: float,
    y: float,
    yaw: float,
    query_descriptors: Optional[np.ndarray],
    cfg: Dict[str, Any],
) -> DedupResult:
    """Near + similar yaw + low visual_difference → DUPLICATE.

    Near + similar yaw + high visual_difference → not duplicate (novelty path).
    """
    dd = dict(cfg.get('dedup') or {})
    max_xy = float(dd.get('max_xy_m', 0.35))
    max_yaw = float(dd.get('max_yaw_deg', 15.0))
    # visual_difference is higher when more different. Duplicate if below this.
    max_visual_diff_for_dup = float(dd.get('max_visual_diff_for_duplicate', 0.25))

    neighbors = model.frames_near(
        x,
        y,
        yaw,
        max_xy_m=max_xy,
        max_yaw_deg=max_yaw,
        include_active=True,
        include_candidate=True,
    )
    if not neighbors:
        return DedupResult(False, 'no_near_neighbors')

    best: Optional[FrameRef] = None
    best_vdiff = 1.0
    best_dist = float('inf')
    best_dyaw = float('inf')
    compared: List[str] = []

    for fr in neighbors:
        compared.append(fr.keyframe_id)
        dist = ((fr.x - x) ** 2 + (fr.y - y) ** 2) ** 0.5
        from xw_global_reloc.phase2d.spatial import yaw_delta_deg

        dyaw = yaw_delta_deg(fr.yaw, yaw)
        desc = _load_desc(fr)
        vdiff = visual_difference(query_descriptors, desc) if query_descriptors is not None else 1.0
        # Prefer closest similar frame
        if vdiff < best_vdiff or (
            abs(vdiff - best_vdiff) < 1e-9 and dist < best_dist
        ):
            best = fr
            best_vdiff = vdiff
            best_dist = dist
            best_dyaw = dyaw

    if best is None:
        return DedupResult(False, 'no_near_neighbors', compared_ids=compared)

    if best_vdiff <= max_visual_diff_for_dup:
        return DedupResult(
            True,
            'DUPLICATE',
            nearest_keyframe_id=best.keyframe_id,
            nearest_distance_m=float(best_dist),
            nearest_yaw_delta_deg=float(best_dyaw),
            visual_difference=float(best_vdiff),
            compared_ids=compared,
        )

    return DedupResult(
        False,
        'VISUAL_NOVELTY',
        nearest_keyframe_id=best.keyframe_id,
        nearest_distance_m=float(best_dist),
        nearest_yaw_delta_deg=float(best_dyaw),
        visual_difference=float(best_vdiff),
        compared_ids=compared,
    )
