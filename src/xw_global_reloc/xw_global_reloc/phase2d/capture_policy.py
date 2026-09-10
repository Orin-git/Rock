"""Adaptive capture policy after A2 quality gates."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional

import numpy as np

from xw_global_reloc.orb_utils import visual_difference
from xw_global_reloc.phase2d.coverage_model import CoverageModel, FrameRef
from xw_global_reloc.phase2d.dedup import DedupResult, evaluate_dedup
from xw_global_reloc.phase2d.spatial import yaw_delta_deg


# Decision reasons (explicit, not bool-only)
CAPTURE_NEW_CELL = 'CAPTURE_NEW_CELL'
CAPTURE_NEW_YAW = 'CAPTURE_NEW_YAW'
CAPTURE_TRANSLATION = 'CAPTURE_TRANSLATION'
CAPTURE_VISUAL_NOVELTY = 'CAPTURE_VISUAL_NOVELTY'
SKIP_DUPLICATE = 'SKIP_DUPLICATE'
SKIP_COVERED = 'SKIP_COVERED'
SKIP_CELL_YAW_QUOTA = 'SKIP_CELL_YAW_QUOTA'
SKIP_SESSION_QUOTA = 'SKIP_SESSION_QUOTA'


@dataclass
class CaptureDecision:
    should_capture: bool
    reason: str
    spatial_cell: str
    yaw_bin: int
    nearest_keyframe_id: Optional[str] = None
    nearest_distance_m: float = float('inf')
    nearest_yaw_delta_deg: float = float('inf')
    visual_difference: Optional[float] = None
    possible_new_appearance: bool = False
    notes: List[str] = field(default_factory=list)

    def to_meta(self) -> Dict[str, Any]:
        return {
            'reason': self.reason,
            'nearest_keyframe_id': self.nearest_keyframe_id,
            'nearest_distance_m': None
            if self.nearest_distance_m == float('inf')
            else float(self.nearest_distance_m),
            'nearest_yaw_delta_deg': None
            if self.nearest_yaw_delta_deg == float('inf')
            else float(self.nearest_yaw_delta_deg),
            'visual_difference': self.visual_difference,
            'possible_new_appearance': bool(self.possible_new_appearance),
        }


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


def _vdiff_vs_cell_yaw(
    model: CoverageModel,
    cell: str,
    yaw_bin: int,
    query_desc: Optional[np.ndarray],
) -> tuple[Optional[float], Optional[str]]:
    slot = model.slots.get((cell, int(yaw_bin)))
    if slot is None or query_desc is None:
        return None, None
    ids = list(slot.active_ids) + list(slot.candidate_ids)
    best = -1.0
    best_id = None
    for kid in ids:
        fr = model.frames.get(kid)
        if fr is None:
            continue
        desc = _load_desc(fr)
        vd = visual_difference(query_desc, desc)
        if vd > best:
            best = vd
            best_id = kid
    if best_id is None:
        return None, None
    return float(best), best_id


def evaluate_capture_policy(
    model: CoverageModel,
    *,
    x: float,
    y: float,
    yaw: float,
    query_descriptors: Optional[np.ndarray],
    cfg: Dict[str, Any],
) -> CaptureDecision:
    """Decide AFTER Pose/Laser/Image gates. Never time-based."""
    cell, yb = model.assign(x, y, yaw)
    cap = dict(cfg.get('capture') or {})
    lim = dict(cfg.get('candidate_limits') or {})
    min_trans = float(cap.get('min_translation_m', 0.75))
    min_yaw = float(cap.get('min_yaw_deg', 35.0))
    min_vdiff = float(cap.get('min_visual_diff', 0.30))
    max_per_slot = int(lim.get('max_per_cell_yaw', 5))
    max_session = int(lim.get('max_per_build_session', 500))

    nearest, nearest_d, nearest_dyaw = model.nearest_frame(x, y, yaw)
    nearest_id = nearest.keyframe_id if nearest else None

    # Session quota (candidates written this process)
    if int(model.session_writes) >= max_session:
        model.stats['quota_skips'] += 1
        return CaptureDecision(
            False,
            SKIP_SESSION_QUOTA,
            cell,
            yb,
            nearest_keyframe_id=nearest_id,
            nearest_distance_m=nearest_d,
            nearest_yaw_delta_deg=nearest_dyaw,
        )

    # Cell+yaw candidate/active quota
    if model.count_cell_yaw(cell, yb) >= max_per_slot:
        # Still allow novelty only if visual very different? Spec says quota skip.
        model.stats['quota_skips'] += 1
        return CaptureDecision(
            False,
            SKIP_CELL_YAW_QUOTA,
            cell,
            yb,
            nearest_keyframe_id=nearest_id,
            nearest_distance_m=nearest_d,
            nearest_yaw_delta_deg=nearest_dyaw,
        )

    # Dedup first among near neighbors (Active + Candidate)
    dedup: DedupResult = evaluate_dedup(
        model, x=x, y=y, yaw=yaw, query_descriptors=query_descriptors, cfg=cfg
    )
    if dedup.is_duplicate:
        model.stats['duplicate_skips'] += 1
        return CaptureDecision(
            False,
            SKIP_DUPLICATE,
            cell,
            yb,
            nearest_keyframe_id=dedup.nearest_keyframe_id or nearest_id,
            nearest_distance_m=dedup.nearest_distance_m,
            nearest_yaw_delta_deg=dedup.nearest_yaw_delta_deg,
            visual_difference=dedup.visual_difference,
        )

    # Condition A — empty cell
    if not model.cell_has_any(cell):
        model.stats['captures'] += 1
        return CaptureDecision(
            True,
            CAPTURE_NEW_CELL,
            cell,
            yb,
            nearest_keyframe_id=nearest_id,
            nearest_distance_m=nearest_d,
            nearest_yaw_delta_deg=nearest_dyaw,
            visual_difference=dedup.visual_difference,
        )

    # Condition B — yaw bin missing in this cell
    if not model.cell_yaw_covered(cell, yb):
        model.stats['captures'] += 1
        return CaptureDecision(
            True,
            CAPTURE_NEW_YAW,
            cell,
            yb,
            nearest_keyframe_id=nearest_id,
            nearest_distance_m=nearest_d,
            nearest_yaw_delta_deg=nearest_dyaw,
            visual_difference=dedup.visual_difference,
        )

    # Condition E — same cell+yaw but visually novel vs existing in slot
    vdiff_slot, vdiff_id = _vdiff_vs_cell_yaw(model, cell, yb, query_descriptors)
    if vdiff_slot is not None and vdiff_slot >= min_vdiff:
        model.stats['novelty_captures'] += 1
        model.stats['captures'] += 1
        return CaptureDecision(
            True,
            CAPTURE_VISUAL_NOVELTY,
            cell,
            yb,
            nearest_keyframe_id=vdiff_id or dedup.nearest_keyframe_id or nearest_id,
            nearest_distance_m=dedup.nearest_distance_m
            if dedup.nearest_keyframe_id
            else nearest_d,
            nearest_yaw_delta_deg=dedup.nearest_yaw_delta_deg
            if dedup.nearest_keyframe_id
            else nearest_dyaw,
            visual_difference=float(vdiff_slot),
            possible_new_appearance=True,
        )

    # Also honor dedup VISUAL_NOVELTY when near-neighbor but different appearance
    if dedup.reason == 'VISUAL_NOVELTY' and (
        dedup.visual_difference is not None and dedup.visual_difference >= min_vdiff
    ):
        model.stats['novelty_captures'] += 1
        model.stats['captures'] += 1
        return CaptureDecision(
            True,
            CAPTURE_VISUAL_NOVELTY,
            cell,
            yb,
            nearest_keyframe_id=dedup.nearest_keyframe_id,
            nearest_distance_m=dedup.nearest_distance_m,
            nearest_yaw_delta_deg=dedup.nearest_yaw_delta_deg,
            visual_difference=dedup.visual_difference,
            possible_new_appearance=True,
        )

    # Condition C — translation vs nearest
    if nearest is not None and nearest_d >= min_trans:
        model.stats['captures'] += 1
        return CaptureDecision(
            True,
            CAPTURE_TRANSLATION,
            cell,
            yb,
            nearest_keyframe_id=nearest_id,
            nearest_distance_m=nearest_d,
            nearest_yaw_delta_deg=nearest_dyaw,
            visual_difference=dedup.visual_difference,
        )

    # Condition D — yaw change vs nearest
    if nearest is not None and nearest_dyaw >= min_yaw:
        model.stats['captures'] += 1
        return CaptureDecision(
            True,
            CAPTURE_YAW_CHANGE,
            cell,
            yb,
            nearest_keyframe_id=nearest_id,
            nearest_distance_m=nearest_d,
            nearest_yaw_delta_deg=nearest_dyaw,
            visual_difference=dedup.visual_difference,
        )

    model.stats['covered_skips'] += 1
    return CaptureDecision(
        False,
        SKIP_COVERED,
        cell,
        yb,
        nearest_keyframe_id=nearest_id,
        nearest_distance_m=nearest_d,
        nearest_yaw_delta_deg=nearest_dyaw,
        visual_difference=dedup.visual_difference or vdiff_slot,
    )
