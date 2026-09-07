"""Hard-gate acceptance policy — composite_score is debug ranking only."""

from __future__ import annotations

from dataclasses import dataclass
from typing import List, Optional

from xw_global_reloc.geometry import GeometryResult
from xw_global_reloc.laser_verify import DistanceField, LaserScore
from xw_global_reloc.transforms import Pose2D


@dataclass
class CandidateEval:
    keyframe_id: str
    pose: Pose2D
    visual_score: float
    geometry_score: float
    laser_score: float
    composite_score: float
    geometry: GeometryResult
    laser: LaserScore
    free_space: bool
    accepted: bool
    reason: str


@dataclass
class AcceptDecision:
    status: str  # ACCEPT | UNKNOWN | REJECTED
    best: Optional[CandidateEval]
    margin: float
    reason: str


def composite(visual: float, geometry: float, laser: float) -> float:
    return float(0.35 * visual + 0.35 * geometry + 0.30 * laser)


def decide(
    evals: List[CandidateEval],
    *,
    min_top_margin: float = 0.05,
) -> AcceptDecision:
    survivors = [e for e in evals if e.accepted]
    if not survivors:
        return AcceptDecision('UNKNOWN', None, 0.0, 'no_survivor')
    survivors.sort(key=lambda e: e.composite_score, reverse=True)
    best = survivors[0]
    margin = 0.0
    if len(survivors) >= 2:
        margin = best.composite_score - survivors[1].composite_score
        if margin < min_top_margin:
            return AcceptDecision('UNKNOWN', best, margin, 'ambiguous_top2')
    return AcceptDecision('ACCEPT', best, margin, 'gates_ok')


def evaluate_candidate(
    keyframe_id: str,
    pose: Pose2D,
    geom: GeometryResult,
    laser: LaserScore,
    field: Optional[DistanceField],
) -> CandidateEval:
    free = True if field is None else field.is_free(pose.x, pose.y)
    reason = 'ok'
    ok = True
    if not geom.accepted:
        ok = False
        reason = f'geom:{geom.reason}'
    elif not laser.accepted:
        ok = False
        reason = f'laser:{laser.reason}'
    elif not free:
        ok = False
        reason = 'occupied'
    # Hard rule: high visual / low laser already rejected by laser.accepted
    comp = composite(geom.visual_score, geom.geometry_score, laser.laser_score)
    return CandidateEval(
        keyframe_id=keyframe_id,
        pose=pose,
        visual_score=geom.visual_score,
        geometry_score=geom.geometry_score,
        laser_score=laser.laser_score,
        composite_score=comp,
        geometry=geom,
        laser=laser,
        free_space=free,
        accepted=ok,
        reason=reason,
    )
