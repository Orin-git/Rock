"""SE(2) pose-cluster acceptance for Visual+Laser.

Local-grid neighbor score flatness is diagnostic only.
Ambiguity is decided only between spatially distinct clusters.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Sequence, Tuple


def _yaw_norm(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


def yaw_err(a: float, b: float) -> float:
    return abs(_yaw_norm(a - b))


def se2_close(
    a: Sequence[float],
    b: Sequence[float],
    *,
    cluster_xy_m: float,
    cluster_yaw_rad: float,
) -> bool:
    return math.hypot(a[0] - b[0], a[1] - b[1]) <= cluster_xy_m and yaw_err(a[2], b[2]) <= cluster_yaw_rad


@dataclass
class ClusterMember:
    keyframe_id: str
    refined: Tuple[float, float, float]
    seed: Tuple[float, float, float]
    laser_score: float
    visual_rank: int
    visual_score: float
    visual_region: str = ''
    dx: float = 0.0
    dy: float = 0.0
    dyaw: float = 0.0
    valid_beams: Optional[int] = None
    matched_ratio: Optional[float] = None
    mean_dist: Optional[float] = None
    p90_dist: Optional[float] = None
    absolute_ok: bool = True
    absolute_reason: str = 'ok'
    extra: Dict[str, Any] = field(default_factory=dict)


@dataclass
class PoseCluster:
    cluster_id: int
    center: Tuple[float, float, float]  # best-score member pose
    best_laser_score: float
    median_laser_score: float
    support_count: int
    member_ids: List[str]
    member_regions: List[str]
    visual_ranks: List[int]
    pose_spread_xy_m: float
    pose_spread_yaw_rad: float
    members: List[ClusterMember]


@dataclass
class ClusterAcceptDecision:
    status: str  # ACCEPT | UNKNOWN
    reason: str
    best_cluster: Optional[PoseCluster]
    second_cluster: Optional[PoseCluster]
    cluster_margin: float
    clusters: List[PoseCluster]
    survivors: List[ClusterMember]
    rejected: List[ClusterMember]


def absolute_gate_member(
    *,
    refined: Tuple[float, float, float],
    seed: Tuple[float, float, float],
    laser_score: float,
    min_laser_score: float = 0.45,
    max_refine_trans_m: float = 1.2,
    max_refine_yaw_rad: float = math.radians(35.0),
    free_space: Optional[bool] = None,
    valid_beams: Optional[int] = None,
    min_valid_beams: int = 20,
    legacy_reason: str = '',
) -> Tuple[bool, str, float, float, float]:
    """Absolute hard gates (no local-grid margin). Returns ok, reason, dx, dy, dyaw."""
    dx = refined[0] - seed[0]
    dy = refined[1] - seed[1]
    dyaw = _yaw_norm(refined[2] - seed[2])
    if free_space is False or legacy_reason in ('occupied', 'no_free_candidates'):
        return False, 'occupied', dx, dy, dyaw
    if valid_beams is not None and valid_beams < min_valid_beams:
        return False, 'few_beams', dx, dy, dyaw
    if laser_score < min_laser_score:
        return False, 'laser_score_gate', dx, dy, dyaw
    if math.hypot(dx, dy) > max_refine_trans_m or abs(dyaw) > max_refine_yaw_rad:
        return False, 'refine_too_far_from_visual_seed', dx, dy, dyaw
    if legacy_reason == 'refine_too_far_from_visual_seed':
        return False, 'refine_too_far_from_visual_seed', dx, dy, dyaw
    # legacy ambiguous_margin / ok: local-grid flatness is NOT an absolute reject
    return True, 'ok', dx, dy, dyaw


def cluster_members(
    members: Sequence[ClusterMember],
    *,
    cluster_xy_m: float = 0.25,
    cluster_yaw_rad: float = math.radians(6.0),
) -> List[PoseCluster]:
    """Greedy SE(2) clustering; center = best laser_score member pose."""
    ordered = sorted(members, key=lambda m: m.laser_score, reverse=True)
    clusters: List[PoseCluster] = []
    for m in ordered:
        placed = False
        for cl in clusters:
            if se2_close(m.refined, cl.center, cluster_xy_m=cluster_xy_m, cluster_yaw_rad=cluster_yaw_rad):
                cl.members.append(m)
                placed = True
                break
        if not placed:
            clusters.append(
                PoseCluster(
                    cluster_id=len(clusters),
                    center=m.refined,
                    best_laser_score=m.laser_score,
                    median_laser_score=m.laser_score,
                    support_count=1,
                    member_ids=[m.keyframe_id],
                    member_regions=[m.visual_region] if m.visual_region else [],
                    visual_ranks=[m.visual_rank],
                    pose_spread_xy_m=0.0,
                    pose_spread_yaw_rad=0.0,
                    members=[m],
                )
            )
    # Finalize stats / merge spatially identical centers after growth
    clusters = _merge_overlapping(clusters, cluster_xy_m=cluster_xy_m, cluster_yaw_rad=cluster_yaw_rad)
    for i, cl in enumerate(clusters):
        cl.cluster_id = i
        _finalize_cluster(cl)
    clusters.sort(key=lambda c: c.best_laser_score, reverse=True)
    for i, cl in enumerate(clusters):
        cl.cluster_id = i
    return clusters


def _finalize_cluster(cl: PoseCluster) -> None:
    ms = sorted(cl.members, key=lambda m: m.laser_score, reverse=True)
    cl.members = ms
    best = ms[0]
    cl.center = best.refined
    cl.best_laser_score = best.laser_score
    scores = [m.laser_score for m in ms]
    scores_sorted = sorted(scores)
    mid = len(scores_sorted) // 2
    if len(scores_sorted) % 2:
        cl.median_laser_score = scores_sorted[mid]
    else:
        cl.median_laser_score = 0.5 * (scores_sorted[mid - 1] + scores_sorted[mid])
    cl.support_count = len(ms)
    cl.member_ids = [m.keyframe_id for m in ms]
    regions = []
    for m in ms:
        if m.visual_region and m.visual_region not in regions:
            regions.append(m.visual_region)
    cl.member_regions = regions
    cl.visual_ranks = [m.visual_rank for m in ms]
    if len(ms) == 1:
        cl.pose_spread_xy_m = 0.0
        cl.pose_spread_yaw_rad = 0.0
    else:
        xs = [m.refined[0] for m in ms]
        ys = [m.refined[1] for m in ms]
        yaws = [m.refined[2] for m in ms]
        # max pairwise-ish spread via bbox + max yaw to center
        cl.pose_spread_xy_m = math.hypot(max(xs) - min(xs), max(ys) - min(ys))
        cl.pose_spread_yaw_rad = max(yaw_err(y, best.refined[2]) for y in yaws)


def _merge_overlapping(
    clusters: List[PoseCluster],
    *,
    cluster_xy_m: float,
    cluster_yaw_rad: float,
) -> List[PoseCluster]:
    if len(clusters) <= 1:
        return clusters
    changed = True
    out = list(clusters)
    while changed:
        changed = False
        merged: List[PoseCluster] = []
        used = [False] * len(out)
        for i, a in enumerate(out):
            if used[i]:
                continue
            cur = a
            for j in range(i + 1, len(out)):
                if used[j]:
                    continue
                b = out[j]
                if se2_close(cur.center, b.center, cluster_xy_m=cluster_xy_m, cluster_yaw_rad=cluster_yaw_rad):
                    cur = PoseCluster(
                        cluster_id=cur.cluster_id,
                        center=cur.center,
                        best_laser_score=max(cur.best_laser_score, b.best_laser_score),
                        median_laser_score=0.0,
                        support_count=0,
                        member_ids=[],
                        member_regions=[],
                        visual_ranks=[],
                        pose_spread_xy_m=0.0,
                        pose_spread_yaw_rad=0.0,
                        members=list(cur.members) + list(b.members),
                    )
                    used[j] = True
                    changed = True
            _finalize_cluster(cur)
            merged.append(cur)
            used[i] = True
        out = merged
    return out


def decide_pose_clusters(
    candidates: Sequence[ClusterMember],
    *,
    cluster_xy_m: float = 0.25,
    cluster_yaw_rad: float = math.radians(6.0),
    cluster_min_score_margin: float = 0.03,
) -> ClusterAcceptDecision:
    survivors = [c for c in candidates if c.absolute_ok]
    rejected = [c for c in candidates if not c.absolute_ok]
    if not survivors:
        return ClusterAcceptDecision(
            'UNKNOWN',
            'no_survivor',
            None,
            None,
            0.0,
            [],
            survivors,
            rejected,
        )
    clusters = cluster_members(
        survivors,
        cluster_xy_m=cluster_xy_m,
        cluster_yaw_rad=cluster_yaw_rad,
    )
    best = clusters[0]
    second = clusters[1] if len(clusters) > 1 else None
    margin = best.best_laser_score - (second.best_laser_score if second else 0.0)
    if second is not None and margin < cluster_min_score_margin:
        return ClusterAcceptDecision(
            'UNKNOWN',
            'ambiguous_distinct_clusters',
            best,
            second,
            margin,
            clusters,
            survivors,
            rejected,
        )
    return ClusterAcceptDecision(
        'ACCEPT',
        'best_cluster_ok' if second is None else 'best_cluster_margin_ok',
        best,
        second,
        margin,
        clusters,
        survivors,
        rejected,
    )


def cluster_to_dict(cl: PoseCluster) -> Dict[str, Any]:
    return {
        'cluster_id': cl.cluster_id,
        'center': list(cl.center),
        'best_laser_score': cl.best_laser_score,
        'median_laser_score': cl.median_laser_score,
        'support_count': cl.support_count,
        'member_ids': cl.member_ids,
        'member_regions': cl.member_regions,
        'visual_ranks': cl.visual_ranks,
        'pose_spread_xy_m': cl.pose_spread_xy_m,
        'pose_spread_yaw_rad': cl.pose_spread_yaw_rad,
    }


def decision_to_dict(dec: ClusterAcceptDecision) -> Dict[str, Any]:
    return {
        'status': dec.status,
        'reason': dec.reason,
        'cluster_margin': dec.cluster_margin,
        'cluster_count': len(dec.clusters),
        'best_cluster': None if dec.best_cluster is None else cluster_to_dict(dec.best_cluster),
        'second_cluster': None if dec.second_cluster is None else cluster_to_dict(dec.second_cluster),
        'survivor_count': len(dec.survivors),
        'rejected_count': len(dec.rejected),
    }
