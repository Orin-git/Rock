"""Keyframe quality helpers: retrieval_ready vs geometry_ready."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

import numpy as np

from xw_global_reloc.orb_utils import OrbFrame


@dataclass
class QualityVerdict:
    retrieval_ready: bool
    geometry_ready: bool
    orb_total: int
    orb_with_valid_depth: int
    orb_depth_coverage_ratio: float
    depth_valid_ratio: float
    pair_dt_sec: float
    reasons: list


def orb_depth_coverage(orb: OrbFrame, depth_u16: np.ndarray) -> tuple:
    h, w = depth_u16.shape[:2]
    with_d = 0
    for kp in orb.keypoints:
        u, v = int(round(kp.pt[0])), int(round(kp.pt[1]))
        if 0 <= u < w and 0 <= v < h and int(depth_u16[v, u]) > 0:
            with_d += 1
    total = int(orb.n_features)
    cov = float(with_d) / float(max(total, 1))
    return total, with_d, cov


def evaluate_quality(
    *,
    pose_ok: bool,
    camera_info_ok: bool,
    orb: OrbFrame,
    depth_u16: Optional[np.ndarray],
    pair_dt_sec: float,
    min_orb: int,
    max_pair_dt: float,
    min_depth_valid: float,
    min_orb_depth_cov: float,
) -> QualityVerdict:
    reasons = []
    orb_total = int(orb.n_features)
    with_d = 0
    cov = 0.0
    valid = 0.0
    if depth_u16 is not None:
        valid = float(np.count_nonzero(depth_u16 > 0)) / float(max(depth_u16.size, 1))
        orb_total, with_d, cov = orb_depth_coverage(orb, depth_u16)

    retrieval = True
    if not pose_ok:
        retrieval = False
        reasons.append('pose_unhealthy')
    if orb_total < min_orb:
        retrieval = False
        reasons.append('orb_low')
    if not camera_info_ok:
        # retrieval can proceed without depth info, but flag
        reasons.append('camera_info_missing')

    geometry = retrieval and camera_info_ok
    if depth_u16 is None:
        geometry = False
        reasons.append('no_depth')
    if pair_dt_sec > max_pair_dt:
        geometry = False
        reasons.append('pair_dt')
    if valid < min_depth_valid and cov < min_orb_depth_cov:
        # allow geometry if ORB coverage OK even when full-frame valid low
        geometry = False
        reasons.append('depth_or_orb_coverage')
    elif cov < min_orb_depth_cov:
        geometry = False
        reasons.append('orb_depth_coverage')

    # If full-frame valid low but orb coverage OK → geometry allowed
    if (
        retrieval
        and camera_info_ok
        and depth_u16 is not None
        and pair_dt_sec <= max_pair_dt
        and cov >= min_orb_depth_cov
    ):
        geometry = True
        reasons = [r for r in reasons if r not in ('depth_or_orb_coverage', 'orb_depth_coverage')]

    return QualityVerdict(
        retrieval_ready=retrieval,
        geometry_ready=bool(geometry),
        orb_total=orb_total,
        orb_with_valid_depth=with_d,
        orb_depth_coverage_ratio=cov,
        depth_valid_ratio=valid,
        pair_dt_sec=pair_dt_sec,
        reasons=reasons,
    )
