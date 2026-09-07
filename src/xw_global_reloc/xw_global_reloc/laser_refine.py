"""Laser local search around a visual seed pose."""

from __future__ import annotations

import math
import time
from dataclasses import dataclass, field as dc_field
from typing import Dict, List, Optional, Tuple

import numpy as np
from sensor_msgs.msg import LaserScan

from xw_global_reloc.laser_verify import (
    DistanceField,
    LaserScore,
    PreparedScan,
    prepare_scan,
    score_scan_at_poses,
)
from xw_global_reloc.transforms import Pose2D


@dataclass
class LaserRefineResult:
    accepted: bool
    reason: str
    seed: Pose2D
    refined: Pose2D
    score: LaserScore
    dx: float
    dy: float
    dyaw: float
    top1_score: float
    top2_score: float
    margin: float
    candidates_evaluated: int
    runtime_sec: float
    stage_timings: Dict[str, float] = dc_field(default_factory=dict)


def _yaw_norm(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


def _build_pose_grid(
    cx: float,
    cy: float,
    cyaw: float,
    xy_r: float,
    yaw_r: float,
    xy_s: float,
    yaw_s: float,
    field: DistanceField,
) -> Tuple[np.ndarray, np.ndarray, np.ndarray, List[Pose2D]]:
    xs: List[float] = []
    ys: List[float] = []
    yaws: List[float] = []
    poses: List[Pose2D] = []
    x = cx - xy_r
    while x <= cx + xy_r + 1e-9:
        y = cy - xy_r
        while y <= cy + xy_r + 1e-9:
            yaw = cyaw - yaw_r
            while yaw <= cyaw + yaw_r + 1e-9:
                yn = _yaw_norm(yaw)
                if field.is_free(x, y):
                    xs.append(x)
                    ys.append(y)
                    yaws.append(yn)
                    poses.append(Pose2D(x, y, yn))
                yaw += yaw_s
            y += xy_s
        x += xy_s
    return (
        np.asarray(xs, dtype=np.float64),
        np.asarray(ys, dtype=np.float64),
        np.asarray(yaws, dtype=np.float64),
        poses,
    )


def refine_candidate_with_laser(
    field: DistanceField,
    scan: LaserScan,
    seed: Pose2D,
    *,
    coarse_xy_m: float = 1.0,
    coarse_yaw_rad: float = math.radians(30.0),
    coarse_xy_step: float = 0.10,
    coarse_yaw_step: float = math.radians(3.0),
    fine_xy_m: float = 0.20,
    fine_yaw_rad: float = math.radians(5.0),
    fine_xy_step: float = 0.05,
    fine_yaw_step: float = math.radians(1.0),
    beam_stride: int = 6,
    match_dist_m: float = 0.25,
    min_valid_beams: int = 20,
    min_laser_score: float = 0.38,
    min_margin: float = 0.03,
    max_refine_trans_m: float = 1.2,
    max_refine_yaw_rad: float = math.radians(35.0),
    top_n_coarse: int = 5,
    reject_local_grid_margin: bool = False,
    prepared: Optional[PreparedScan] = None,
) -> LaserRefineResult:
    t0 = time.monotonic()
    timings: Dict[str, float] = {}

    t_prep = time.monotonic()
    prep = prepared if prepared is not None else prepare_scan(scan, beam_stride=beam_stride)
    timings['prepare_scan'] = time.monotonic() - t_prep

    def grid_search(cx, cy, cyaw, xy_r, yaw_r, xy_s, yaw_s) -> List[Tuple[Pose2D, LaserScore]]:
        xs, ys, yaws, poses = _build_pose_grid(cx, cy, cyaw, xy_r, yaw_r, xy_s, yaw_s, field)
        if not poses:
            return []
        scores = score_scan_at_poses(
            field,
            prep,
            xs,
            ys,
            yaws,
            match_dist_m=match_dist_m,
            min_valid_beams=min_valid_beams,
            min_laser_score=min_laser_score,
        )
        out = list(zip(poses, scores))
        out.sort(key=lambda p: p[1].laser_score, reverse=True)
        return out

    t_c = time.monotonic()
    coarse = grid_search(
        seed.x,
        seed.y,
        seed.yaw,
        coarse_xy_m,
        coarse_yaw_rad,
        coarse_xy_step,
        coarse_yaw_step,
    )
    timings['coarse_search'] = time.monotonic() - t_c
    n_eval = len(coarse)
    if not coarse:
        return LaserRefineResult(
            False,
            'no_free_candidates',
            seed,
            seed,
            LaserScore(False, 'empty', 0, 0.0, 999.0, 999.0, 0.0, 0.0),
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0,
            time.monotonic() - t0,
            timings,
        )

    t_f = time.monotonic()
    refined_pool: List[Tuple[Pose2D, LaserScore]] = []
    for pose, _ in coarse[: max(1, top_n_coarse)]:
        refined_pool.extend(
            grid_search(
                pose.x,
                pose.y,
                pose.yaw,
                fine_xy_m,
                fine_yaw_rad,
                fine_xy_step,
                fine_yaw_step,
            )
        )
    timings['fine_search'] = time.monotonic() - t_f
    refined_pool.sort(key=lambda p: p[1].laser_score, reverse=True)
    n_eval += len(refined_pool)
    best_pose, best_score = refined_pool[0] if refined_pool else coarse[0]
    top2 = refined_pool[1][1].laser_score if len(refined_pool) > 1 else (
        coarse[1][1].laser_score if len(coarse) > 1 else 0.0
    )
    margin = best_score.laser_score - top2
    dx = best_pose.x - seed.x
    dy = best_pose.y - seed.y
    dyaw = _yaw_norm(best_pose.yaw - seed.yaw)
    trans = math.hypot(dx, dy)

    # Absolute hard gates only. Local-grid neighbor flatness (margin) is
    # diagnostic pose uncertainty — not place ambiguity.
    ok = True
    reason = 'ok'
    if not best_score.accepted or best_score.laser_score < min_laser_score:
        ok = False
        reason = 'laser_score_gate'
    elif not field.is_free(best_pose.x, best_pose.y):
        ok = False
        reason = 'occupied'
    elif trans > max_refine_trans_m or abs(dyaw) > max_refine_yaw_rad:
        ok = False
        reason = 'refine_too_far_from_visual_seed'
    elif reject_local_grid_margin and margin < min_margin:
        ok = False
        reason = 'ambiguous_margin'

    timings['total'] = time.monotonic() - t0
    return LaserRefineResult(
        ok,
        reason,
        seed,
        best_pose,
        best_score,
        dx,
        dy,
        dyaw,
        best_score.laser_score,
        float(top2),
        float(margin),
        n_eval,
        timings['total'],
        timings,
    )
