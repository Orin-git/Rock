"""Laser local search around a visual seed pose."""

from __future__ import annotations

import math
import time
from dataclasses import dataclass
from typing import List, Optional, Tuple

from sensor_msgs.msg import LaserScan

from xw_global_reloc.laser_verify import DistanceField, LaserScore, score_scan_at_pose
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


def _yaw_norm(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


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
    min_laser_score: float = 0.45,
    min_margin: float = 0.03,
    max_refine_trans_m: float = 1.2,
    max_refine_yaw_rad: float = math.radians(35.0),
    top_n_coarse: int = 5,
) -> LaserRefineResult:
    t0 = time.monotonic()

    def grid_search(cx, cy, cyaw, xy_r, yaw_r, xy_s, yaw_s) -> List[Tuple[Pose2D, LaserScore]]:
        out: List[Tuple[Pose2D, LaserScore]] = []
        x = cx - xy_r
        while x <= cx + xy_r + 1e-9:
            y = cy - xy_r
            while y <= cy + xy_r + 1e-9:
                yaw = cyaw - yaw_r
                while yaw <= cyaw + yaw_r + 1e-9:
                    pose = Pose2D(x, y, _yaw_norm(yaw))
                    if not field.is_free(pose.x, pose.y):
                        yaw += yaw_s
                        continue
                    sc = score_scan_at_pose(
                        field,
                        scan,
                        pose.x,
                        pose.y,
                        pose.yaw,
                        beam_stride=beam_stride,
                        match_dist_m=match_dist_m,
                        min_valid_beams=min_valid_beams,
                        min_laser_score=min_laser_score,
                    )
                    out.append((pose, sc))
                    yaw += yaw_s
                y += xy_s
            x += xy_s
        out.sort(key=lambda p: p[1].laser_score, reverse=True)
        return out

    coarse = grid_search(
        seed.x,
        seed.y,
        seed.yaw,
        coarse_xy_m,
        coarse_yaw_rad,
        coarse_xy_step,
        coarse_yaw_step,
    )
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
        )

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

    ok = True
    reason = 'ok'
    if not best_score.accepted or best_score.laser_score < min_laser_score:
        ok = False
        reason = 'laser_score_gate'
    elif not field.is_free(best_pose.x, best_pose.y):
        ok = False
        reason = 'occupied'
    elif margin < min_margin:
        ok = False
        reason = 'ambiguous_margin'
    elif trans > max_refine_trans_m or abs(dyaw) > max_refine_yaw_rad:
        ok = False
        reason = 'refine_too_far_from_visual_seed'

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
        time.monotonic() - t0,
    )
