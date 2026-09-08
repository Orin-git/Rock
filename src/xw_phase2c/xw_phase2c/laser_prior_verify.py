"""Laser verify for BOOT P1/P2 soft priors — thr frozen at 0.38.

Reuses xw_global_reloc.laser_verify DistanceField / score_scan_at_pose.
Must only run when /scan is fresh (LiDAR may lag ~25s after bringup).
"""

from __future__ import annotations

import math
from typing import Any, Dict, Optional, Tuple

import numpy as np
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan

from xw_global_reloc.laser_verify import (
    DistanceField,
    LaserScore,
    prepare_scan,
    score_scan_at_pose,
    score_scan_at_poses,
)

# Frozen Phase2A/B threshold — do not change in C2.
MIN_LASER_SCORE = 0.38


def verify_pose_with_laser(
    pose_xy_yaw: Tuple[float, float, float],
    scan: LaserScan,
    occupancy_map: OccupancyGrid,
    *,
    min_score: float = MIN_LASER_SCORE,
    beam_stride: int = 6,
    match_dist_m: float = 0.25,
    min_valid_beams: int = 20,
    field: Optional[DistanceField] = None,
) -> Dict[str, Any]:
    """Score scan at proposed map pose. Never publishes /initialpose."""
    x, y, yaw = float(pose_xy_yaw[0]), float(pose_xy_yaw[1]), float(pose_xy_yaw[2])
    df = field if field is not None else DistanceField(occupancy_map)
    sc: LaserScore = score_scan_at_pose(
        df,
        scan,
        x,
        y,
        yaw,
        beam_stride=beam_stride,
        match_dist_m=match_dist_m,
        min_valid_beams=min_valid_beams,
        min_laser_score=float(min_score),
    )
    return {
        'ok': bool(sc.accepted),
        'implemented': True,
        'status': 'pass' if sc.accepted else 'reject',
        'reason': sc.reason,
        'laser_score': float(sc.laser_score),
        'matched_ratio': float(sc.matched_ratio),
        'valid_beams': int(sc.valid_beams),
        'mean_dist': float(sc.mean_dist),
        'runtime_sec': float(sc.runtime_sec),
        'min_laser_score': float(min_score),
        'pose': {'x': x, 'y': y, 'yaw': yaw},
        'note': 'SOFT_PRIOR_VERIFY — proposal only until AMCL READY window',
    }


def verify_prior_in_window(
    pose_xy_yaw: Tuple[float, float, float],
    scan: LaserScan,
    occupancy_map: OccupancyGrid,
    *,
    min_score: float = MIN_LASER_SCORE,
    field: Optional[DistanceField] = None,
    xy_radius_m: float = 0.35,
    yaw_radius_rad: float = math.radians(20.0),
    xy_step_m: float = 0.05,
    yaw_step_rad: float = math.radians(5.0),
) -> Dict[str, Any]:
    """Score a stored prior, then a tight local window. Gate stays min_score.

    Used for the charger waypoint, which can sit ~15° / 0.25 m off the dock
    peak. This is not a global search and does not change the 0.38 gate.
    """
    exact = verify_pose_with_laser(
        pose_xy_yaw, scan, occupancy_map, min_score=min_score, field=field
    )
    if exact.get('ok'):
        exact['refined'] = False
        return exact
    df = field if field is not None else DistanceField(occupancy_map)
    sx, sy, syaw = float(pose_xy_yaw[0]), float(pose_xy_yaw[1]), float(pose_xy_yaw[2])
    xs = np.arange(sx - xy_radius_m, sx + xy_radius_m + 1e-9, xy_step_m)
    ys = np.arange(sy - xy_radius_m, sy + xy_radius_m + 1e-9, xy_step_m)
    yaws = np.arange(syaw - yaw_radius_rad, syaw + yaw_radius_rad + 1e-9, yaw_step_rad)
    xx, yy, yw = np.meshgrid(xs, ys, yaws, indexing='ij')
    flat_x = xx.reshape(-1)
    flat_y = yy.reshape(-1)
    flat_w = yw.reshape(-1)
    free = np.array([df.is_free(float(x), float(y)) for x, y in zip(flat_x, flat_y)])
    if not np.any(free):
        exact['refined'] = False
        exact['reason'] = exact.get('reason') or 'no_free_candidates'
        return exact
    prep = prepare_scan(scan, beam_stride=6)
    scores = score_scan_at_poses(
        df,
        prep,
        flat_x[free],
        flat_y[free],
        flat_w[free],
        min_laser_score=float(min_score),
    )
    best_i = max(range(len(scores)), key=lambda i: scores[i].laser_score)
    best = scores[best_i]
    bx = float(flat_x[free][best_i])
    by = float(flat_y[free][best_i])
    bw = float(flat_w[free][best_i])
    out = {
        'ok': bool(best.accepted),
        'implemented': True,
        'status': 'pass' if best.accepted else 'reject',
        'reason': best.reason if best.accepted else 'laser_gate',
        'laser_score': float(best.laser_score),
        'matched_ratio': float(best.matched_ratio),
        'valid_beams': int(best.valid_beams),
        'mean_dist': float(best.mean_dist),
        'runtime_sec': float(best.runtime_sec),
        'min_laser_score': float(min_score),
        'pose': {'x': bx, 'y': by, 'yaw': bw},
        'seed_pose': {'x': sx, 'y': sy, 'yaw': syaw},
        'refined': True,
        'exact_laser_score': float(exact.get('laser_score') or 0.0),
        'note': 'CHARGER_PRIOR_LOCAL_WINDOW — same 0.38 gate, not a global search',
    }
    return out
