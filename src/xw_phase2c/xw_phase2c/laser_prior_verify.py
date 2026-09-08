"""Laser verify for BOOT P1/P2 soft priors — thr frozen at 0.38.

Reuses xw_global_reloc.laser_verify DistanceField / score_scan_at_pose.
Must only run when /scan is fresh (LiDAR may lag ~25s after bringup).
"""

from __future__ import annotations

from typing import Any, Dict, Optional, Tuple

from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan

from xw_global_reloc.laser_verify import DistanceField, LaserScore, score_scan_at_pose

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
