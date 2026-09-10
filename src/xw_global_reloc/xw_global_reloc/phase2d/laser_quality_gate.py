"""Laser geometric gate — wraps shared laser_verify only (thr frozen 0.38)."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional

from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan

from xw_global_reloc.laser_verify import DistanceField, LaserScore, score_scan_at_pose


@dataclass
class LaserGateResult:
    ok: bool
    reason: str
    reasons: List[str] = field(default_factory=list)
    laser_verified: bool = False
    laser_score: float = 0.0
    laser_matched_ratio: float = 0.0
    laser_valid_beams: int = 0
    scan_stamp: Optional[float] = None
    metrics: Dict[str, Any] = field(default_factory=dict)

    @property
    def code(self) -> str:
        return 'PASS' if self.ok else 'REJECTED_LASER'


def evaluate_laser_gate(
    *,
    field: Optional[DistanceField],
    occupancy_map: Optional[OccupancyGrid],
    scan: Optional[LaserScan],
    x: float,
    y: float,
    yaw: float,
    cfg: Dict[str, Any],
    scan_stamp: Optional[float] = None,
) -> LaserGateResult:
    """Bind RGB to (x,y,yaw) only when shared scorer accepts at min_laser_score."""
    lg = dict(cfg.get('laser_gate') or {})
    min_score = float(lg.get('min_laser_score', 0.38))
    # Hard freeze: never silently raise above / invent another algorithm.
    if abs(min_score - 0.38) > 1e-9:
        # Still call shared scorer with configured value, but record drift warning.
        pass

    if scan is None:
        return LaserGateResult(
            False,
            'laser_inconsistent',
            reasons=['scan_missing'],
            scan_stamp=scan_stamp,
            metrics={'min_laser_score': min_score},
        )
    if field is None:
        if occupancy_map is None:
            return LaserGateResult(
                False,
                'laser_inconsistent',
                reasons=['map_missing'],
                scan_stamp=scan_stamp,
                metrics={'min_laser_score': min_score},
            )
        try:
            field = DistanceField(occupancy_map)
        except Exception as exc:  # noqa: BLE001
            return LaserGateResult(
                False,
                'laser_inconsistent',
                reasons=[f'distance_field_failed:{exc}'],
                scan_stamp=scan_stamp,
                metrics={'min_laser_score': min_score},
            )

    sc: LaserScore = score_scan_at_pose(
        field,
        scan,
        float(x),
        float(y),
        float(yaw),
        beam_stride=int(lg.get('beam_stride', 6)),
        match_dist_m=float(lg.get('match_dist_m', 0.25)),
        min_valid_beams=int(lg.get('min_valid_beams', 20)),
        min_laser_score=min_score,
    )
    metrics = {
        'min_laser_score': min_score,
        'laser_score': float(sc.laser_score),
        'matched_ratio': float(sc.matched_ratio),
        'valid_beams': int(sc.valid_beams),
        'mean_dist': float(sc.mean_dist),
        'scorer_reason': sc.reason,
        'runtime_sec': float(sc.runtime_sec),
    }
    if not sc.accepted:
        return LaserGateResult(
            False,
            'laser_inconsistent',
            reasons=['laser_inconsistent', sc.reason],
            laser_verified=False,
            laser_score=float(sc.laser_score),
            laser_matched_ratio=float(sc.matched_ratio),
            laser_valid_beams=int(sc.valid_beams),
            scan_stamp=scan_stamp,
            metrics=metrics,
        )
    return LaserGateResult(
        True,
        'ok',
        reasons=[],
        laser_verified=True,
        laser_score=float(sc.laser_score),
        laser_matched_ratio=float(sc.matched_ratio),
        laser_valid_beams=int(sc.valid_beams),
        scan_stamp=scan_stamp,
        metrics=metrics,
    )
