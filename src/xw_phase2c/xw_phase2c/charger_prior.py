"""Charger soft-prior helper — never publishes /initialpose.

charger_prior_available=True means SOFT PRIOR only (charging/docked evidence +
charger waypoint exists). Must laser-verify before any seed (C2).
"""

from __future__ import annotations

from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any, Dict, Optional, Tuple

import yaml


@dataclass
class ChargerPriorResult:
    charger_prior_available: bool
    soft_prior_only: bool = True
    reason: str = ''
    charging: bool = False
    docked: bool = False
    battery_charging: bool = False
    charger_waypoint: Optional[Dict[str, float]] = None
    laser_verify_status: str = 'not_run'

    def as_dict(self) -> Dict[str, Any]:
        d = asdict(self)
        d['policy'] = 'SOFT_PRIOR_ONLY — forbid direct /initialpose'
        return d


def load_charger_waypoint(maps_dir: Path | str, map_name: str) -> Optional[Tuple[float, float, float]]:
    """Load charger from maps/waypoints/<map>_pointList.yaml (gen1 layout)."""
    root = Path(maps_dir)
    candidates = [
        root / 'waypoints' / f'{map_name}_pointList.yaml',
        root / map_name / 'waypoints' / f'{map_name}_pointList.yaml',
        root / 'waypoints' / f'{map_name}_pointList.yml',
    ]
    path = next((p for p in candidates if p.is_file()), None)
    if path is None:
        return None
    try:
        with open(path, 'r', encoding='utf-8') as f:
            data = yaml.safe_load(f) or {}
    except (OSError, yaml.YAMLError):
        return None
    charger = data.get('charger')
    if isinstance(charger, dict):
        try:
            return (
                float(charger['x']),
                float(charger['y']),
                float(charger.get('yaw', charger.get('theta', 0.0)) or 0.0),
            )
        except (KeyError, TypeError, ValueError):
            pass
    for w in data.get('waypoints') or []:
        if str(w.get('name') or '').lower() == 'charger':
            try:
                return (
                    float(w['x']),
                    float(w['y']),
                    float(w.get('yaw', w.get('theta', 0.0)) or 0.0),
                )
            except (KeyError, TypeError, ValueError):
                return None
    return None


def evaluate_charger_soft_prior(
    *,
    charging: bool,
    docked: bool,
    battery_charging: bool,
    maps_dir: Path | str,
    map_name: str,
) -> ChargerPriorResult:
    """Pure helper: available iff (any charge evidence) AND charger waypoint exists."""
    wp = load_charger_waypoint(maps_dir, map_name)
    charge_ev = bool(charging or docked or battery_charging)
    if not charge_ev:
        return ChargerPriorResult(
            charger_prior_available=False,
            reason='no_charging_evidence',
            charging=charging,
            docked=docked,
            battery_charging=battery_charging,
            charger_waypoint=None,
        )
    if wp is None:
        return ChargerPriorResult(
            charger_prior_available=False,
            reason='charger_waypoint_missing',
            charging=charging,
            docked=docked,
            battery_charging=battery_charging,
            charger_waypoint=None,
        )
    return ChargerPriorResult(
        charger_prior_available=True,
        reason='soft_prior_charge_evidence_plus_waypoint',
        charging=charging,
        docked=docked,
        battery_charging=battery_charging,
        charger_waypoint={'x': wp[0], 'y': wp[1], 'yaw': wp[2]},
    )


def verify_charger_with_laser(
    pose_xy_yaw: Tuple[float, float, float],
    scan: Any = None,
    occupancy_map: Any = None,
    *,
    min_score: float = 0.38,
) -> Dict[str, Any]:
    """C2: real laser verify when scan+map provided; else reports missing_inputs."""
    if scan is None or occupancy_map is None:
        return {
            'ok': False,
            'implemented': True,
            'status': 'missing_inputs',
            'note': 'Need fresh /scan + /map; never blind-seed charger',
            'laser_thr': float(min_score),
        }
    from xw_phase2c.laser_prior_verify import verify_pose_with_laser

    return verify_pose_with_laser(
        pose_xy_yaw, scan, occupancy_map, min_score=float(min_score)
    )
