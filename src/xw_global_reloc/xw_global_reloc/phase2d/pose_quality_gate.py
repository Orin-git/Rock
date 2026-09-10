"""Pose quality gate for Phase2D Candidate capture (cheap checks, no laser)."""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional


@dataclass
class PoseGateInput:
    localization_status: Optional[int] = None
    phase2c_state: str = ''
    phase2c_loc_state: str = ''
    follow_active: bool = False
    legacy_freeze_active: bool = False
    amcl_cov_xy: Optional[float] = None
    amcl_cov_yaw: Optional[float] = None
    map_base_age_sec: Optional[float] = None
    map_odom_age_sec: Optional[float] = None
    scan_age_sec: Optional[float] = None
    speed_mps: Optional[float] = None
    yaw_rate: Optional[float] = None
    x: Optional[float] = None
    y: Optional[float] = None
    yaw: Optional[float] = None
    map_name: str = ''
    map_hash: str = ''
    scan_present: bool = False


@dataclass
class PoseGateResult:
    ok: bool
    reason: str
    reasons: List[str] = field(default_factory=list)
    metrics: Dict[str, Any] = field(default_factory=dict)

    @property
    def code(self) -> str:
        return 'PASS' if self.ok else 'REJECTED_POSE'


def _finite(v: Optional[float]) -> bool:
    return v is not None and math.isfinite(float(v))


def evaluate_pose_gate(inp: PoseGateInput, cfg: Dict[str, Any]) -> PoseGateResult:
    """All configured pose checks must pass. No laser scoring here."""
    pg = dict(cfg.get('pose_gate') or {})
    reasons: List[str] = []
    metrics: Dict[str, Any] = {
        'localization_status': inp.localization_status,
        'phase2c_state': inp.phase2c_state,
        'phase2c_loc_state': inp.phase2c_loc_state,
        'follow_active': bool(inp.follow_active),
        'legacy_freeze_active': bool(inp.legacy_freeze_active),
        'amcl_cov_xy': inp.amcl_cov_xy,
        'amcl_cov_yaw': inp.amcl_cov_yaw,
        'map_base_age_sec': inp.map_base_age_sec,
        'map_odom_age_sec': inp.map_odom_age_sec,
        'scan_age_sec': inp.scan_age_sec,
        'speed_mps': inp.speed_mps,
        'yaw_rate': inp.yaw_rate,
        'x': inp.x,
        'y': inp.y,
        'yaw': inp.yaw,
        'map_name': inp.map_name,
        'map_hash': inp.map_hash,
    }

    if bool(pg.get('require_localization_status_0', True)):
        if inp.localization_status is None or int(inp.localization_status) != 0:
            reasons.append('localization_status_not_0')

    state = str(inp.phase2c_state or inp.phase2c_loc_state or '').strip()
    blocked = {str(s) for s in (pg.get('blocked_phase2c_states') or [])}
    allowed = {str(s) for s in (pg.get('allowed_phase2c_states') or ['READY'])}
    if bool(pg.get('require_phase2c_ready', True)):
        if not state:
            reasons.append('phase2c_state_missing')
        elif state in blocked or state not in allowed:
            reasons.append(f'phase2c_not_ready:{state or "empty"}')

    if bool(pg.get('block_if_follow_active', True)) and inp.follow_active:
        reasons.append('follow_active')

    if bool(pg.get('block_if_legacy_freeze_active', True)) and inp.legacy_freeze_active:
        reasons.append('legacy_freeze_active')

    max_xy = float(pg.get('max_amcl_cov_xy', 0.6))
    max_yaw = float(pg.get('max_amcl_cov_yaw', 0.35))
    if inp.amcl_cov_xy is None or not math.isfinite(float(inp.amcl_cov_xy)):
        reasons.append('amcl_cov_xy_missing')
    elif float(inp.amcl_cov_xy) > max_xy:
        reasons.append('amcl_cov_xy_high')
    if inp.amcl_cov_yaw is None or not math.isfinite(float(inp.amcl_cov_yaw)):
        reasons.append('amcl_cov_yaw_missing')
    elif float(inp.amcl_cov_yaw) > max_yaw:
        reasons.append('amcl_cov_yaw_high')

    max_mb = float(pg.get('max_map_base_age_sec', 1.5))
    if inp.map_base_age_sec is None:
        reasons.append('map_base_tf_missing')
    elif float(inp.map_base_age_sec) > max_mb:
        reasons.append('map_base_tf_stale')

    max_mo = float(pg.get('max_map_odom_age_sec', 1.5))
    if inp.map_odom_age_sec is None:
        reasons.append('map_odom_tf_missing')
    elif float(inp.map_odom_age_sec) > max_mo:
        reasons.append('map_odom_tf_stale')

    max_scan = float(pg.get('max_scan_age_sec', 1.0))
    if not inp.scan_present or inp.scan_age_sec is None:
        reasons.append('scan_missing')
    elif float(inp.scan_age_sec) > max_scan:
        reasons.append('scan_stale')

    if bool(pg.get('require_finite_pose', True)):
        if not (_finite(inp.x) and _finite(inp.y) and _finite(inp.yaw)):
            reasons.append('pose_not_finite')

    max_spd = float(pg.get('max_speed_mps', 0.20))
    max_yr = float(pg.get('max_yaw_rate', 0.35))
    if inp.speed_mps is None or not math.isfinite(float(inp.speed_mps)):
        reasons.append('speed_missing')
    elif abs(float(inp.speed_mps)) > max_spd:
        reasons.append('speed_too_high')
    if inp.yaw_rate is None or not math.isfinite(float(inp.yaw_rate)):
        reasons.append('yaw_rate_missing')
    elif abs(float(inp.yaw_rate)) > max_yr:
        reasons.append('yaw_rate_too_high')

    if bool(pg.get('require_map_name', True)) and not str(inp.map_name or '').strip():
        reasons.append('map_name_missing')
    if bool(pg.get('require_map_hash', True)) and not str(inp.map_hash or '').strip():
        reasons.append('map_hash_missing')

    if reasons:
        return PoseGateResult(False, reasons[0], reasons=reasons, metrics=metrics)
    return PoseGateResult(True, 'ok', reasons=[], metrics=metrics)
