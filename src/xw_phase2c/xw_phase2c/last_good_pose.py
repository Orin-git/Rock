"""last_good_pose.yaml read / write / validate — proposal only, never final pose.

Path: maps/<map>/state/last_good_pose.yaml
Atomic replace to avoid power-loss corruption.
"""

from __future__ import annotations

import hashlib
import math
import os
import time
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Any, Dict, Optional, Tuple

import yaml


@dataclass
class LastGoodPose:
    map_name: str
    map_hash: str
    timestamp: float
    x: float
    y: float
    yaw: float
    covariance: list  # [xx, yy, yaw] preferred
    source: str = 'amcl'
    quality: float = 0.0

    def as_dict(self) -> Dict[str, Any]:
        d = asdict(self)
        d['note'] = 'PROPOSAL_ONLY — must laser-verify before /initialpose'
        return d


@dataclass
class ValidateResult:
    ok: bool
    reason: str = ''
    pose: Optional[LastGoodPose] = None


def state_dir(maps_dir: Path | str, map_name: str) -> Path:
    return Path(maps_dir) / map_name / 'state'


def pose_path(maps_dir: Path | str, map_name: str) -> Path:
    return state_dir(maps_dir, map_name) / 'last_good_pose.yaml'


def compute_map_hash(maps_dir: Path | str, map_name: str) -> str:
    """SHA256 over map yaml + pgm bytes (order: yaml then pgm)."""
    root = Path(maps_dir)
    h = hashlib.sha256()
    found = False
    for suffix in ('.yaml', '.pgm'):
        p = root / f'{map_name}{suffix}'
        if p.is_file():
            h.update(suffix.encode())
            h.update(p.read_bytes())
            found = True
    if not found:
        return ''
    return h.hexdigest()


def quality_from_cov(xx: float, yy: float, yaw: float, xy_good: float, yaw_good: float) -> float:
    """1.0 when well below good thresholds; →0 as cov approaches/exceeds."""
    xy = max(float(xx), float(yy))
    q_xy = max(0.0, 1.0 - (xy / max(xy_good, 1e-6)))
    q_yaw = max(0.0, 1.0 - (float(yaw) / max(yaw_good, 1e-6)))
    return float(max(0.0, min(1.0, 0.6 * q_xy + 0.4 * q_yaw)))


def atomic_write_yaml(path: Path, data: Dict[str, Any]) -> None:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_suffix(path.suffix + '.tmp')
    text = yaml.safe_dump(data, default_flow_style=False, sort_keys=False)
    with open(tmp, 'w', encoding='utf-8') as f:
        f.write(text)
        f.flush()
        os.fsync(f.fileno())
    os.replace(tmp, path)


def write_last_good_pose(maps_dir: Path | str, pose: LastGoodPose) -> Path:
    path = pose_path(maps_dir, pose.map_name)
    atomic_write_yaml(path, pose.as_dict())
    return path


def read_last_good_pose(maps_dir: Path | str, map_name: str) -> Optional[LastGoodPose]:
    path = pose_path(maps_dir, map_name)
    if not path.is_file():
        return None
    try:
        with open(path, 'r', encoding='utf-8') as f:
            raw = yaml.safe_load(f) or {}
    except (OSError, yaml.YAMLError):
        return None
    try:
        cov = raw.get('covariance') or [999.0, 999.0, 999.0]
        if isinstance(cov, dict):
            cov = [float(cov.get('xx', 999)), float(cov.get('yy', 999)), float(cov.get('yaw', 999))]
        return LastGoodPose(
            map_name=str(raw.get('map_name') or map_name),
            map_hash=str(raw.get('map_hash') or ''),
            timestamp=float(raw.get('timestamp') or 0.0),
            x=float(raw['x']),
            y=float(raw['y']),
            yaw=float(raw['yaw']),
            covariance=[float(cov[0]), float(cov[1]), float(cov[2])],
            source=str(raw.get('source') or 'amcl'),
            quality=float(raw.get('quality') or 0.0),
        )
    except (KeyError, TypeError, ValueError, IndexError):
        return None


def validate_as_proposal(
    maps_dir: Path | str,
    map_name: str,
    *,
    max_age_sec: float = 7 * 24 * 3600.0,
    min_quality: float = 0.35,
    now: Optional[float] = None,
    expected_hash: Optional[str] = None,
) -> ValidateResult:
    """Validate file as BOOT/LOST *proposal only* — never implies final localization."""
    pose = read_last_good_pose(maps_dir, map_name)
    if pose is None:
        return ValidateResult(False, 'missing_or_unreadable')
    if pose.map_name and pose.map_name != map_name:
        return ValidateResult(False, 'map_name_mismatch', pose)
    cur_hash = expected_hash if expected_hash is not None else compute_map_hash(maps_dir, map_name)
    if not cur_hash:
        return ValidateResult(False, 'current_map_hash_unavailable', pose)
    if not pose.map_hash or pose.map_hash != cur_hash:
        return ValidateResult(False, 'map_hash_mismatch', pose)
    t_now = float(now if now is not None else time.time())
    age = t_now - float(pose.timestamp)
    if age < 0:
        return ValidateResult(False, 'timestamp_in_future', pose)
    if age > float(max_age_sec):
        return ValidateResult(False, 'age_exceeded', pose)
    if float(pose.quality) < float(min_quality):
        return ValidateResult(False, 'quality_too_low', pose)
    return ValidateResult(True, 'ok_proposal_only', pose)


def yaw_from_quat(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def pose_delta(
    a: Tuple[float, float, float], b: Tuple[float, float, float]
) -> Tuple[float, float]:
    dxy = math.hypot(a[0] - b[0], a[1] - b[1])
    dyaw = abs(math.atan2(math.sin(a[2] - b[2]), math.cos(a[2] - b[2])))
    return dxy, dyaw
