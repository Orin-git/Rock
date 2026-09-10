"""Spatial cell + yaw bin helpers for Phase2D coverage."""

from __future__ import annotations

import math
from typing import Any, Dict, Tuple


def wrap_yaw(yaw: float) -> float:
    """Wrap yaw to [0, 2π)."""
    two_pi = 2.0 * math.pi
    y = math.fmod(float(yaw), two_pi)
    if y < 0.0:
        y += two_pi
    # fmod can leave -0.0; also clamp 2π → 0
    if y >= two_pi or abs(y - two_pi) < 1e-12:
        y = 0.0
    return y


def cell_indices(x: float, y: float, cell_size_m: float) -> Tuple[int, int]:
    cs = float(cell_size_m)
    if cs <= 0.0:
        raise ValueError('cell_size_m must be > 0')
    return int(math.floor(float(x) / cs)), int(math.floor(float(y) / cs))


def spatial_cell_id(x: float, y: float, cell_size_m: float = 1.0) -> str:
    cx, cy = cell_indices(x, y, cell_size_m)
    return f'cell_{cx}_{cy}'


def parse_spatial_cell(cell_id: str) -> Tuple[int, int]:
    """Parse 'cell_<cx>_<cy>' including negative indices."""
    s = str(cell_id or '')
    if not s.startswith('cell_'):
        raise ValueError(f'bad spatial_cell id: {cell_id}')
    rest = s[len('cell_') :]
    # split from the right once so negatives work: cell_-3_1 → -3, 1
    # Also cell_-3_-1 → -3, -1
    parts = rest.split('_')
    if len(parts) < 2:
        raise ValueError(f'bad spatial_cell id: {cell_id}')
    # Rejoin carefully: last token is cy, everything before is cx with possible leading '-'
    # For cell_-3_1 parts=['', '3', '1'] after split on '_' from ' -3_1 '? 
    # rest for cell_-3_1 is '-3_1' → split '_' → ['-3', '1']
    # rest for cell_-3_-1 is '-3_-1' → ['-3', '-1']
    cy = int(parts[-1])
    cx = int('_'.join(parts[:-1]))
    return cx, cy


def yaw_bin_index(yaw: float, yaw_bins: int = 8) -> int:
    n = int(yaw_bins)
    if n <= 0:
        raise ValueError('yaw_bins must be > 0')
    y = wrap_yaw(yaw)
    bin_w = (2.0 * math.pi) / float(n)
    idx = int(math.floor(y / bin_w)) % n
    return idx


def yaw_delta_rad(a: float, b: float) -> float:
    return abs(math.atan2(math.sin(float(a) - float(b)), math.cos(float(a) - float(b))))


def yaw_delta_deg(a: float, b: float) -> float:
    return math.degrees(yaw_delta_rad(a, b))


def coverage_params(cfg: Dict[str, Any]) -> Dict[str, Any]:
    cov = dict(cfg.get('coverage') or {})
    return {
        'cell_size_m': float(cov.get('cell_size_m', 1.0)),
        'yaw_bins': int(cov.get('yaw_bins', 8)),
        'neighbor_cell_radius': int(cov.get('neighbor_cell_radius', 1)),
    }
