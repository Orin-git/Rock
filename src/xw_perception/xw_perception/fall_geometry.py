#!/usr/bin/env python3
"""Gen1-style Stage1 fall geometry on COCO-17 keypoints (no hobot classifier)."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional, Tuple

import numpy as np


@dataclass
class FallGeometryParams:
    # Defaults tuned for robot forward cam (lying poses often lower conf / odd aspect).
    kp_conf_min: float = 0.35
    kp_min_visible: int = 4
    bbox_aspect_min: float = 0.55
    bbox_aspect_min_relaxed: float = 0.4
    torso_angle_min_deg: float = 30.0
    torso_compression_max: float = 0.40
    torso_inversion_margin_ratio: float = 0.0
    # Wide / flat bbox alone (side-lying in frame) — robot-height FOV often misses torso angle.
    flat_aspect_min: float = 1.15


def passes_fall_geometry(
    keypoints: np.ndarray,
    xmin: float,
    ymin: float,
    xmax: float,
    ymax: float,
    params: Optional[FallGeometryParams] = None,
) -> Tuple[bool, str]:
    """Return (ok, reason). keypoints: (17, 3) x,y,conf in image pixels."""
    p = params or FallGeometryParams()
    if keypoints is None or keypoints.shape[0] < 17:
        return False, 'no_kps'
    confs = keypoints[:, 2]
    visible = int(np.sum(confs >= p.kp_conf_min))
    if visible < p.kp_min_visible:
        return False, f'visible={visible}'

    width = max(1.0, float(xmax - xmin))
    height = max(1.0, float(ymax - ymin))
    aspect = width / height

    # Flat bbox: person much wider than tall → strong lying cue even if torso kps weak.
    if aspect >= p.flat_aspect_min and visible >= p.kp_min_visible:
        return True, f'flat aspect={aspect:.2f}'

    required = (5, 6, 11, 12)  # L/R shoulder, L/R hip
    torso_ok = not any(float(confs[i]) < p.kp_conf_min for i in required)
    if not torso_ok:
        # Allow shoulder-or-hip pair soft path when flat-ish
        soft = aspect >= p.bbox_aspect_min_relaxed and visible >= p.kp_min_visible
        if soft:
            return True, f'soft_flat aspect={aspect:.2f} vis={visible}'
        return False, 'missing_torso_kps'

    sx = (float(keypoints[5, 0]) + float(keypoints[6, 0])) / 2.0
    sy = (float(keypoints[5, 1]) + float(keypoints[6, 1])) / 2.0
    hx = (float(keypoints[11, 0]) + float(keypoints[12, 0])) / 2.0
    hy = (float(keypoints[11, 1]) + float(keypoints[12, 1])) / 2.0

    torso_dx = abs(sx - hx)
    torso_dy = abs(sy - hy)
    body_scale = max(height, width, 1.0)

    theta = math.degrees(math.atan2(torso_dx, torso_dy + 1e-6))
    sideways = aspect >= p.bbox_aspect_min and theta >= p.torso_angle_min_deg

    torso_dist = math.hypot(torso_dx, torso_dy)
    compression = torso_dist / body_scale
    backward = aspect >= p.bbox_aspect_min_relaxed and compression <= p.torso_compression_max

    inversion_dy = sy - hy
    inversion_margin = body_scale * p.torso_inversion_margin_ratio
    forward = aspect >= p.bbox_aspect_min_relaxed and inversion_dy >= inversion_margin

    if sideways or backward or forward:
        which = 'side' if sideways else ('back' if backward else 'fwd')
        return True, which
    return (
        False,
        f'aspect={aspect:.2f} theta={theta:.1f} comp={compression:.3f} inv={inversion_dy:.0f}',
    )
