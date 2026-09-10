"""Lightweight CPU image quality gate (no ML / NPU / segmentation)."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional

import cv2
import numpy as np

from xw_global_reloc.orb_utils import OrbFrame, extract_orb, make_orb


@dataclass
class ImageGateResult:
    ok: bool
    reason: str
    reasons: List[str] = field(default_factory=list)
    image_quality: Dict[str, Any] = field(default_factory=dict)
    orb: Optional[OrbFrame] = None
    reject_bucket: str = ''  # rejected_blur / rejected_dark / ...

    @property
    def code(self) -> str:
        return 'PASS' if self.ok else 'REJECTED_IMAGE'


def _as_gray(bgr_or_gray: np.ndarray) -> np.ndarray:
    if bgr_or_gray.ndim == 2:
        return bgr_or_gray
    if bgr_or_gray.ndim == 3 and bgr_or_gray.shape[2] >= 3:
        return cv2.cvtColor(bgr_or_gray, cv2.COLOR_BGR2GRAY)
    raise ValueError('unsupported_image_shape')


def evaluate_image_gate(
    bgr: Optional[np.ndarray],
    cfg: Dict[str, Any],
    *,
    orb: Optional[OrbFrame] = None,
) -> ImageGateResult:
    ig = dict(cfg.get('image_gate') or {})
    orb_cfg = dict(cfg.get('orb') or {})
    reasons: List[str] = []
    bucket = ''

    if bgr is None:
        q = {
            'sharpness': None,
            'brightness': None,
            'underexposure_ratio': None,
            'overexposure_ratio': None,
            'orb_features': 0,
            'occlusion_ratio': None,
            'reasons': ['empty_frame'],
        }
        return ImageGateResult(
            False, 'empty_frame', reasons=['empty_frame'], image_quality=q, reject_bucket='rejected_invalid_frame'
        )

    try:
        if not isinstance(bgr, np.ndarray) or bgr.size == 0:
            raise ValueError('empty')
        h, w = int(bgr.shape[0]), int(bgr.shape[1])
        if h < int(ig.get('expected_min_height', 120)) or w < int(ig.get('expected_min_width', 160)):
            reasons.append('frame_too_small')
            bucket = 'rejected_invalid_frame'
        gray = _as_gray(bgr)
    except Exception:  # noqa: BLE001
        q = {
            'sharpness': None,
            'brightness': None,
            'underexposure_ratio': None,
            'overexposure_ratio': None,
            'orb_features': 0,
            'occlusion_ratio': None,
            'reasons': ['decode_or_shape_error'],
        }
        return ImageGateResult(
            False,
            'decode_or_shape_error',
            reasons=['decode_or_shape_error'],
            image_quality=q,
            reject_bucket='rejected_invalid_frame',
        )

    # Sharpness: Laplacian variance
    lap = cv2.Laplacian(gray, cv2.CV_64F)
    sharpness = float(lap.var())
    brightness = float(np.mean(gray))
    under_t = int(ig.get('underexpose_thresh', 20))
    over_t = int(ig.get('overexpose_thresh', 235))
    underexposure_ratio = float(np.mean(gray < under_t))
    overexposure_ratio = float(np.mean(gray > over_t))

    if orb is None:
        extractor = make_orb(int(orb_cfg.get('n_features', 1000)))
        orb = extract_orb(bgr, extractor)
    orb_n = int(orb.n_features)

    min_sharp = float(ig.get('min_sharpness', 40.0))
    if sharpness < min_sharp:
        reasons.append('blurry')
        bucket = bucket or 'rejected_blur'

    bmin = float(ig.get('brightness_min', 25.0))
    bmax = float(ig.get('brightness_max', 230.0))
    max_under = float(ig.get('max_underexposure_ratio', 0.45))
    max_over = float(ig.get('max_overexposure_ratio', 0.35))
    if brightness < bmin or underexposure_ratio > max_under:
        reasons.append('too_dark')
        bucket = bucket or 'rejected_dark'
    if brightness > bmax or overexposure_ratio > max_over:
        reasons.append('overexposed')
        bucket = bucket or 'rejected_overexposed'

    min_orb = int(ig.get('min_orb_features', 80))
    if orb_n < min_orb:
        reasons.append('orb_low')
        bucket = bucket or 'rejected_low_features'

    q = {
        'sharpness': sharpness,
        'brightness': brightness,
        'underexposure_ratio': underexposure_ratio,
        'overexposure_ratio': overexposure_ratio,
        'orb_features': orb_n,
        'occlusion_ratio': None,  # reserved; no heavy occlusion model in A2
        'reasons': list(reasons),
    }
    if reasons:
        return ImageGateResult(
            False, reasons[0], reasons=reasons, image_quality=q, orb=orb, reject_bucket=bucket
        )
    return ImageGateResult(True, 'ok', reasons=[], image_quality=q, orb=orb, reject_bucket='')
