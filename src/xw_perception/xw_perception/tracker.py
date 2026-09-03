"""Kalman + lock FSM for person tracks (L2 association, low CPU)."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List, Optional, Sequence, Tuple

import numpy as np


@dataclass
class Detection:
    xmin: float
    ymin: float
    xmax: float
    ymax: float
    bearing: float  # -1 .. +1
    distance: float
    confidence: float
    area: float = 0.0
    width_ratio: float = 0.0  # bbox_w / image_w (gen1-style range cue)

    @property
    def cx(self) -> float:
        return 0.5 * (self.xmin + self.xmax)

    @property
    def cy(self) -> float:
        return 0.5 * (self.ymin + self.ymax)

    @property
    def w(self) -> float:
        return max(1.0, self.xmax - self.xmin)

    @property
    def h(self) -> float:
        return max(1.0, self.ymax - self.ymin)

    def as_bbox(self) -> Tuple[float, float, float, float]:
        return (self.xmin, self.ymin, self.xmax, self.ymax)


def bbox_iou(a: Sequence[float], b: Sequence[float]) -> float:
    ax0, ay0, ax1, ay1 = a
    bx0, by0, bx1, by1 = b
    ix0, iy0 = max(ax0, bx0), max(ay0, by0)
    ix1, iy1 = min(ax1, bx1), min(ay1, by1)
    inter = max(0.0, ix1 - ix0) * max(0.0, iy1 - iy0)
    area_a = max(1.0, (ax1 - ax0) * (ay1 - ay0))
    area_b = max(1.0, (bx1 - bx0) * (by1 - by0))
    return inter / (area_a + area_b - inter)


class KalmanBBox:
    """Constant-velocity Kalman on [cx, cy, w, h, vx, vy]."""

    def __init__(self, det: Detection, process_var: float = 8.0, meas_var: float = 25.0) -> None:
        self.x = np.array(
            [det.cx, det.cy, det.w, det.h, 0.0, 0.0], dtype=np.float64
        )
        self.P = np.eye(6, dtype=np.float64) * 50.0
        self.Q_base = process_var
        self.R = np.eye(4, dtype=np.float64) * meas_var
        self.H = np.zeros((4, 6), dtype=np.float64)
        self.H[0, 0] = self.H[1, 1] = self.H[2, 2] = self.H[3, 3] = 1.0

    def predict(self, dt: float = 1.0) -> None:
        dt = max(1e-3, float(dt))
        F = np.eye(6, dtype=np.float64)
        F[0, 4] = dt
        F[1, 5] = dt
        self.x = F @ self.x
        Q = np.eye(6, dtype=np.float64) * self.Q_base
        Q[4, 4] = Q[5, 5] = self.Q_base * 2.0
        self.P = F @ self.P @ F.T + Q

    def update(self, det: Detection) -> None:
        z = np.array([det.cx, det.cy, det.w, det.h], dtype=np.float64)
        y = z - self.H @ self.x
        S = self.H @ self.P @ self.H.T + self.R
        K = self.P @ self.H.T @ np.linalg.inv(S)
        self.x = self.x + K @ y
        I = np.eye(6)
        self.P = (I - K @ self.H) @ self.P

    def predicted_bbox(self) -> Tuple[float, float, float, float]:
        cx, cy, w, h = float(self.x[0]), float(self.x[1]), float(self.x[2]), float(self.x[3])
        w = max(1.0, w)
        h = max(1.0, h)
        return (cx - 0.5 * w, cy - 0.5 * h, cx + 0.5 * w, cy + 0.5 * h)

    def mahalanobis(self, det: Detection) -> float:
        z = np.array([det.cx, det.cy, det.w, det.h], dtype=np.float64)
        y = z - self.H @ self.x
        S = self.H @ self.P @ self.H.T + self.R
        try:
            return float(y.T @ np.linalg.inv(S) @ y)
        except np.linalg.LinAlgError:
            return 1e9


class PersonLockTracker:
    """Lock once on follow start; associate with IoU + Mahalanobis; coast on occlusion."""

    def __init__(
        self,
        *,
        lock_strategy: str = 'center',
        coast_frames: int = 12,
        iou_thresh: float = 0.15,
        maha_thresh: float = 40.0,
        center_frac: float = 0.35,
        img_w: float = 640.0,
        img_h: float = 480.0,
        min_lock_conf: float = 0.42,
        min_lock_area_frac: float = 0.035,
        min_lock_width_ratio: float = 0.11,
    ) -> None:
        self.lock_strategy = lock_strategy
        self.coast_frames = max(1, int(coast_frames))
        self.iou_thresh = float(iou_thresh)
        self.maha_thresh = float(maha_thresh)
        self.center_frac = float(center_frac)
        self.img_w = float(img_w)
        self.img_h = float(img_h)
        self.min_lock_conf = float(min_lock_conf)
        self.min_lock_area_frac = float(min_lock_area_frac)
        self.min_lock_width_ratio = float(min_lock_width_ratio)

        self._locked = False
        self._target_id = 0
        self._next_id = 1
        self._kf: Optional[KalmanBBox] = None
        self._miss = 0
        self._last_det: Optional[Detection] = None
        self._lost = False

    @property
    def last_det(self) -> Optional[Detection]:
        return self._last_det

    @property
    def locked(self) -> bool:
        return self._locked and not self._lost

    @property
    def target_id(self) -> int:
        return self._target_id

    @property
    def lost(self) -> bool:
        return self._lost

    @property
    def coasting(self) -> bool:
        return self._locked and not self._lost and self._miss > 0

    def reset(self) -> None:
        self._locked = False
        self._target_id = 0
        self._kf = None
        self._miss = 0
        self._last_det = None
        self._lost = False

    def set_image_size(self, w: float, h: float) -> None:
        if w > 0:
            self.img_w = float(w)
        if h > 0:
            self.img_h = float(h)

    def _eligible(self, dets: List[Detection]) -> List[Detection]:
        """Drop weak / tiny / edge slivers so we lock a real person, not clutter."""
        cx0 = 0.5 * self.img_w
        img_area = max(1.0, self.img_w * self.img_h)
        min_area = max(0.01, self.min_lock_area_frac) * img_area
        min_wr = max(0.05, self.min_lock_width_ratio)
        out: List[Detection] = []
        for d in dets:
            if d.confidence < self.min_lock_conf:
                continue
            if d.area < min_area and d.width_ratio < min_wr:
                continue
            if d.width_ratio < 0.07:
                continue
            # Reject flat chair-like blobs (wide & short) unless very confident.
            aspect = d.h / max(1.0, d.w)
            if aspect < 0.55 and d.confidence < 0.55:
                continue
            if d.w < 0.08 * self.img_w and abs(d.cx - cx0) > 0.32 * self.img_w:
                continue
            out.append(d)
        # Soft fallback: still require conf, but allow slightly smaller area.
        if out:
            return out
        soft = []
        for d in dets:
            if d.confidence < self.min_lock_conf:
                continue
            if d.area < 0.5 * min_area and d.width_ratio < 0.5 * min_wr:
                continue
            soft.append(d)
        return soft

    def _select_lock(self, dets: List[Detection]) -> Optional[Detection]:
        dets = self._eligible(dets)
        if not dets:
            return None
        strategy = (self.lock_strategy or 'center').lower()
        if strategy == 'largest':
            return max(dets, key=lambda d: d.area)
        if strategy == 'nearest':
            return min(
                dets,
                key=lambda d: d.distance if d.distance > 0 else 1e6,
            )
        # center: person standing in front — center + conf + body size
        cx = 0.5 * self.img_w
        img_area = max(1.0, self.img_w * self.img_h)

        def score(d: Detection) -> Tuple[float, float, float]:
            center_n = abs(d.cx - cx) / max(1.0, 0.5 * self.img_w)
            # Prefer confident, reasonably large bodies near image center.
            quality = -float(d.confidence) - 0.45 * min(1.0, d.area / (0.10 * img_area))
            dist = d.distance if d.distance > 0 else 1e6
            return (center_n + 0.20 * quality, dist, -d.area)

        return min(dets, key=score)

    def lock_now(self, dets: List[Detection]) -> bool:
        """Rising-edge lock: pick once and stick."""
        self.reset()
        chosen = self._select_lock(dets)
        if chosen is None:
            return False
        self._target_id = self._next_id
        self._next_id += 1
        self._kf = KalmanBBox(chosen)
        self._last_det = chosen
        self._locked = True
        self._miss = 0
        self._lost = False
        return True

    def update(
        self, dets: List[Detection], dt: float = 1.0
    ) -> Tuple[Optional[Detection], bool, bool]:
        """Associate detections to locked target.

        Returns (target_det_or_predicted, is_coasting, is_lost).
        """
        if not self._locked or self._kf is None:
            return None, False, True

        self._kf.predict(dt)
        best_i = -1
        best_score = -1.0
        pred_bbox = self._kf.predicted_bbox()
        pred_cx = 0.5 * (pred_bbox[0] + pred_bbox[2])
        pred_cy = 0.5 * (pred_bbox[1] + pred_bbox[3])
        center_lim = max(40.0, self.center_frac * self.img_w)
        assoc_conf = max(0.28, 0.85 * self.min_lock_conf)

        for i, d in enumerate(dets):
            if d.confidence < assoc_conf:
                continue
            iou = bbox_iou(pred_bbox, d.as_bbox())
            maha = self._kf.mahalanobis(d)
            cdist = math.hypot(d.cx - pred_cx, d.cy - pred_cy)
            # Accept if IoU/Maha OK, OR center is still close (handles spin / blur)
            if iou < self.iou_thresh and maha > self.maha_thresh and cdist > center_lim:
                continue
            # Prefer high IoU; then near center; break ties with low Mahalanobis
            score = iou * 10.0 - 0.01 * maha - 0.002 * cdist + 0.5 * float(d.confidence)
            if score > best_score:
                best_score = score
                best_i = i

        # Bearing continuity: if prediction failed, pick det closest in image-x
        # among confident ones (lateral walk).
        if best_i < 0 and self._last_det is not None and dets:
            prev_cx = self._last_det.cx
            cand = [d for d in dets if d.confidence >= self.min_lock_conf]
            if cand:
                d0 = min(cand, key=lambda d: abs(d.cx - prev_cx))
                if abs(d0.cx - prev_cx) < 0.45 * self.img_w:
                    best_i = dets.index(d0)
        # Lone high-conf person → keep lock (side-step out of pred gate).
        if best_i < 0 and len(dets) == 1 and dets[0].confidence >= self.min_lock_conf:
            # Still require non-tiny body so we don't stick to chair slivers.
            d0 = dets[0]
            if d0.area >= 0.02 * self.img_w * self.img_h or d0.width_ratio >= 0.10:
                best_i = 0

        if best_i >= 0:
            det = dets[best_i]
            self._kf.update(det)
            self._last_det = det
            self._miss = 0
            self._lost = False
            return det, False, False

        self._miss += 1
        if self._miss > self.coast_frames:
            self._lost = True
            return None, False, True

        # Coast with predicted bbox / last measurement fields
        pred = pred_bbox
        last = self._last_det
        coast = Detection(
            xmin=pred[0],
            ymin=pred[1],
            xmax=pred[2],
            ymax=pred[3],
            bearing=last.bearing if last else 0.0,
            distance=last.distance if last else 0.0,
            confidence=(last.confidence * 0.5) if last else 0.0,
            area=max(1.0, (pred[2] - pred[0]) * (pred[3] - pred[1])),
            width_ratio=last.width_ratio if last else 0.0,
        )
        # Keep last good bearing during coast — predicted cx drifts while spinning
        if last is not None:
            coast.bearing = last.bearing
            coast.distance = last.distance
            coast.width_ratio = last.width_ratio
        elif self.img_w > 1:
            coast.bearing = (coast.cx - 0.5 * self.img_w) / (0.5 * self.img_w)
        return coast, True, False
