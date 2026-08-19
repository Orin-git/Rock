"""Coded lidar-reflector charger detector (no ROS).

Ported from ros2_ws/doc/auto_charger.py ReflactionDetector. Matching is the
sliding-window chord-length barcode; geometric windows belong in base_link
(the caller), because gen2 lidar_link is yaw=π relative to base_link.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Iterable, List, Optional, Sequence, Tuple


def wrap_angle(angle: float) -> float:
    a = (angle + math.pi) % (2.0 * math.pi)
    if a <= 0.0:
        a += 2.0 * math.pi
    return a - math.pi


def hypot_chord(theta1: float, rho1: float, theta2: float, rho2: float) -> float:
    return math.sqrt(
        rho1 * rho1 + rho2 * rho2 - 2.0 * rho1 * rho2 * math.cos(theta1 - theta2)
    )


@dataclass
class ReflactionSegment:
    theta1: float
    rho1: float
    theta2: float
    rho2: float

    @property
    def width(self) -> float:
        return hypot_chord(self.theta1, self.rho1, self.theta2, self.rho2)

    @property
    def cx(self) -> float:
        x1 = self.rho1 * math.cos(self.theta1)
        x2 = self.rho2 * math.cos(self.theta2)
        return 0.5 * (x1 + x2)

    @property
    def cy(self) -> float:
        y1 = self.rho1 * math.sin(self.theta1)
        y2 = self.rho2 * math.sin(self.theta2)
        return 0.5 * (y1 + y2)


@dataclass
class LaserChargerDetection:
    x: float
    y: float
    yaw: float
    range: float
    matched_segments: int
    frame_id: str = ''


@dataclass
class DetectorParams:
    intensity_threshold: float = 200.0
    code: Tuple[float, ...] = (0.06, 0.025, 0.08, 0.025, 0.06)
    code_tol: float = 0.02
    min_points_per_segment: int = 2


class ReflactionDetector:
    def __init__(self, params: Optional[DetectorParams] = None) -> None:
        self.params = params or DetectorParams()

    def find_segments(
        self,
        ranges: Sequence[float],
        intensities: Sequence[float],
        angle_min: float,
        angle_increment: float,
    ) -> List[ReflactionSegment]:
        p = self.params
        out: List[ReflactionSegment] = []
        cur: List[int] = []
        n = min(len(ranges), len(intensities) if intensities else 0)
        # Some drivers omit intensities; treat as empty → no segments.
        if n <= 0:
            return out
        for i in range(n):
            rng = float(ranges[i])
            inten = float(intensities[i])
            if inten > p.intensity_threshold and math.isfinite(rng) and rng > 0.01:
                cur.append(i)
                continue
            if len(cur) >= p.min_points_per_segment:
                out.append(self._seg_from_indices(cur, ranges, angle_min, angle_increment))
            cur = []
        if len(cur) >= p.min_points_per_segment:
            out.append(self._seg_from_indices(cur, ranges, angle_min, angle_increment))
        return out

    @staticmethod
    def _seg_from_indices(
        idxs: Sequence[int],
        ranges: Sequence[float],
        angle_min: float,
        angle_increment: float,
    ) -> ReflactionSegment:
        i0, i1 = idxs[0], idxs[-1]
        return ReflactionSegment(
            theta1=angle_min + angle_increment * i0,
            rho1=float(ranges[i0]),
            theta2=angle_min + angle_increment * i1,
            rho2=float(ranges[i1]),
        )

    def detect(
        self,
        ranges: Sequence[float],
        intensities: Sequence[float],
        angle_min: float,
        angle_increment: float,
        frame_id: str = '',
    ) -> Optional[LaserChargerDetection]:
        segs = self.find_segments(ranges, intensities, angle_min, angle_increment)
        code = self.params.code
        need = (len(code) + 1) // 2
        if len(segs) < need:
            return None
        tol = self.params.code_tol
        for start in range(0, len(segs) - need + 1):
            ci = 0
            ok = True
            for k in range(need):
                if abs(segs[start + k].width - code[ci]) > tol:
                    ok = False
                    break
                ci += 1
                if k + 1 < need:
                    gap = hypot_chord(
                        segs[start + k].theta2,
                        segs[start + k].rho2,
                        segs[start + k + 1].theta1,
                        segs[start + k + 1].rho1,
                    )
                    if abs(gap - code[ci]) > tol:
                        ok = False
                        break
                    ci += 1
            if not ok:
                continue
            first = segs[start]
            last = segs[start + need - 1]
            x1 = first.rho1 * math.cos(first.theta1)
            y1 = first.rho1 * math.sin(first.theta1)
            x2 = last.rho2 * math.cos(last.theta2)
            y2 = last.rho2 * math.sin(last.theta2)
            x = 0.5 * (x1 + x2)
            y = 0.5 * (y1 + y2)
            yaw = math.atan2(y2 - y1, x2 - x1) + math.pi / 2.0
            return LaserChargerDetection(
                x=x,
                y=y,
                yaw=wrap_angle(yaw),
                range=math.hypot(x, y),
                matched_segments=need,
                frame_id=frame_id,
            )
        return None


class DetectionTracker:
    """Require N consistent frames before trusting a detection."""

    def __init__(self, confirm_frames: int = 3, confirm_std_m: float = 0.03) -> None:
        self.confirm_frames = max(1, int(confirm_frames))
        self.confirm_std_m = float(confirm_std_m)
        self._buf: List[LaserChargerDetection] = []

    def reset(self) -> None:
        self._buf.clear()

    def update(self, det: Optional[LaserChargerDetection]) -> Optional[LaserChargerDetection]:
        if det is None:
            self._buf.clear()
            return None
        self._buf.append(det)
        if len(self._buf) > self.confirm_frames:
            self._buf.pop(0)
        if len(self._buf) < self.confirm_frames:
            return None
        mx = sum(d.x for d in self._buf) / len(self._buf)
        my = sum(d.y for d in self._buf) / len(self._buf)
        var = sum((d.x - mx) ** 2 + (d.y - my) ** 2 for d in self._buf) / len(self._buf)
        if math.sqrt(var) > self.confirm_std_m:
            return None
        last = self._buf[-1]
        return LaserChargerDetection(
            x=mx,
            y=my,
            yaw=last.yaw,
            range=math.hypot(mx, my),
            matched_segments=last.matched_segments,
            frame_id=last.frame_id,
        )


def in_base_window(
    x: float,
    y: float,
    min_x: float = 0.25,
    max_x: float = 1.5,
    min_y: float = -0.8,
    max_y: float = 0.8,
) -> bool:
    return min_x <= x <= max_x and min_y <= y <= max_y


def transform_xy_yaw(
    x: float,
    y: float,
    yaw: float,
    tx: float,
    ty: float,
    tyaw: float,
) -> Tuple[float, float, float]:
    c, s = math.cos(tyaw), math.sin(tyaw)
    return tx + c * x - s * y, ty + s * x + c * y, wrap_angle(tyaw + yaw)


def make_synthetic_scan(
    *,
    charger_x: float = 0.80,
    charger_y: float = 0.0,
    charger_yaw: float = 0.0,
    code: Sequence[float] = (0.06, 0.025, 0.08, 0.025, 0.06),
    intensity: float = 250.0,
    background: float = 10.0,
    angle_min: float = -math.pi,
    angle_max: float = math.pi,
    increment: float = 0.004,
    wall_range: float = 8.0,
) -> Tuple[List[float], List[float], float, float]:
    """Barcode on a plane facing the sensor, centered at (charger_x, charger_y).

    charger_yaw is the dock facing (into free space / toward the robot when
    head-on). The strip is perpendicular to that yaw.
    """
    n = int(round((angle_max - angle_min) / increment)) + 1
    ranges = [wall_range] * n
    intensities = [background] * n
    # Along-strip unit (perpendicular to facing)
    sx, sy = -math.sin(charger_yaw), math.cos(charger_yaw)
    nx, ny = math.cos(charger_yaw), math.sin(charger_yaw)
    # Build intervals along strip axis: + is left of facing
    cursor = -0.5 * sum(code)
    intervals: List[Tuple[float, float, bool]] = []
    for i, length in enumerate(code):
        reflective = (i % 2) == 0
        intervals.append((cursor, cursor + length, reflective))
        cursor += length

    def hit_range_for_angle(th: float) -> Optional[float]:
        # Ray p = t*(cos th, sin th). Intersect plane through charger with normal n.
        cth, sth = math.cos(th), math.sin(th)
        denom = cth * nx + sth * ny
        if abs(denom) < 1e-6:
            return None
        t = (charger_x * nx + charger_y * ny) / denom
        if t < 0.05:
            return None
        hx, hy = t * cth, t * sth
        along = (hx - charger_x) * sx + (hy - charger_y) * sy
        return t, along  # type: ignore[return-value]

    for i in range(n):
        th = angle_min + i * increment
        hit = hit_range_for_angle(th)
        if hit is None:
            continue
        t, along = hit
        for a, b, refl in intervals:
            if a <= along <= b:
                ranges[i] = float(t)
                intensities[i] = intensity if refl else background
                break
    return ranges, intensities, angle_min, increment


def intensity_histogram(intensities: Iterable[float], bins: int = 10) -> List[Tuple[float, int]]:
    vals = [float(v) for v in intensities if math.isfinite(v)]
    if not vals:
        return []
    lo, hi = min(vals), max(vals)
    if hi <= lo:
        return [(lo, len(vals))]
    width = (hi - lo) / bins
    counts = [0] * bins
    for v in vals:
        k = min(bins - 1, int((v - lo) / width))
        counts[k] += 1
    return [(lo + (i + 0.5) * width, counts[i]) for i in range(bins)]
