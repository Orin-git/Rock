"""Ring-buffer nearest-timestamp RGB↔Depth pairing."""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from typing import Deque, Optional, Tuple

from sensor_msgs.msg import Image


def stamp_sec(msg: Image) -> float:
    return float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9


@dataclass
class TimedImage:
    t: float
    msg: Image


@dataclass
class PairResult:
    rgb: Image
    depth: Image
    pair_dt_sec: float
    rgb_t: float
    depth_t: float


class ImageRingBuffer:
    def __init__(self, maxlen: int = 30) -> None:
        self._buf: Deque[TimedImage] = deque(maxlen=int(maxlen))

    def push(self, msg: Image) -> None:
        self._buf.append(TimedImage(stamp_sec(msg), msg))

    def __len__(self) -> int:
        return len(self._buf)

    def nearest(self, t: float) -> Optional[TimedImage]:
        if not self._buf:
            return None
        best = None
        best_dt = 1e9
        for item in self._buf:
            dt = abs(item.t - t)
            if dt < best_dt:
                best_dt = dt
                best = item
        return best


def pair_nearest(
    rgb_buf: ImageRingBuffer,
    depth_buf: ImageRingBuffer,
    *,
    prefer: str = 'rgb',
) -> Optional[PairResult]:
    """Pair latest prefer-stream frame with nearest other-stream frame."""
    if prefer == 'rgb':
        if not rgb_buf._buf:
            return None
        anchor = rgb_buf._buf[-1]
        other = depth_buf.nearest(anchor.t)
        if other is None:
            return None
        return PairResult(
            rgb=anchor.msg,
            depth=other.msg,
            pair_dt_sec=abs(anchor.t - other.t),
            rgb_t=anchor.t,
            depth_t=other.t,
        )
    if not depth_buf._buf:
        return None
    anchor = depth_buf._buf[-1]
    other = rgb_buf.nearest(anchor.t)
    if other is None:
        return None
    return PairResult(
        rgb=other.msg,
        depth=anchor.msg,
        pair_dt_sec=abs(anchor.t - other.t),
        rgb_t=other.t,
        depth_t=anchor.t,
    )
