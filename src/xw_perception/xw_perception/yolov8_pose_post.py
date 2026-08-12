#!/usr/bin/env python3
"""YOLOv8-pose letterbox + Rockchip DFL decode (rknn_model_zoo compatible)."""

from __future__ import annotations

from dataclasses import dataclass
from typing import List, Sequence, Tuple

import numpy as np

CLASSES = ['person']
NMS_THRESH = 0.4
OBJECT_THRESH = 0.5


@dataclass
class PoseBox:
    score: float
    xmin: float
    ymin: float
    xmax: float
    ymax: float
    # shape (17, 3): x, y, conf in letterbox pixel space
    keypoints: np.ndarray


def letterbox_resize(
    image: np.ndarray, size: Tuple[int, int] = (640, 640), bg_color: int = 56
) -> Tuple[np.ndarray, float, int, int]:
    target_width, target_height = size
    image_height, image_width = image.shape[:2]
    aspect_ratio = min(target_width / image_width, target_height / image_height)
    new_width = int(image_width * aspect_ratio)
    new_height = int(image_height * aspect_ratio)
    # Lazy import cv2 so node can start without it for dry tests.
    import cv2

    image = cv2.resize(image, (new_width, new_height), interpolation=cv2.INTER_AREA)
    result = np.ones((target_height, target_width, 3), dtype=np.uint8) * bg_color
    offset_x = (target_width - new_width) // 2
    offset_y = (target_height - new_height) // 2
    result[offset_y : offset_y + new_height, offset_x : offset_x + new_width] = image
    return result, aspect_ratio, offset_x, offset_y


def _sigmoid(x: np.ndarray) -> np.ndarray:
    return 1.0 / (1.0 + np.exp(-x))


def _softmax(x: np.ndarray, axis: int = -1) -> np.ndarray:
    exp_x = np.exp(x - np.max(x, axis=axis, keepdims=True))
    return exp_x / np.sum(exp_x, axis=axis, keepdims=True)


def _iou(a: PoseBox, b: PoseBox) -> float:
    xmin = max(a.xmin, b.xmin)
    ymin = max(a.ymin, b.ymin)
    xmax = min(a.xmax, b.xmax)
    ymax = min(a.ymax, b.ymax)
    iw = max(0.0, xmax - xmin)
    ih = max(0.0, ymax - ymin)
    inter = iw * ih
    area1 = (a.xmax - a.xmin) * (a.ymax - a.ymin)
    area2 = (b.xmax - b.xmin) * (b.ymax - b.ymin)
    denom = area1 + area2 - inter
    return inter / denom if denom > 0 else 0.0


def _nms(boxes: List[PoseBox], thresh: float = NMS_THRESH) -> List[PoseBox]:
    ordered = sorted(boxes, key=lambda b: b.score, reverse=True)
    keep: List[PoseBox] = []
    suppressed = [False] * len(ordered)
    for i, bi in enumerate(ordered):
        if suppressed[i]:
            continue
        keep.append(bi)
        for j in range(i + 1, len(ordered)):
            if suppressed[j]:
                continue
            if _iou(bi, ordered[j]) > thresh:
                suppressed[j] = True
    return keep


def _process_scale(
    out: np.ndarray,
    keypoints: np.ndarray,
    index: int,
    model_w: int,
    model_h: int,
    stride: int,
    object_thresh: float,
) -> List[PoseBox]:
    """Decode one YOLO scale. out: [1, 65, H*W], keypoints: [..., N]."""
    xywh = out[:, :64, :]
    conf = _sigmoid(out[:, 64:, :])
    boxes: List[PoseBox] = []
    for h in range(model_h):
        for w in range(model_w):
            for c in range(len(CLASSES)):
                score = float(conf[0, c, (h * model_w) + w])
                if score <= object_thresh:
                    continue
                xywh_ = xywh[0, :, (h * model_w) + w].reshape(1, 4, 16, 1)
                data = np.arange(16, dtype=np.float32).reshape(1, 1, 16, 1)
                xywh_ = _softmax(xywh_, 2)
                xywh_ = np.multiply(data, xywh_)
                xywh_ = np.sum(xywh_, axis=2, keepdims=True).reshape(-1)

                xywh_temp = xywh_.copy()
                xywh_temp[0] = (w + 0.5) - xywh_[0]
                xywh_temp[1] = (h + 0.5) - xywh_[1]
                xywh_temp[2] = (w + 0.5) + xywh_[2]
                xywh_temp[3] = (h + 0.5) + xywh_[3]

                cx = (xywh_temp[0] + xywh_temp[2]) / 2.0
                cy = (xywh_temp[1] + xywh_temp[3]) / 2.0
                bw = xywh_temp[2] - xywh_temp[0]
                bh = xywh_temp[3] - xywh_temp[1]
                cx *= stride
                cy *= stride
                bw *= stride
                bh *= stride

                xmin = cx - bw / 2.0
                ymin = cy - bh / 2.0
                xmax = cx + bw / 2.0
                ymax = cy + bh / 2.0

                keypoint = keypoints[..., (h * model_w) + w + index].copy()
                keypoint = np.array(keypoint, dtype=np.float32).reshape(-1)
                # Rockchip demo floored x,y; keep float for geometry.
                boxes.append(
                    PoseBox(
                        score=score,
                        xmin=float(xmin),
                        ymin=float(ymin),
                        xmax=float(xmax),
                        ymax=float(ymax),
                        keypoints=keypoint.reshape(-1, 3),
                    )
                )
    return boxes


def decode_rknn_outputs(
    results: Sequence[np.ndarray],
    object_thresh: float = OBJECT_THRESH,
) -> List[PoseBox]:
    """Decode Rockchip yolov8n-pose RKNN outputs (4 tensors)."""
    if len(results) < 4:
        raise ValueError(f'expected ≥4 RKNN outputs, got {len(results)}')
    keypoints = results[3]
    outputs: List[PoseBox] = []
    for x in results[:3]:
        # x shape typically [1, 65, H, W]
        if x.ndim == 4:
            h, w = int(x.shape[2]), int(x.shape[3])
            feature = x.reshape(1, 65, -1)
        elif x.ndim == 3:
            # [1, 65, H*W] — infer H=W=sqrt
            n = int(x.shape[2])
            side = int(round(n**0.5))
            h = w = side
            feature = x
        else:
            continue
        if h == 20:
            stride, index = 32, 20 * 4 * 20 * 4 + 20 * 2 * 20 * 2
        elif h == 40:
            stride, index = 16, 20 * 4 * 20 * 4
        elif h == 80:
            stride, index = 8, 0
        else:
            continue
        outputs.extend(
            _process_scale(feature, keypoints, index, w, h, stride, object_thresh)
        )
    return _nms(outputs)


def map_to_original(
    box: PoseBox,
    aspect_ratio: float,
    offset_x: int,
    offset_y: int,
) -> PoseBox:
    """Undo letterbox to original image coordinates."""
    kps = box.keypoints.copy()
    kps[:, 0] = (kps[:, 0] - offset_x) / aspect_ratio
    kps[:, 1] = (kps[:, 1] - offset_y) / aspect_ratio
    return PoseBox(
        score=box.score,
        xmin=(box.xmin - offset_x) / aspect_ratio,
        ymin=(box.ymin - offset_y) / aspect_ratio,
        xmax=(box.xmax - offset_x) / aspect_ratio,
        ymax=(box.ymax - offset_y) / aspect_ratio,
        keypoints=kps,
    )
