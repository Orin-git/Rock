#!/usr/bin/env python3
"""Track A sensor root-cause probes (few frames, no continuous recording)."""

from __future__ import annotations

import hashlib
import json
import time
from pathlib import Path

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Bool


_SENSOR = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=5,
)
_LATCH = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)


def analyze_depth(msg: Image, label: str, out_dir: Path) -> dict:
    bridge = CvBridge()
    depth = bridge.imgmsg_to_cv2(msg, 'passthrough')
    h, w = int(msg.height), int(msg.width)
    step = int(msg.step)
    data_size = len(msg.data)
    expect = step * h
    row16 = w * 2
    valid = (depth > 0) if depth.dtype == np.uint16 else np.isfinite(depth)
    ys, xs = np.where(valid)
    bbox = None
    if len(xs):
        bbox = {
            'xmin': int(xs.min()),
            'xmax': int(xs.max()),
            'ymin': int(ys.min()),
            'ymax': int(ys.max()),
        }
    tile = 32
    tiles = {}
    for y0 in range(0, h, tile):
        for x0 in range(0, w, tile):
            t = valid[y0 : y0 + tile, x0 : x0 + tile]
            tiles[f'{y0}_{x0}'] = float(np.count_nonzero(t)) / float(max(t.size, 1))
    # left vs right thirds
    regions = {
        'left': float(np.count_nonzero(valid[:, : w // 3])) / float(max(valid[:, : w // 3].size, 1)),
        'center': float(np.count_nonzero(valid[:, w // 3 : 2 * w // 3]))
        / float(max(valid[:, w // 3 : 2 * w // 3].size, 1)),
        'right': float(np.count_nonzero(valid[:, 2 * w // 3 :]))
        / float(max(valid[:, 2 * w // 3 :].size, 1)),
    }
    mask = (valid.astype(np.uint8) * 255)
    mask_path = out_dir / f'{label}_depth_valid_mask.png'
    cv2.imwrite(str(mask_path), mask)
    crc = hashlib.sha1(msg.data).hexdigest()[:16]
    # systematic left concentration?
    systematic = regions['left'] > 0.15 and regions['center'] < 0.05 and regions['right'] < 0.05
    return {
        'label': label,
        'encoding': msg.encoding,
        'frame_id': msg.header.frame_id,
        'width': w,
        'height': h,
        'step': step,
        'data_size': data_size,
        'data_size_eq_step_height': data_size == expect,
        'theory_row_bytes_16uc1': row16,
        'step_matches_16uc1_row': step == row16,
        'valid_ratio': float(np.count_nonzero(valid)) / float(max(valid.size, 1)),
        'valid_bbox': bbox,
        'region_valid_ratio': regions,
        'systematic_spatial_truncation': systematic,
        'mask_png': str(mask_path),
        'sha1_16': crc,
        'np_shape': list(depth.shape),
        'dtype': str(depth.dtype),
        'tile32_nonzero_count': sum(1 for v in tiles.values() if v > 0.01),
        'tile32_total': len(tiles),
    }


def main() -> None:
    out = Path('/ros2_ws/bench/phase2a_poc_v1_2026-09-07/track_a_sensor')
    out.mkdir(parents=True, exist_ok=True)
    rclpy.init()
    n = Node('track_a_sensor_probe')
    state = {'vendor': None, 'public': None, 'vinfo': None, 'pinfo': None, 'rgb': None, 'rinfo': None}
    n.create_subscription(
        Image, '/ascamera_hp60c/camera_publisher/depth0/image_raw',
        lambda m: state.update(vendor=m), _SENSOR,
    )
    n.create_subscription(
        Image, '/camera/front_up/depth/image_raw',
        lambda m: state.update(public=m), _SENSOR,
    )
    n.create_subscription(
        CameraInfo, '/ascamera_hp60c/camera_publisher/depth0/camera_info',
        lambda m: state.update(vinfo=m), _SENSOR,
    )
    n.create_subscription(
        CameraInfo, '/camera/front_up/depth/camera_info',
        lambda m: state.update(pinfo=m), _SENSOR,
    )
    n.create_subscription(
        Image, '/camera/front_up/color/image_raw',
        lambda m: state.update(rgb=m), _SENSOR,
    )
    n.create_subscription(
        CameraInfo, '/camera/front_up/color/camera_info',
        lambda m: state.update(rinfo=m), _SENSOR,
    )
    req = n.create_publisher(Bool, '/xw/reloc/rgb_request', _LATCH)
    req.publish(Bool(data=True))
    t0 = time.time()
    while time.time() - t0 < 12:
        rclpy.spin_once(n, timeout_sec=0.05)
        if state['vendor'] is not None and state['public'] is not None and state['rgb'] is not None:
            # wait a couple more spins for infos
            if state['vinfo'] is not None and state['pinfo'] is not None and state['rinfo'] is not None:
                break
    req.publish(Bool(data=False))

    report = {'frames_ok': False}
    if state['vendor'] is None or state['public'] is None:
        report['error'] = 'missing depth frames'
    else:
        vend = analyze_depth(state['vendor'], 'vendor', out)
        pub = analyze_depth(state['public'], 'public', out)
        report = {
            'frames_ok': True,
            'vendor': vend,
            'public': pub,
            'vendor_vs_public': {
                'same_wh': vend['width'] == pub['width'] and vend['height'] == pub['height'],
                'same_step': vend['step'] == pub['step'],
                'same_sha1': vend['sha1_16'] == pub['sha1_16'],
                'valid_ratio_delta': abs(vend['valid_ratio'] - pub['valid_ratio']),
                'bbox_equal': vend['valid_bbox'] == pub['valid_bbox'],
                'conclusion': (
                    'VENDOR_ALREADY_LEFT_TRUNCATED → Camera/SDK/ascamera'
                    if vend.get('systematic_spatial_truncation')
                    else (
                        'PUBLIC_DIFFERS_FROM_VENDOR → bridge/ROS'
                        if vend['sha1_16'] != pub['sha1_16']
                        else 'VENDOR_AND_PUBLIC_MATCH'
                    )
                ),
            },
            'camera_info': {
                'vendor_depth': None
                if state['vinfo'] is None
                else {
                    'wh': [state['vinfo'].width, state['vinfo'].height],
                    'k': [float(x) for x in state['vinfo'].k],
                    'frame_id': state['vinfo'].header.frame_id,
                },
                'public_depth': None
                if state['pinfo'] is None
                else {
                    'wh': [state['pinfo'].width, state['pinfo'].height],
                    'k': [float(x) for x in state['pinfo'].k],
                    'frame_id': state['pinfo'].header.frame_id,
                },
                'rgb': None
                if state['rinfo'] is None
                else {
                    'wh': [state['rinfo'].width, state['rinfo'].height],
                    'k': [float(x) for x in state['rinfo'].k],
                    'frame_id': state['rinfo'].header.frame_id,
                },
            },
            'rgb_image': None
            if state['rgb'] is None
            else {
                'wh': [state['rgb'].width, state['rgb'].height],
                'step': state['rgb'].step,
                'encoding': state['rgb'].encoding,
                'frame_id': state['rgb'].header.frame_id,
            },
            'notes': {
                'no_648x488_observed_in_ros': True,
                'requested_launch_depth_wh': [640, 480],
                'publisher_copies_sdk_frame_wh': True,
            },
        }
    path = out / 'track_a_probe.json'
    path.write_text(json.dumps(report, indent=2), encoding='utf-8')
    print(json.dumps(report, indent=2))
    n.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
