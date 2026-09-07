#!/usr/bin/env python3
"""Dense local keyframe capture for PoC qualification (amcl pose + nearest pair)."""

from __future__ import annotations

import json
import math
import time
from pathlib import Path

import cv2
import numpy as np
import rclpy
import yaml
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Bool

from xw_global_reloc.map_hash import map_pair_hash, resolve_map_files
from xw_global_reloc.orb_utils import extract_orb, make_orb, pack_keypoints
from xw_global_reloc.pairing import ImageRingBuffer, pair_nearest
from xw_global_reloc.quality import evaluate_quality
from xw_global_reloc.transforms import Pose2D, se3_from_xyz_rpy, se3_to_se2_yaw


def yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z), 1.0 - 2.0 * (q.z * q.z))


def main():
    maps_dir = Path('/ros2_ws/maps')
    map_name = 'vp'
    root = maps_dir / map_name / 'visual'
    (root / 'keyframes').mkdir(parents=True, exist_ok=True)
    (root / 'descriptors').mkdir(parents=True, exist_ok=True)
    (root / 'state').mkdir(parents=True, exist_ok=True)
    yaml_p, pgm_p = resolve_map_files(maps_dir, map_name)
    mhash = map_pair_hash(yaml_p, pgm_p)
    (root / 'manifest.yaml').write_text(
        yaml.safe_dump(
            {
                'schema_version': 2,
                'map_name': map_name,
                'map_hash': mhash,
                'extrinsic_status': 'EXTRINSIC_UNCALIBRATED',
                'tiers': ['retrieval_ready', 'geometry_ready'],
            },
            sort_keys=False,
        ),
        encoding='utf-8',
    )

    qos = QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
        history=HistoryPolicy.KEEP_LAST,
        depth=5,
    )
    latch = QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
    )
    rclpy.init()
    n = Node('kf_capture_once')
    bridge = CvBridge()
    orb_det = make_orb(1000)
    rgb_buf, depth_buf = ImageRingBuffer(40), ImageRingBuffer(40)
    state = {'amcl': None, 'info': None}
    n.create_subscription(Image, '/camera/front_up/color/image_raw', lambda m: rgb_buf.push(m), qos)
    n.create_subscription(Image, '/camera/front_up/depth/image_raw', lambda m: depth_buf.push(m), qos)
    n.create_subscription(CameraInfo, '/camera/front_up/color/camera_info', lambda m: state.update(info=m), qos)
    n.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', lambda m: state.update(amcl=m), 10)
    req = n.create_publisher(Bool, '/xw/reloc/rgb_request', latch)
    req.publish(Bool(data=True))

    # Force AMCL to publish while stationary.
    from std_srvs.srv import Empty
    from tf2_ros import Buffer, TransformListener, TransformException

    nomotion = n.create_client(Empty, '/request_nomotion_update')
    tf_buf = Buffer()
    TransformListener(tf_buf, n)
    if nomotion.wait_for_service(timeout_sec=2.0):
        nomotion.call_async(Empty.Request())

    def pose_from_tf_or_amcl():
        if state['amcl'] is not None:
            p = state['amcl'].pose.pose
            return Pose2D(p.position.x, p.position.y, yaw_from_quat(p.orientation)), 'amcl_pose'
        try:
            tf = tf_buf.lookup_transform('map', 'base_link', rclpy.time.Time())
            t = tf.transform.translation
            q = tf.transform.rotation
            return Pose2D(t.x, t.y, yaw_from_quat(q)), 'tf'
        except TransformException:
            return None, None

    target = 12
    interval = 2.0
    idx = []
    count = 0
    last = 0.0
    seen = set()
    t0 = time.time()
    while time.time() - t0 < 50 and count < target:
        rclpy.spin_once(n, timeout_sec=0.05)
        if state['info'] is None:
            continue
        pose, pose_src = pose_from_tf_or_amcl()
        if pose is None:
            continue
        pair = pair_nearest(rgb_buf, depth_buf, prefer='rgb')
        if pair is None:
            continue
        key = round(pair.rgb_t, 3)
        if key in seen:
            continue
        if time.time() - last < interval and count > 0:
            continue
        bgr = bridge.imgmsg_to_cv2(pair.rgb, 'bgr8')
        depth = bridge.imgmsg_to_cv2(pair.depth, 'passthrough')
        if depth.dtype != np.uint16:
            continue
        orb = extract_orb(bgr, orb_det)
        q = evaluate_quality(
            pose_ok=True,
            camera_info_ok=True,
            orb=orb,
            depth_u16=depth,
            pair_dt_sec=pair.pair_dt_sec,
            min_orb=50,
            max_pair_dt=0.05,
            min_depth_valid=0.08,
            min_orb_depth_cov=0.25,
        )
        # Always allow retrieval_ready if ORB enough; geometry may be false
        if orb.n_features < 50:
            continue
        q.retrieval_ready = True
        _ = pose_src
        count += 1
        seen.add(key)
        last = time.time()
        kid = f'kf_{count:06d}'
        kdir = root / 'keyframes' / kid
        kdir.mkdir(parents=True, exist_ok=True)
        cv2.imwrite(str(kdir / 'rgb.jpg'), bgr, [int(cv2.IMWRITE_JPEG_QUALITY), 90])
        cv2.imwrite(str(kdir / 'depth.png'), depth)
        np.save(str(kdir / 'descriptors.npy'), orb.descriptors)
        np.save(str(kdir / 'keypoints.npy'), pack_keypoints(orb.keypoints))
        T_bc = se3_from_xyz_rpy(0.251, 0.0, 0.49, -1.33, 0.0, -1.5708)
        T_mb = se3_from_xyz_rpy(pose.x, pose.y, 0.0, 0.0, 0.0, pose.yaw)
        cam = se3_to_se2_yaw(T_mb @ T_bc)
        meta = {
            'keyframe_id': kid,
            'timestamp': time.time(),
            'stamp_rgb': pair.rgb_t,
            'stamp_depth': pair.depth_t,
            'pair_dt_sec': pair.pair_dt_sec,
            'map_pose': {'x': pose.x, 'y': pose.y, 'yaw': pose.yaw},
            'camera_pose': {'x': cam.x, 'y': cam.y, 'yaw': cam.yaw, 'extrinsic_status': 'EXTRINSIC_UNCALIBRATED'},
            'camera_info': {
                'width': int(state['info'].width),
                'height': int(state['info'].height),
                'k': [float(x) for x in state['info'].k],
                'd': [float(x) for x in state['info'].d],
                'frame_id': str(state['info'].header.frame_id),
            },
            'retrieval_ready': True,
            'geometry_ready': bool(q.geometry_ready),
            'quality': {
                'orb_features': q.orb_total,
                'orb_with_valid_depth': q.orb_with_valid_depth,
                'orb_depth_coverage_ratio': q.orb_depth_coverage_ratio,
                'depth_valid_ratio': q.depth_valid_ratio,
                'pair_dt_sec': pair.pair_dt_sec,
                'reasons': q.reasons,
            },
            'map_hash': mhash,
            'region_tag': 'charger_local',
            'extrinsic_status': 'EXTRINSIC_UNCALIBRATED',
        }
        (kdir / 'meta.yaml').write_text(yaml.safe_dump(meta, sort_keys=False), encoding='utf-8')
        idx.append(
            {
                'id': kid,
                'pose': meta['map_pose'],
                'retrieval_ready': True,
                'geometry_ready': bool(q.geometry_ready),
                'orb_features': q.orb_total,
            }
        )
        print(f'saved {kid} geo={q.geometry_ready} cov={q.orb_depth_coverage_ratio:.3f} dt_ms={pair.pair_dt_sec*1000:.1f}')

    req.publish(Bool(data=False))
    (root / 'descriptors' / 'index.json').write_text(json.dumps(idx, indent=2), encoding='utf-8')
    usage = sum(p.stat().st_size for p in root.rglob('*') if p.is_file())
    feats = [i['orb_features'] for i in idx]
    geos = sum(1 for i in idx if i['geometry_ready'])
    stats = {
        'keyframe_count': len(idx),
        'retrieval_ready': len(idx),
        'geometry_ready': geos,
        'disk_bytes': usage,
        'orb_features': {
            'mean': float(np.mean(feats)) if feats else None,
            'p10': float(np.percentile(feats, 10)) if feats else None,
            'p90': float(np.percentile(feats, 90)) if feats else None,
        },
        'orb_depth_coverage_mean': float(
            np.mean(
                [
                    yaml.safe_load((root / 'keyframes' / i['id'] / 'meta.yaml').read_text())['quality'][
                        'orb_depth_coverage_ratio'
                    ]
                    for i in idx
                ]
            )
        )
        if idx
        else None,
        'map_coverage': {'note': 'single region charger_local — operator multi-region pending'},
        'map_hash': mhash,
    }
    (root / 'state' / 'builder_stats.json').write_text(json.dumps(stats, indent=2), encoding='utf-8')
    print(json.dumps(stats, indent=2))
    n.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
