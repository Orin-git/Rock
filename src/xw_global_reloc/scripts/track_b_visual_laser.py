#!/usr/bin/env python3
"""Track B — Visual Top-K + Laser refine dry-run (apply_initial_pose forbidden)."""

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
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import Bool
from tf2_ros import Buffer, TransformException, TransformListener

from xw_global_reloc.laser_refine import refine_candidate_with_laser
from xw_global_reloc.laser_verify import DistanceField
from xw_global_reloc.orb_utils import unpack_keypoints
from xw_global_reloc.retrieval import retrieve_topk
from xw_global_reloc.transforms import Pose2D


_SENSOR = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=5,
)
_MAP = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)
_LATCH = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)


def yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z), 1.0 - 2.0 * (q.z * q.z))


def load_db(root: Path):
    idx = json.loads((root / 'descriptors' / 'index.json').read_text(encoding='utf-8'))
    out = []
    for item in idx:
        kdir = root / 'keyframes' / item['id']
        meta = yaml.safe_load((kdir / 'meta.yaml').read_text(encoding='utf-8'))
        if not meta.get('retrieval_ready', True):
            continue
        out.append(
            {
                'id': item['id'],
                'descriptors': np.load(str(kdir / 'descriptors.npy')),
                'meta': meta,
                'rgb': cv2.imread(str(kdir / 'rgb.jpg')),
                'pose': meta['map_pose'],
            }
        )
    return out


def main():
    root = Path('/ros2_ws/maps/vp/visual')
    out_path = Path('/ros2_ws/bench/phase2a_poc_v1_2026-09-07/track_b_visual_laser.json')
    kfs = load_db(root)
    rclpy.init()
    n = Node('track_b_vl')
    bridge = CvBridge()
    state = {'rgb': None, 'scan': None, 'map': None}
    n.create_subscription(Image, '/camera/front_up/color/image_raw', lambda m: state.update(rgb=m), _SENSOR)
    n.create_subscription(LaserScan, '/scan', lambda m: state.update(scan=m), _SENSOR)
    n.create_subscription(OccupancyGrid, '/map', lambda m: state.update(map=m), _MAP)
    req = n.create_publisher(Bool, '/xw/reloc/rgb_request', _LATCH)
    req.publish(Bool(data=True))
    tf = Buffer()
    TransformListener(tf, n)

    t0 = time.time()
    while time.time() - t0 < 15:
        rclpy.spin_once(n, timeout_sec=0.05)
        if state['rgb'] is not None and state['scan'] is not None and state['map'] is not None:
            break
    req.publish(Bool(data=False))
    if state['rgb'] is None or state['scan'] is None or state['map'] is None:
        report = {'error': 'missing rgb/scan/map', 'n_kfs': len(kfs)}
        out_path.write_text(json.dumps(report, indent=2), encoding='utf-8')
        print(json.dumps(report, indent=2))
        n.destroy_node()
        rclpy.shutdown()
        return

    try:
        tr = tf.lookup_transform('map', 'base_link', rclpy.time.Time())
        gt = Pose2D(
            tr.transform.translation.x,
            tr.transform.translation.y,
            yaw_from_quat(tr.transform.rotation),
        )
        gt_src = 'tf'
    except TransformException:
        gt = Pose2D(kfs[0]['pose']['x'], kfs[0]['pose']['y'], kfs[0]['pose']['yaw'])
        gt_src = 'db_fallback'

    field = DistanceField(state['map'])
    bgr = bridge.imgmsg_to_cv2(state['rgb'], 'bgr8')
    retr = retrieve_topk(bgr, kfs, top_k=8)

    # Hard-negative seeds: correct + wrong corridor offset
    trials = []
    # Live query trial
    laser_rows = []
    for c in retr.candidates:
        kf = next(k for k in kfs if k['id'] == c.keyframe_id)
        seed = Pose2D(kf['pose']['x'], kf['pose']['y'], kf['pose']['yaw'])
        ref = refine_candidate_with_laser(
            field,
            state['scan'],
            seed,
            coarse_xy_m=1.0,
            coarse_yaw_rad=math.radians(30),
            coarse_xy_step=0.20,
            coarse_yaw_step=math.radians(5),
            fine_xy_m=0.20,
            fine_yaw_rad=math.radians(5),
            fine_xy_step=0.05,
            fine_yaw_step=math.radians(1),
            beam_stride=8,
            top_n_coarse=3,
        )
        laser_rows.append(
            {
                'id': c.keyframe_id,
                'visual_rank': c.rank,
                'visual_score': c.score,
                'matches': c.ratio_matches,
                'seed': seed.as_tuple(),
                'refined': ref.refined.as_tuple(),
                'laser_ok': ref.accepted,
                'laser_reason': ref.reason,
                'laser_score': ref.top1_score,
                'margin_internal': ref.margin,
                'dx': ref.dx,
                'dy': ref.dy,
                'dyaw': ref.dyaw,
                'valid_beams': ref.score.valid_beams,
                'matched_ratio': ref.score.matched_ratio,
                'mean_dist': ref.score.mean_dist,
                'p90_dist': ref.score.p90_dist,
                'runtime': ref.runtime_sec,
                'pos_err_to_gt': math.hypot(ref.refined.x - gt.x, ref.refined.y - gt.y),
                'yaw_err_to_gt': abs(math.atan2(math.sin(ref.refined.yaw - gt.yaw), math.cos(ref.refined.yaw - gt.yaw))),
            }
        )
    survivors = [r for r in laser_rows if r['laser_ok']]
    survivors.sort(key=lambda r: r['laser_score'], reverse=True)
    decision = 'UNKNOWN'
    reason = 'no_survivor'
    best = None
    if survivors:
        best = survivors[0]
        margin = best['laser_score'] - (survivors[1]['laser_score'] if len(survivors) > 1 else 0.0)
        if margin < 0.03:
            decision = 'UNKNOWN'
            reason = 'margin'
        else:
            decision = 'ACCEPT'
            reason = 'ok'
            best['top_margin'] = margin

    # Hard-negative: force wrong seed far away
    wrong_seed = Pose2D(gt.x + 3.0, gt.y + 3.0, gt.yaw)
    wrong_ref = refine_candidate_with_laser(
        field, state['scan'], wrong_seed,
        coarse_xy_m=1.0, coarse_yaw_rad=math.radians(30),
        coarse_xy_step=0.20, coarse_yaw_step=math.radians(5),
        fine_xy_m=0.2, fine_yaw_rad=math.radians(5),
        fine_xy_step=0.05, fine_yaw_step=math.radians(1),
        beam_stride=8, top_n_coarse=3,
    )
    # Hard-negative: similar pose shifted along corridor-ish +1.5m x
    shift_seed = Pose2D(gt.x + 1.5, gt.y, gt.yaw)
    shift_ref = refine_candidate_with_laser(
        field, state['scan'], shift_seed,
        coarse_xy_m=1.0, coarse_yaw_rad=math.radians(30),
        coarse_xy_step=0.20, coarse_yaw_step=math.radians(5),
        fine_xy_m=0.2, fine_yaw_rad=math.radians(5),
        fine_xy_step=0.05, fine_yaw_step=math.radians(1),
        beam_stride=8, top_n_coarse=3,
    )

    # Offline LOO visual+laser using each KF rgb as query, live scan (scene-dependent caveat)
    loo = []
    for q in kfs[:10]:
        pool = [k for k in kfs if k['id'] != q['id']]
        r = retrieve_topk(q['rgb'], pool, top_k=5)
        rows = []
        for c in r.candidates:
            kf = next(k for k in pool if k['id'] == c.keyframe_id)
            seed = Pose2D(kf['pose']['x'], kf['pose']['y'], kf['pose']['yaw'])
            ref = refine_candidate_with_laser(
                field, state['scan'], seed,
                coarse_xy_m=0.8, coarse_yaw_rad=math.radians(25),
                coarse_xy_step=0.20, coarse_yaw_step=math.radians(5),
                fine_xy_m=0.15, fine_yaw_rad=math.radians(4),
                fine_xy_step=0.05, fine_yaw_step=math.radians(1),
                beam_stride=8, top_n_coarse=2,
            )
            rows.append(
                {
                    'id': c.keyframe_id,
                    'vrank': c.rank,
                    'vscore': c.score,
                    'laser_ok': ref.accepted,
                    'lscore': ref.top1_score,
                    'refined': ref.refined.as_tuple(),
                }
            )
        surv = sorted([x for x in rows if x['laser_ok']], key=lambda x: x['lscore'], reverse=True)
        loo.append(
            {
                'query': q['id'],
                'visual_top1': rows[0]['id'] if rows else None,
                'laser_top1': surv[0]['id'] if surv else None,
                'n_laser_ok': len(surv),
                'rows': rows,
            }
        )

    false_accept = False
    if decision == 'ACCEPT' and best is not None:
        if best['pos_err_to_gt'] > 1.5 or best['yaw_err_to_gt'] > 0.7:
            false_accept = True

    report = {
        'gt_pose': gt.as_tuple(),
        'gt_src': gt_src,
        'n_db': len(kfs),
        'live_query': {
            'decision': decision,
            'reason': reason,
            'false_accept': false_accept,
            'best': best,
            'visual_topk': laser_rows,
            'apply_initial_pose': False,
        },
        'hard_negative_wrong_seed': {
            'seed': wrong_seed.as_tuple(),
            'laser_ok': wrong_ref.accepted,
            'laser_score': wrong_ref.top1_score,
            'reason': wrong_ref.reason,
            'rejected_or_far': (not wrong_ref.accepted)
            or math.hypot(wrong_ref.refined.x - gt.x, wrong_ref.refined.y - gt.y) > 1.0,
        },
        'hard_negative_shift_seed': {
            'seed': shift_seed.as_tuple(),
            'laser_ok': shift_ref.accepted,
            'laser_score': shift_ref.top1_score,
            'reason': shift_ref.reason,
            'refined': shift_ref.refined.as_tuple(),
            'pos_err_to_gt': math.hypot(shift_ref.refined.x - gt.x, shift_ref.refined.y - gt.y),
        },
        'loo_visual_laser': loo,
        'note': 'Single-region DB; multi-region operator capture still required for formal gate.',
    }
    out_path.write_text(json.dumps(report, indent=2), encoding='utf-8')
    print(json.dumps({k: report[k] for k in report if k not in ('loo_visual_laser',)}, indent=2))
    print('loo_summary', [{'q': x['query'], 'v1': x['visual_top1'], 'l1': x['laser_top1'], 'n': x['n_laser_ok']} for x in loo])
    n.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
