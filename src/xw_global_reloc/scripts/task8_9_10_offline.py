#!/usr/bin/env python3
"""Offline Stage3+5 on leave-one-out pairs (dry-run, no AMCL)."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import cv2
import numpy as np
import yaml
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan

from xw_global_reloc.acceptance import decide, evaluate_candidate
from xw_global_reloc.geometry import verify_pnp_rgbd
from xw_global_reloc.laser_verify import DistanceField, score_scan_at_pose
from xw_global_reloc.orb_utils import unpack_keypoints
from xw_global_reloc.retrieval import retrieve_topk
from xw_global_reloc.transforms import Pose2D, compose_candidate_base_pose, se3_from_xyz_rpy


def yaw_err(a, b):
    return abs(math.atan2(math.sin(a - b), math.cos(a - b)))


def load_map(yaml_path: Path) -> DistanceField:
    import yaml as y

    meta = y.safe_load(yaml_path.read_text(encoding='utf-8'))
    img = yaml_path.parent / meta['image']
    pgm = cv2.imread(str(img), cv2.IMREAD_UNCHANGED)
    # OccupancyGrid-like from pgm (trinary)
    h, w = pgm.shape[:2]
    grid = OccupancyGrid()
    grid.info.resolution = float(meta['resolution'])
    grid.info.width = w
    grid.info.height = h
    grid.info.origin.position.x = float(meta['origin'][0])
    grid.info.origin.position.y = float(meta['origin'][1])
    # PGM: 0 black occupied often depending on negate
    negate = int(meta.get('negate', 0))
    data = []
    flat = pgm.reshape(-1)
    for v in flat:
        if negate:
            v = 255 - int(v)
        else:
            v = int(v)
        if v < 50:
            data.append(100)
        elif v > 200:
            data.append(0)
        else:
            data.append(-1)
    grid.data = data
    return DistanceField(grid)


def synthetic_scan_from_pose(field: DistanceField, x, y, yaw, n=90):
    """Approx laser by raycasting distance field (debug only)."""
    scan = LaserScan()
    scan.angle_min = -math.pi
    scan.angle_max = math.pi
    scan.angle_increment = (2 * math.pi) / n
    scan.range_min = 0.2
    scan.range_max = 8.0
    ranges = []
    for i in range(n):
        a = scan.angle_min + i * scan.angle_increment
        # lidar yaw offset π
        ly = yaw + math.pi
        best = scan.range_max
        for r in np.linspace(0.2, 8.0, 40):
            mx = x + math.cos(ly) * (r * math.cos(a)) - math.sin(ly) * (r * math.sin(a))
            my = y + math.sin(ly) * (r * math.cos(a)) + math.cos(ly) * (r * math.sin(a))
            if field.sample_dist(mx, my) < 0.05:
                best = float(r)
                break
        ranges.append(best)
    scan.ranges = ranges
    return scan


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--db', required=True)
    ap.add_argument('--map-yaml', default='/ros2_ws/maps/vp.yaml')
    ap.add_argument('--out', default='/ros2_ws/bench/phase2a_poc_v1_2026-09-07/task8_9_10_offline.json')
    ap.add_argument('--max-trials', type=int, default=10)
    args = ap.parse_args()
    root = Path(args.db)
    idx = json.loads((root / 'descriptors' / 'index.json').read_text(encoding='utf-8'))
    kfs = []
    for item in idx:
        kdir = root / 'keyframes' / item['id']
        meta = yaml.safe_load((kdir / 'meta.yaml').read_text(encoding='utf-8'))
        kfs.append(
            {
                'id': item['id'],
                'meta': meta,
                'descriptors': np.load(str(kdir / 'descriptors.npy')),
                'keypoints': unpack_keypoints(np.load(str(kdir / 'keypoints.npy'))),
                'rgb': cv2.imread(str(kdir / 'rgb.jpg')),
                'depth': cv2.imread(str(kdir / 'depth.png'), cv2.IMREAD_UNCHANGED),
                'retrieval_ready': bool(meta.get('retrieval_ready', True)),
                'geometry_ready': bool(meta.get('geometry_ready', False)),
            }
        )
    field = load_map(Path(args.map_yaml))
    T_bc = se3_from_xyz_rpy(0.251, 0.0, 0.49, -1.33, 0.0, -1.5708)
    trials = []
    queries = [k for k in kfs if k['retrieval_ready']][: args.max_trials]
    for q in queries:
        pool = [k for k in kfs if k['id'] != q['id'] and k['retrieval_ready']]
        retr = retrieve_topk(q['rgb'], pool, top_k=10)
        K = np.array(q['meta']['camera_info']['k'], dtype=np.float64).reshape(3, 3)
        scale = 0.001
        q_depth_m = q['depth'].astype(np.float32) * scale
        cand_evals = []
        details = []
        for c in retr.candidates:
            kf = next(k for k in pool if k['id'] == c.keyframe_id)
            if not kf['geometry_ready'] or kf['depth'] is None:
                details.append({'id': kf['id'], 'skipped': 'not_geometry_ready'})
                continue
            geom = verify_pnp_rgbd(
                unpack_keypoints(np.load(str(root / 'keyframes' / q['id'] / 'keypoints.npy'))),
                np.load(str(root / 'keyframes' / q['id'] / 'descriptors.npy')),
                q_depth_m,
                kf['keypoints'],
                kf['descriptors'],
                kf['depth'].astype(np.float32) * scale,
                K,
            )
            # reload query kps properly
            from xw_global_reloc.orb_utils import extract_orb, make_orb

            qorb = extract_orb(q['rgb'], make_orb(1000))
            geom = verify_pnp_rgbd(
                qorb.keypoints,
                qorb.descriptors,
                q_depth_m,
                kf['keypoints'],
                kf['descriptors'],
                kf['depth'].astype(np.float32) * scale,
                K,
            )
            entry = {
                'id': kf['id'],
                'rank': c.rank,
                'geom_ok': geom.accepted,
                'reason': geom.reason,
                'matches': geom.matches,
                'inliers': geom.inliers,
                'inlier_ratio': geom.inlier_ratio,
                'reproj': geom.reproj_error,
                'depth_consistency': geom.depth_consistency,
                'pattern_reproj_ok_depth_bad': (
                    geom.inliers >= 12 and geom.reproj_error <= 4.0 and geom.depth_consistency < 0.55
                ),
            }
            if geom.accepted and geom.T_query_from_kf is not None:
                mp = kf['meta']['map_pose']
                pose, _ = compose_candidate_base_pose(
                    Pose2D(mp['x'], mp['y'], mp['yaw']), geom.T_query_from_kf, T_bc
                )
                # hard-negative laser: score at candidate; also score GT for reference
                gt = q['meta']['map_pose']
                scan = synthetic_scan_from_pose(field, gt['x'], gt['y'], gt['yaw'])
                laser = score_scan_at_pose(field, scan, pose.x, pose.y, pose.yaw)
                # wrong pose laser: shift candidate far
                laser_wrong = score_scan_at_pose(field, scan, pose.x + 3.0, pose.y + 3.0, pose.yaw)
                ev = evaluate_candidate(kf['id'], pose, geom, laser, field)
                cand_evals.append(ev)
                entry.update(
                    {
                        'cand_pose': pose.as_tuple(),
                        'laser_score': laser.laser_score,
                        'laser_ok': laser.accepted,
                        'laser_matched_ratio': laser.matched_ratio,
                        'laser_mean': laser.mean_dist,
                        'laser_p90': laser.p90_dist,
                        'laser_valid_beams': laser.valid_beams,
                        'laser_wrong_pose_score': laser_wrong.laser_score,
                        'laser_rejects_wrong': not laser_wrong.accepted,
                    }
                )
            details.append(entry)

        decision = decide(cand_evals)
        gt = q['meta']['map_pose']
        pos_err = yaw_e = None
        if decision.best is not None:
            pos_err = math.hypot(decision.best.pose.x - gt['x'], decision.best.pose.y - gt['y'])
            yaw_e = yaw_err(decision.best.pose.yaw, gt['yaw'])
        trials.append(
            {
                'query_id': q['id'],
                'gt_pose': gt,
                'decision': decision.status,
                'reason': decision.reason,
                'position_error_m': pos_err,
                'yaw_error_rad': yaw_e,
                'retrieval_top1': retr.candidates[0].keyframe_id if retr.candidates else None,
                'details': details,
                'false_accept': bool(
                    decision.status == 'ACCEPT'
                    and pos_err is not None
                    and (pos_err > 1.5 or (yaw_e is not None and yaw_e > 0.7))
                ),
            }
        )

    fa = sum(1 for t in trials if t['false_accept'])
    accepts = sum(1 for t in trials if t['decision'] == 'ACCEPT')
    report = {
        'n_trials': len(trials),
        'accepts': accepts,
        'false_accepts': fa,
        'unknown_or_rejected': len(trials) - accepts,
        'reproj_ok_depth_bad_count': sum(
            1 for t in trials for d in t['details'] if d.get('pattern_reproj_ok_depth_bad')
        ),
        'trials': trials,
        'note': 'Offline dry-run; laser uses synthetic scan from GT+map (not live /scan). apply_initial_pose forbidden.',
    }
    Path(args.out).write_text(json.dumps(report, indent=2), encoding='utf-8')
    print(json.dumps({k: report[k] for k in report if k != 'trials'}, indent=2))


if __name__ == '__main__':
    main()
