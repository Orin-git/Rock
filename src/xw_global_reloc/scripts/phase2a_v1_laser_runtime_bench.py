#!/usr/bin/env python3
"""Equivalence + speed check for vectorized laser refine (same semantics)."""

from __future__ import annotations

import json
import math
import time
from pathlib import Path

import numpy as np
import yaml
from sensor_msgs.msg import LaserScan

from xw_global_reloc.laser_refine import refine_candidate_with_laser
from xw_global_reloc.laser_verify import DistanceField, prepare_scan, score_scan_at_pose, score_scan_at_poses
from xw_global_reloc.transforms import Pose2D

# Import map loader from dryrun
import importlib.util

spec = importlib.util.spec_from_file_location(
    'dry', '/ros2_ws/src/xw_global_reloc/scripts/phase2a_v1_laser_dryrun.py'
)
dry = importlib.util.module_from_spec(spec)
spec.loader.exec_module(dry)

OUT = Path('/ros2_ws/bench/phase2a_poc_v1_2026-09-07/laser_runtime_equivalence.json')


def load_scan(path: Path) -> LaserScan:
    z = np.load(str(path), allow_pickle=True)
    scan = LaserScan()
    scan.ranges = z['ranges'].astype(np.float32).tolist()
    scan.angle_min = float(z['angle_min'])
    scan.angle_max = float(z['angle_max'])
    scan.angle_increment = float(z['angle_increment'])
    scan.range_min = float(z['range_min'])
    scan.range_max = float(z['range_max'])
    return scan


def main() -> None:
    field = dry.load_map(Path('/ros2_ws/maps/vp.yaml'))
    # Use one UNKNOWN + one ACCEPT query from debug set
    debug = json.loads(
        Path('/ros2_ws/bench/phase2a_poc_v1_2026-09-07/laser_dryrun_debug_cluster.json').read_text()
    )
    qids = ['kf_000014', 'kf_000013', 'kf_000033']
    root = Path('/ros2_ws/maps/vp/visual/keyframes')
    rows = []
    for kid in qids:
        meta = yaml.safe_load((root / kid / 'meta.yaml').read_text())
        scan = load_scan(root / kid / 'scan.npz')
        seed = Pose2D(meta['map_pose']['x'], meta['map_pose']['y'], meta['map_pose']['yaw'])
        # Reference: single-pose API (vectorized under the hood now)
        prep = prepare_scan(scan, beam_stride=6)
        # Batch vs single equivalence on a small yaw grid
        xs = np.array([seed.x, seed.x + 0.1, seed.x - 0.1])
        ys = np.array([seed.y, seed.y, seed.y + 0.1])
        yaws = np.array([seed.yaw, seed.yaw + 0.05, seed.yaw - 0.05])
        batch = score_scan_at_poses(field, prep, xs, ys, yaws, min_laser_score=0.45)
        singles = [
            score_scan_at_pose(field, scan, float(xs[i]), float(ys[i]), float(yaws[i]), prepared=prep)
            for i in range(3)
        ]
        score_deltas = [abs(batch[i].laser_score - singles[i].laser_score) for i in range(3)]
        t0 = time.monotonic()
        ref = refine_candidate_with_laser(
            field,
            scan,
            seed,
            beam_stride=6,
            min_laser_score=0.45,
            reject_local_grid_margin=False,
            prepared=prep,
        )
        dt = time.monotonic() - t0
        # Compare against stored debug refined for same KF as query seed of itself
        stored = None
        for t in debug['trials']:
            if t['query_id'] == kid:
                # find same-id candidate if present else nearest seed
                for r in t['visual_topk']:
                    if r['id'] == kid:
                        stored = r
                        break
                break
        rows.append(
            {
                'id': kid,
                'batch_vs_single_max_abs_score_delta': max(score_deltas),
                'refine_runtime_sec': dt,
                'stage_timings': ref.stage_timings,
                'refined': ref.refined.as_tuple(),
                'laser_score': ref.top1_score,
                'accepted': ref.accepted,
                'reason': ref.reason,
                'n_eval': ref.candidates_evaluated,
                'stored_self_cand': None
                if stored is None
                else {
                    'laser_score': stored['laser_score'],
                    'refined': stored['laser_refined'],
                    'runtime': stored['runtime'],
                },
            }
        )
    # Full E2E timing sample: one trial via dry.run_one
    kfs = dry.load_kfs(dry.DB)
    q = next(k for k in kfs if k['id'] == 'kf_000014')
    pool = [k for k in kfs if k['id'] != q['id']]
    t0 = time.monotonic()
    one = dry.run_one(q, pool, field, top_k=5)
    e2e = time.monotonic() - t0
    q3 = dry.run_one(q, pool, field, top_k=3)
    report = {
        'micro': rows,
        'e2e_sample_kf_000014_top5': {
            'runtime': e2e,
            'stage_timings': one.get('stage_timings'),
            'decision': one['decision'],
            'xy': one['position_error'],
            'yaw': one['yaw_error'],
            'laser_score': one['laser_score'],
        },
        'e2e_sample_kf_000014_top3': {
            'runtime': q3['runtime'],
            'stage_timings': q3.get('stage_timings'),
            'decision': q3['decision'],
            'xy': q3['position_error'],
            'yaw': q3['yaw_error'],
            'laser_score': q3['laser_score'],
        },
        'prior_debug_runtime_mean': debug['stats'].get('runtime_mean'),
    }
    OUT.write_text(json.dumps(report, indent=2), encoding='utf-8')
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
