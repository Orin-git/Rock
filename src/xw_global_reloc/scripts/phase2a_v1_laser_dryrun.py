#!/usr/bin/env python3
"""Phase2A V1 Visual+Laser dry-run (apply_initial_pose forbidden).

Uses captured query RGB + scan + AMCL GT. No /initialpose, no AMCL handoff.
Stops immediately on False Accept.
"""

from __future__ import annotations

import argparse
import json
import math
import time
from pathlib import Path

import cv2
import numpy as np
import yaml
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan

from xw_global_reloc.laser_refine import refine_candidate_with_laser
from xw_global_reloc.laser_verify import DistanceField
from xw_global_reloc.retrieval import retrieve_topk
from xw_global_reloc.transforms import Pose2D


DB = Path('/ros2_ws/maps/vp/visual')
MAP_YAML = Path('/ros2_ws/maps/vp.yaml')
BENCH = Path('/ros2_ws/bench/phase2a_poc_v1_2026-09-07')

# FA if ACCEPT but beyond AMCL-useful seed range.
FA_XY_M = 1.0
FA_YAW_RAD = 0.52  # 30 deg
# Candidate considered AMCL-convergent.
AMCL_XY_M = 0.50
AMCL_YAW_RAD = 0.35  # 20 deg


def yaw_norm(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


def yaw_err(a: float, b: float) -> float:
    return abs(yaw_norm(a - b))


def load_map(yaml_path: Path) -> DistanceField:
    meta = yaml.safe_load(yaml_path.read_text(encoding='utf-8'))
    img = yaml_path.parent / meta['image']
    pgm = cv2.imread(str(img), cv2.IMREAD_UNCHANGED)
    h, w = pgm.shape[:2]
    grid = OccupancyGrid()
    grid.info.resolution = float(meta['resolution'])
    grid.info.width = w
    grid.info.height = h
    grid.info.origin.position.x = float(meta['origin'][0])
    grid.info.origin.position.y = float(meta['origin'][1])
    negate = int(meta.get('negate', 0))
    data = []
    for v in pgm.reshape(-1):
        vv = 255 - int(v) if negate else int(v)
        if vv < 50:
            data.append(100)
        elif vv > 200:
            data.append(0)
        else:
            data.append(-1)
    grid.data = data
    return DistanceField(grid)


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


def load_kfs(root: Path):
    idx = json.loads((root / 'descriptors' / 'index.json').read_text(encoding='utf-8'))
    out = []
    for item in idx:
        kdir = root / 'keyframes' / item['id']
        meta = yaml.safe_load((kdir / 'meta.yaml').read_text(encoding='utf-8'))
        if not meta.get('retrieval_ready', True):
            continue
        scan_p = kdir / 'scan.npz'
        rgb = cv2.imread(str(kdir / 'rgb.jpg'))
        if rgb is None or not scan_p.is_file():
            continue
        out.append(
            {
                'id': item['id'],
                'descriptors': np.load(str(kdir / 'descriptors.npy')),
                'meta': meta,
                'rgb': rgb,
                'scan': scan_p,
                'pose': meta['map_pose'],
                'region_id': meta.get('region_id', 'unknown'),
                'region_class': meta.get('region_class', ''),
                'location_id': meta.get('location_id', ''),
            }
        )
    return out


def pick_queries(kfs, mode: str):
    """mode=debug → 5 loc × 2; mode=formal → ≥10 loc × ≥3."""
    by_loc = {}
    for k in kfs:
        by_loc.setdefault(k['location_id'] or k['id'], []).append(k)
    # Prefer A then B, spread regions.
    loc_ids = sorted(by_loc.keys())
    # Order: doorway, corridor, similar, room, open, charger
    def rank(lid: str) -> tuple:
        order = [
            'doorway',
            'corridor_wp2',
            'similar',
            'room_wp5',
            'open',
            'charger',
        ]
        pri = 99
        for i, key in enumerate(order):
            if key in lid:
                pri = i
                break
        ab = 0 if lid.endswith('_A') else 1
        return (pri, ab, lid)

    loc_ids = sorted(loc_ids, key=rank)
    selected = []
    if mode == 'debug':
        # 5 locations × 2 queries. Mix similar door/corridor/room.
        want = [lid for lid in loc_ids if lid.endswith('_A')][:5]
        for lid in want:
            selected.extend(by_loc[lid][:2])
        return selected[:10], want
    # formal: 10 locations × 3
    want = loc_ids[:10]
    for lid in want:
        qs = by_loc[lid]
        if len(qs) < 3:
            # pad from same region other location
            extra = [k for k in kfs if k['region_id'] == qs[0]['region_id'] and k not in qs]
            qs = qs + extra
        selected.extend(qs[:3])
    return selected[:30], want


def run_one(q, pool, field: DistanceField, top_k: int = 5) -> dict:
    t0 = time.monotonic()
    retr = retrieve_topk(q['rgb'], pool, top_k=top_k)
    scan = load_scan(q['scan'])
    gt = Pose2D(q['pose']['x'], q['pose']['y'], q['pose']['yaw'])
    rows = []
    for c in retr.candidates:
        kf = next(k for k in pool if k['id'] == c.keyframe_id)
        seed = Pose2D(kf['pose']['x'], kf['pose']['y'], kf['pose']['yaw'])
        ref = refine_candidate_with_laser(
            field,
            scan,
            seed,
            coarse_xy_m=1.0,
            coarse_yaw_rad=math.radians(30),
            coarse_xy_step=0.10,
            coarse_yaw_step=math.radians(3.0),
            fine_xy_m=0.20,
            fine_yaw_rad=math.radians(5.0),
            fine_xy_step=0.05,
            fine_yaw_step=math.radians(1.0),
            beam_stride=6,
            match_dist_m=0.25,
            min_valid_beams=20,
            min_laser_score=0.45,
            min_margin=0.03,
            max_refine_trans_m=1.2,
            max_refine_yaw_rad=math.radians(35.0),
            top_n_coarse=5,
        )
        pos_err = math.hypot(ref.refined.x - gt.x, ref.refined.y - gt.y)
        yerr = yaw_err(ref.refined.yaw, gt.yaw)
        rows.append(
            {
                'id': c.keyframe_id,
                'visual_rank': c.rank,
                'visual_score': c.score,
                'match_count': c.ratio_matches,
                'visual_region': kf['region_id'],
                'seed': seed.as_tuple(),
                'laser_refined': ref.refined.as_tuple(),
                'laser_ok': ref.accepted,
                'laser_reason': ref.reason,
                'laser_score': ref.top1_score,
                'laser_internal_margin': ref.margin,
                'position_error': pos_err,
                'yaw_error': yerr,
                'runtime': ref.runtime_sec,
            }
        )
    survivors = [r for r in rows if r['laser_ok']]
    survivors.sort(key=lambda r: r['laser_score'], reverse=True)
    decision = 'UNKNOWN'
    reason = 'no_survivor'
    best = None
    top_margin = 0.0
    if survivors:
        best = survivors[0]
        top_margin = best['laser_score'] - (survivors[1]['laser_score'] if len(survivors) > 1 else 0.0)
        if top_margin < 0.03:
            decision = 'UNKNOWN'
            reason = 'laser_top_margin'
        else:
            decision = 'ACCEPT'
            reason = 'visual_topk_and_laser_ok'
    false_accept = False
    amcl_ready = False
    if decision == 'ACCEPT' and best is not None:
        if best['position_error'] > FA_XY_M or best['yaw_error'] > FA_YAW_RAD:
            false_accept = True
        if best['position_error'] <= AMCL_XY_M and best['yaw_error'] <= AMCL_YAW_RAD:
            amcl_ready = True
    visual_margin = 0.0
    if len(retr.candidates) >= 2:
        visual_margin = retr.candidates[0].score - retr.candidates[1].score
    return {
        'query_id': q['id'],
        'gt_region': q['region_id'],
        'gt_class': q['region_class'],
        'gt_location': q['location_id'],
        'gt_x': gt.x,
        'gt_y': gt.y,
        'gt_yaw': gt.yaw,
        'visual_candidate': rows[0]['id'] if rows else None,
        'visual_topk': rows,
        'laser_refined_pose': None if best is None else best['laser_refined'],
        'position_error': None if best is None else best['position_error'],
        'yaw_error': None if best is None else best['yaw_error'],
        'laser_score': None if best is None else best['laser_score'],
        'top1_top2_margin': top_margin,
        'visual_top1_top2_margin': visual_margin,
        'decision': decision,
        'reason': reason,
        'false_accept': false_accept,
        'amcl_convergent_range': amcl_ready,
        'apply_initial_pose': False,
        'runtime': time.monotonic() - t0,
        'best': best,
    }


def summarize(trials: list) -> dict:
    n = len(trials)
    fa = sum(1 for t in trials if t['false_accept'])
    acc = sum(1 for t in trials if t['decision'] == 'ACCEPT')
    unk = sum(1 for t in trials if t['decision'] == 'UNKNOWN')
    correct = sum(
        1 for t in trials if t['decision'] == 'ACCEPT' and not t['false_accept']
    )
    amcl_n = sum(1 for t in trials if t.get('amcl_convergent_range'))
    xy = [t['position_error'] for t in trials if t['decision'] == 'ACCEPT' and t['position_error'] is not None]
    yaw = [t['yaw_error'] for t in trials if t['decision'] == 'ACCEPT' and t['yaw_error'] is not None]
    rt = [t['runtime'] for t in trials]
    return {
        'n': n,
        'ACCEPT': acc,
        'UNKNOWN': unk,
        'False_Accept': fa,
        'Correct_Accept': correct,
        'Correct_Accept_rate': correct / n if n else 0.0,
        'UNKNOWN_rate': unk / n if n else 0.0,
        'AMCL_range_accepts': amcl_n,
        'xy_error_accept_mean': float(np.mean(xy)) if xy else None,
        'xy_error_accept_max': float(np.max(xy)) if xy else None,
        'yaw_error_accept_mean': float(np.mean(yaw)) if yaw else None,
        'yaw_error_accept_max': float(np.max(yaw)) if yaw else None,
        'runtime_mean': float(np.mean(rt)) if rt else None,
        'runtime_max': float(np.max(rt)) if rt else None,
    }


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument('--mode', choices=['debug', 'formal'], required=True)
    ap.add_argument('--out', default='')
    args = ap.parse_args()
    out = Path(args.out) if args.out else BENCH / f'laser_dryrun_{args.mode}.json'
    kfs = load_kfs(DB)
    queries, loc_ids = pick_queries(kfs, args.mode)
    field = load_map(MAP_YAML)
    trials = []
    stopped_fa = False
    for q in queries:
        pool = [k for k in kfs if k['id'] != q['id']]
        row = run_one(q, pool, field)
        trials.append(row)
        print(
            json.dumps(
                {
                    'q': q['id'],
                    'region': q['region_id'],
                    'decision': row['decision'],
                    'fa': row['false_accept'],
                    'xy': row['position_error'],
                    'yaw': row['yaw_error'],
                    'runtime': round(row['runtime'], 2),
                }
            ),
            flush=True,
        )
        if row['false_accept']:
            stopped_fa = True
            break
    stats = summarize(trials)
    report = {
        'mode': args.mode,
        'apply_initial_pose': False,
        'locations': loc_ids,
        'n_db': len(kfs),
        'stopped_on_false_accept': stopped_fa,
        'stats': stats,
        'trials': trials,
        'fa_xy_m': FA_XY_M,
        'fa_yaw_rad': FA_YAW_RAD,
        'amcl_xy_m': AMCL_XY_M,
        'amcl_yaw_rad': AMCL_YAW_RAD,
    }
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(report, indent=2), encoding='utf-8')
    print(json.dumps({'stats': stats, 'stopped_fa': stopped_fa, 'out': str(out)}, indent=2))


if __name__ == '__main__':
    main()
