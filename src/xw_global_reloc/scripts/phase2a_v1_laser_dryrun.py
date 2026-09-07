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
from xw_global_reloc.laser_verify import DistanceField, prepare_scan
from xw_global_reloc.pose_cluster_accept import (
    ClusterMember,
    absolute_gate_member,
    decide_pose_clusters,
    decision_to_dict,
)
from xw_global_reloc.retrieval import retrieve_topk
from xw_global_reloc.transforms import Pose2D


DB = Path('/ros2_ws/maps/vp/visual')
MAP_YAML = Path('/ros2_ws/maps/vp.yaml')
BENCH = Path('/ros2_ws/bench/phase2a_poc_v1_2026-09-07')

# FA if ACCEPT but beyond AMCL-useful seed range.
FA_XY_M = 1.0
FA_YAW_RAD = 0.52  # 30 deg
# PoC engineering ACCEPT quality (debug gate).
POC_XY_M = 0.25
POC_YAW_RAD = math.radians(5.0)
# Candidate considered AMCL-convergent.
AMCL_XY_M = 0.50
AMCL_YAW_RAD = 0.35  # 20 deg

CLUSTER_XY_M = 0.25
CLUSTER_YAW_RAD = math.radians(6.0)
CLUSTER_MIN_MARGIN = 0.03
# Data-backed (see laser_score_threshold_analysis.json): FA=0, CA≥8/10; margin over wrong_max≈0.348.
MIN_LASER_SCORE = 0.38
MAX_REFINE_TRANS_M = 1.2
MAX_REFINE_YAW_RAD = math.radians(35.0)


def yaw_norm(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


def yaw_err(a: float, b: float) -> float:
    return abs(yaw_norm(a - b))


def load_map(yaml_path: Path) -> DistanceField:
    """Load OccupancyGrid like map_server: PGM row0 is image top, OccupancyGrid row0 is origin (flipud)."""
    meta = yaml.safe_load(yaml_path.read_text(encoding='utf-8'))
    img = yaml_path.parent / meta['image']
    pgm = cv2.imread(str(img), cv2.IMREAD_UNCHANGED)
    if pgm is None:
        raise FileNotFoundError(img)
    if pgm.ndim == 3:
        pgm = cv2.cvtColor(pgm, cv2.COLOR_BGR2GRAY)
    img_u8 = np.flipud(np.asarray(pgm, dtype=np.uint8))
    h, w = img_u8.shape[:2]
    grid = OccupancyGrid()
    grid.info.resolution = float(meta['resolution'])
    grid.info.width = int(w)
    grid.info.height = int(h)
    grid.info.origin.position.x = float(meta['origin'][0])
    grid.info.origin.position.y = float(meta['origin'][1])
    negate = int(meta.get('negate', 0))
    occ_t = float(meta.get('occupied_thresh', 0.65))
    free_t = float(meta.get('free_thresh', 0.25))
    pix = img_u8.astype(np.float64)
    occ = (pix / 255.0) if negate else ((255.0 - pix) / 255.0)
    data = np.full(occ.shape, -1, dtype=np.int8)
    data[occ > occ_t] = 100
    data[occ < free_t] = 0
    grid.data = data.reshape(-1).astype(np.int8).tolist()
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
    stage = {}
    t_r = time.monotonic()
    retr = retrieve_topk(q['rgb'], pool, top_k=top_k)
    stage['retrieval'] = time.monotonic() - t_r
    scan = load_scan(q['scan'])
    t_p = time.monotonic()
    prepared = prepare_scan(scan, beam_stride=6)
    stage['prepare_scan'] = time.monotonic() - t_p
    gt = Pose2D(q['pose']['x'], q['pose']['y'], q['pose']['yaw'])
    rows = []
    members: list = []
    laser_stage_sum = {'coarse_search': 0.0, 'fine_search': 0.0, 'prepare_scan': 0.0, 'total': 0.0}
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
            min_laser_score=MIN_LASER_SCORE,
            min_margin=0.03,
            max_refine_trans_m=MAX_REFINE_TRANS_M,
            max_refine_yaw_rad=MAX_REFINE_YAW_RAD,
            top_n_coarse=5,
            reject_local_grid_margin=False,
            prepared=prepared,
        )
        for k, v in (ref.stage_timings or {}).items():
            laser_stage_sum[k] = laser_stage_sum.get(k, 0.0) + float(v)
        refined_t = ref.refined.as_tuple()
        seed_t = seed.as_tuple()
        abs_ok, abs_reason, dx, dy, dyaw = absolute_gate_member(
            refined=refined_t,
            seed=seed_t,
            laser_score=ref.top1_score,
            min_laser_score=MIN_LASER_SCORE,
            max_refine_trans_m=MAX_REFINE_TRANS_M,
            max_refine_yaw_rad=MAX_REFINE_YAW_RAD,
            free_space=field.is_free(ref.refined.x, ref.refined.y),
            valid_beams=ref.score.valid_beams,
            min_valid_beams=20,
            legacy_reason=ref.reason,
        )
        # Prefer refine absolute result if stricter
        if not ref.accepted and ref.reason != 'ambiguous_margin':
            abs_ok = False
            abs_reason = ref.reason
        pos_err = math.hypot(ref.refined.x - gt.x, ref.refined.y - gt.y)
        yerr = yaw_err(ref.refined.yaw, gt.yaw)
        rows.append(
            {
                'id': c.keyframe_id,
                'visual_rank': c.rank,
                'visual_score': c.score,
                'match_count': c.ratio_matches,
                'visual_region': kf['region_id'],
                'seed': seed_t,
                'laser_refined': refined_t,
                'laser_ok': abs_ok,
                'laser_reason': abs_reason,
                'laser_score': ref.top1_score,
                'laser_internal_margin': ref.margin,
                'valid_beams': ref.score.valid_beams,
                'matched_ratio': ref.score.matched_ratio,
                'mean_dist': ref.score.mean_dist,
                'p90_dist': ref.score.p90_dist,
                'dx': dx,
                'dy': dy,
                'dyaw': dyaw,
                'position_error': pos_err,
                'yaw_error': yerr,
                'runtime': ref.runtime_sec,
                'stage_timings': ref.stage_timings,
            }
        )
        members.append(
            ClusterMember(
                keyframe_id=c.keyframe_id,
                refined=refined_t,
                seed=seed_t,
                laser_score=float(ref.top1_score),
                visual_rank=int(c.rank),
                visual_score=float(c.score),
                visual_region=str(kf['region_id']),
                dx=dx,
                dy=dy,
                dyaw=dyaw,
                valid_beams=ref.score.valid_beams,
                matched_ratio=ref.score.matched_ratio,
                mean_dist=ref.score.mean_dist,
                p90_dist=ref.score.p90_dist,
                absolute_ok=abs_ok,
                absolute_reason=abs_reason,
            )
        )
    t_cl = time.monotonic()
    dec = decide_pose_clusters(
        members,
        cluster_xy_m=CLUSTER_XY_M,
        cluster_yaw_rad=CLUSTER_YAW_RAD,
        cluster_min_score_margin=CLUSTER_MIN_MARGIN,
    )
    stage['pose_clustering'] = time.monotonic() - t_cl
    stage['laser_coarse_sum'] = laser_stage_sum.get('coarse_search', 0.0)
    stage['laser_fine_sum'] = laser_stage_sum.get('fine_search', 0.0)
    stage['laser_total_sum'] = laser_stage_sum.get('total', 0.0)
    stage['n_topk'] = len(retr.candidates)
    decision = dec.status
    reason = dec.reason
    best = None
    refined_pose = None
    pos_err = None
    yerr = None
    laser_score = None
    if dec.best_cluster is not None:
        m0 = dec.best_cluster.members[0]
        refined_pose = list(dec.best_cluster.center)
        pos_err = math.hypot(refined_pose[0] - gt.x, refined_pose[1] - gt.y)
        yerr = yaw_err(refined_pose[2], gt.yaw)
        laser_score = dec.best_cluster.best_laser_score
        best = next(r for r in rows if r['id'] == m0.keyframe_id)
        best = dict(best)
        best['cluster_support'] = dec.best_cluster.support_count
        best['cluster_center'] = refined_pose
    false_accept = False
    amcl_ready = False
    poc_quality = False
    if decision == 'ACCEPT' and refined_pose is not None:
        if pos_err > FA_XY_M or yerr > FA_YAW_RAD:
            false_accept = True
        if pos_err <= AMCL_XY_M and yerr <= AMCL_YAW_RAD:
            amcl_ready = True
        if pos_err <= POC_XY_M and yerr <= POC_YAW_RAD:
            poc_quality = True
    visual_margin = 0.0
    if len(retr.candidates) >= 2:
        visual_margin = retr.candidates[0].score - retr.candidates[1].score
    stage['e2e'] = time.monotonic() - t0
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
        'laser_refined_pose': refined_pose,
        'position_error': pos_err,
        'yaw_error': yerr,
        'laser_score': laser_score,
        'top1_top2_margin': dec.cluster_margin,
        'visual_top1_top2_margin': visual_margin,
        'decision': decision,
        'reason': reason,
        'false_accept': false_accept,
        'poc_quality_ok': poc_quality,
        'amcl_convergent_range': amcl_ready,
        'apply_initial_pose': False,
        'runtime': stage['e2e'],
        'stage_timings': stage,
        'best': best,
        'cluster_accept': decision_to_dict(dec),
        'accept_policy': 'pose_cluster_v1',
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
    poc_n = sum(1 for t in trials if t.get('poc_quality_ok'))
    xy = [t['position_error'] for t in trials if t['decision'] == 'ACCEPT' and t['position_error'] is not None]
    yaw = [t['yaw_error'] for t in trials if t['decision'] == 'ACCEPT' and t['yaw_error'] is not None]
    rt = [t['runtime'] for t in trials]

    def pct(arr, p):
        if not arr:
            return None
        return float(np.percentile(arr, p))

    return {
        'n': n,
        'ACCEPT': acc,
        'UNKNOWN': unk,
        'False_Accept': fa,
        'Correct_Accept': correct,
        'Correct_Accept_rate': correct / n if n else 0.0,
        'UNKNOWN_rate': unk / n if n else 0.0,
        'AMCL_range_accepts': amcl_n,
        'poc_quality_accepts': poc_n,
        'xy_error_accept_mean': float(np.mean(xy)) if xy else None,
        'xy_error_accept_max': float(np.max(xy)) if xy else None,
        'xy_error_accept_p50': pct(xy, 50),
        'xy_error_accept_p95': pct(xy, 95),
        'yaw_error_accept_mean': float(np.mean(yaw)) if yaw else None,
        'yaw_error_accept_max': float(np.max(yaw)) if yaw else None,
        'yaw_error_accept_p50': pct(yaw, 50),
        'yaw_error_accept_p95': pct(yaw, 95),
        'runtime_mean': float(np.mean(rt)) if rt else None,
        'runtime_max': float(np.max(rt)) if rt else None,
        'runtime_p50': pct(rt, 50),
        'runtime_p95': pct(rt, 95),
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
        'accept_policy': 'pose_cluster_v1',
        'cluster_xy_m': CLUSTER_XY_M,
        'cluster_yaw_rad': CLUSTER_YAW_RAD,
        'cluster_min_score_margin': CLUSTER_MIN_MARGIN,
        'fa_xy_m': FA_XY_M,
        'fa_yaw_rad': FA_YAW_RAD,
        'poc_xy_m': POC_XY_M,
        'poc_yaw_rad': POC_YAW_RAD,
        'amcl_xy_m': AMCL_XY_M,
        'amcl_yaw_rad': AMCL_YAW_RAD,
    }
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(report, indent=2), encoding='utf-8')
    print(json.dumps({'stats': stats, 'stopped_fa': stopped_fa, 'out': str(out)}, indent=2))


if __name__ == '__main__':
    main()
