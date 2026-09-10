#!/usr/bin/env python3
"""Phase2D-A2 offline Cases A/B/C using real Phase2A query RGB+scan+GT.

Bringup-independent. Writes Candidate only under a temp or candidate/ path;
never touches production keyframes. Laser thr remains 0.38 via shared scorer.
"""

from __future__ import annotations

import json
import math
import shutil
import tempfile
import time
from pathlib import Path

import cv2
import numpy as np
import yaml
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan

from xw_global_reloc.laser_verify import DistanceField
from xw_global_reloc.map_hash import map_pair_hash, resolve_map_files
from xw_global_reloc.phase2d.candidate_writer import CandidateWriter
from xw_global_reloc.phase2d.capture_pipeline import run_capture_pipeline
from xw_global_reloc.phase2d.config_loader import load_phase2d_config, legacy_production_paths
from xw_global_reloc.phase2d.pose_quality_gate import PoseGateInput

MAP_YAML = Path('/ros2_ws/maps/vp.yaml')
QUERIES = Path('/ros2_ws/bench/phase2a_poc_v1_2026-09-07/queries_v1')
OUT = Path('/ros2_ws/bench/phase2d_a2_2026-09-10')


def load_map(yaml_path: Path):
    meta = yaml.safe_load(yaml_path.read_text(encoding='utf-8'))
    img = yaml_path.parent / meta['image']
    pgm = cv2.imread(str(img), cv2.IMREAD_UNCHANGED)
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
    occupied = int(meta.get('occupied_thresh', 0.65) * 100)
    free = int(meta.get('free_thresh', 0.25) * 100)
    data = np.full((h, w), -1, dtype=np.int8)
    data[img_u8 >= occupied] = 100
    data[img_u8 <= free] = 0
    grid.data = data.reshape(-1).tolist()
    return grid, DistanceField(grid)


def load_scan(path: Path) -> LaserScan:
    z = np.load(path)
    scan = LaserScan()
    scan.angle_min = float(z['angle_min'])
    scan.angle_max = float(z['angle_max'])
    scan.angle_increment = float(z['angle_increment'])
    scan.range_min = float(z['range_min'])
    scan.range_max = float(z['range_max'])
    scan.ranges = z['ranges'].astype(np.float32).tolist()
    if 'intensities' in z.files and z['intensities'].size:
        scan.intensities = z['intensities'].astype(np.float32).tolist()
    scan.header.frame_id = str(z['frame_id']) if 'frame_id' in z.files else 'laser'
    return scan


def good_pose(meta: dict, map_hash: str) -> PoseGateInput:
    p = meta['map_pose']
    q = meta.get('quality') or {}
    return PoseGateInput(
        localization_status=0,
        phase2c_state='READY',
        phase2c_loc_state='READY',
        follow_active=False,
        legacy_freeze_active=False,
        amcl_cov_xy=float(q.get('amcl_cov_xy') or 0.05),
        amcl_cov_yaw=float(q.get('amcl_cov_yaw') or 0.01),
        map_base_age_sec=0.05,
        map_odom_age_sec=0.05,
        scan_age_sec=0.05,
        speed_mps=0.0,
        yaw_rate=0.0,
        x=float(p['x']),
        y=float(p['y']),
        yaw=float(p['yaw']),
        map_name='vp',
        map_hash=map_hash,
        scan_present=True,
    )


def main() -> int:
    OUT.mkdir(parents=True, exist_ok=True)
    cfg = load_phase2d_config()
    tmp = Path(tempfile.mkdtemp(prefix='phase2d_a2_cases_'))
    cfg['candidate_root'] = str(tmp / 'candidate')
    cfg['maps_dir'] = '/ros2_ws/maps'
    writer = CandidateWriter(cfg)
    y, p = resolve_map_files(Path('/ros2_ws/maps'), 'vp')
    mhash = map_pair_hash(y, p)
    grid, field = load_map(MAP_YAML)

    # Prefer a charger / known-good query
    qid = 'kf_000002'
    qdir = QUERIES / qid
    meta = yaml.safe_load((qdir / 'gt.yaml').read_text(encoding='utf-8'))
    bgr = cv2.imread(str(qdir / 'rgb.jpg'))
    scan = load_scan(qdir / 'scan.npz')
    pose = good_pose(meta, mhash)

    legacy_before = {
        'manifest': (legacy_production_paths(cfg)['manifest']).read_bytes(),
        'index': (legacy_production_paths(cfg)['index']).read_bytes(),
        'kf_count': len(list(legacy_production_paths(cfg)['keyframes'].iterdir())),
    }

    results = {}

    # Case A — strong localization pose + real RGB
    t0 = time.monotonic()
    a = run_capture_pipeline(
        cfg=cfg,
        pose_input=pose,
        scan=scan,
        occupancy_map=grid,
        field=field,
        bgr=bgr,
        writer=writer,
        dry_run=False,
        source='manual_test',
        scan_stamp=time.time(),
    )
    results['case_a_strong_pose'] = {
        **a.to_dict(),
        'runtime_sec': time.monotonic() - t0,
        'expect': 'ACCEPTED',
        'pass': a.status == 'ACCEPTED' and a.written and (a.laser.laser_score >= 0.38 if a.laser else False),
    }

    # Case B — false-good: cheap pose gate PASS, laser FAIL.
    # Claim a map pose far from where the scan was taken so geometric
    # verify cannot bind RGB to (x,y,yaw) (score 0 / few in-map beams).
    bad = good_pose(meta, mhash)
    bad.x = float(pose.x) + 25.0
    bad.y = float(pose.y) + 25.0
    t0 = time.monotonic()
    b = run_capture_pipeline(
        cfg=cfg,
        pose_input=bad,
        scan=scan,
        occupancy_map=grid,
        field=field,
        bgr=bgr,
        writer=writer,
        dry_run=False,
        source='manual_test',
        scan_stamp=time.time(),
    )
    laser_lt = (b.laser is not None) and (float(b.laser.laser_score) < 0.38)
    results['case_b_false_good_pose'] = {
        **b.to_dict(),
        'runtime_sec': time.monotonic() - t0,
        'expect': 'REJECTED_LASER',
        'pass': b.status == 'REJECTED_LASER' and not b.written and laser_lt,
        'note': 'pose gates pass; laser_inconsistent prevents Candidate write',
    }

    # Case C — blurry image, good pose/laser
    blur = cv2.GaussianBlur(bgr, (51, 51), 0)
    t0 = time.monotonic()
    c = run_capture_pipeline(
        cfg=cfg,
        pose_input=pose,
        scan=scan,
        occupancy_map=grid,
        field=field,
        bgr=blur,
        writer=writer,
        dry_run=False,
        source='manual_test',
        scan_stamp=time.time(),
    )
    results['case_c_blurry_image'] = {
        **c.to_dict(),
        'runtime_sec': time.monotonic() - t0,
        'expect': 'REJECTED_IMAGE',
        'pass': c.status == 'REJECTED_IMAGE' and not c.written,
    }

    # Dry-run
    d = run_capture_pipeline(
        cfg=cfg,
        pose_input=pose,
        scan=scan,
        occupancy_map=grid,
        field=field,
        bgr=bgr,
        writer=writer,
        dry_run=True,
        source='manual_test',
        scan_stamp=time.time(),
    )
    results['dry_run'] = {
        **d.to_dict(),
        'pass': d.status == 'ACCEPTED' and not d.written,
    }

    legacy_after = {
        'manifest': (legacy_production_paths(cfg)['manifest']).read_bytes(),
        'index': (legacy_production_paths(cfg)['index']).read_bytes(),
        'kf_count': len(list(legacy_production_paths(cfg)['keyframes'].iterdir())),
    }
    results['legacy_37_untouched'] = {
        'pass': legacy_before == legacy_after,
        'kf_count': legacy_after['kf_count'],
        'manifest_unchanged': legacy_before['manifest'] == legacy_after['manifest'],
        'index_unchanged': legacy_before['index'] == legacy_after['index'],
    }
    results['stats'] = dict(writer.stats)
    results['all_pass'] = all(
        results[k]['pass']
        for k in (
            'case_a_strong_pose',
            'case_b_false_good_pose',
            'case_c_blurry_image',
            'dry_run',
            'legacy_37_untouched',
        )
    )

    out_path = OUT / 'offline_cases.json'
    out_path.write_text(json.dumps(results, indent=2, default=str) + '\n', encoding='utf-8')
    print(json.dumps({k: {'status': results[k].get('status'), 'pass': results[k].get('pass'), 'laser_score': results[k].get('laser_score')} for k in results if isinstance(results[k], dict) and 'pass' in results[k]}, indent=2))
    print('all_pass', results['all_pass'], 'wrote', out_path)
    shutil.rmtree(tmp, ignore_errors=True)
    return 0 if results['all_pass'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
