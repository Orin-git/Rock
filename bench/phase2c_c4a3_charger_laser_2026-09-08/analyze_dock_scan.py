#!/usr/bin/env python3
"""Offline score surface + overlays for the captured dock scan. No live TF."""
from __future__ import annotations

import json
import math
from pathlib import Path

import numpy as np

from nav_msgs.msg import OccupancyGrid, MapMetaData
from sensor_msgs.msg import LaserScan

from xw_global_reloc.laser_verify import DistanceField, prepare_scan, score_scan_at_pose

OUT = Path('/ros2_ws/bench/phase2c_c4a3_charger_laser_2026-09-08')
CHARGER = (1.8663955491712294, -0.05958837147746455, -3.1286646850836126)
CORRECT = (1.7993268507431137, 0.01590625307226233, -2.8414570739649156)
WRONG = (-8.93, 1.60, 0.0)
BOOT_D = (-9.15, 0.88, 3.0)  # control location, not this scan's true pose


def load_scan() -> LaserScan:
    raw = json.loads((OUT / 'scan.json').read_text(encoding='utf-8'))
    s = LaserScan()
    s.header.frame_id = raw['header']['frame_id']
    s.header.stamp.sec = int(raw['header']['stamp_sec'])
    s.header.stamp.nanosec = int(raw['header']['stamp_nanosec'])
    s.angle_min = float(raw['angle_min'])
    s.angle_max = float(raw['angle_max'])
    s.angle_increment = float(raw['angle_increment'])
    s.time_increment = float(raw['time_increment'])
    s.scan_time = float(raw['scan_time'])
    s.range_min = float(raw['range_min'])
    s.range_max = float(raw['range_max'])
    s.ranges = [float(r) for r in raw['ranges']]
    return s


def load_grid() -> OccupancyGrid:
    z = np.load(OUT / 'map_grid.npz')
    g = OccupancyGrid()
    g.header.frame_id = 'map'
    g.info = MapMetaData()
    g.info.resolution = float(z['resolution'])
    g.info.width = int(z['width'])
    g.info.height = int(z['height'])
    g.info.origin.position.x = float(z['origin_x'])
    g.info.origin.position.y = float(z['origin_y'])
    data = z['data'].astype(np.int16)
    # OccupancyGrid data is signed int8 in ROS; stored as int16 to keep -1.
    g.data = [int(v) for v in data.tolist()]
    return g


def project(x, y, yaw, prep, yaw_offset=math.pi, reverse_beams=False):
    bx = prep.bx.copy()
    by = prep.by.copy()
    if reverse_beams:
        bx = -bx
        by = -by
    lyaw = yaw + yaw_offset
    lc, ls = math.cos(lyaw), math.sin(lyaw)
    mx = x + lc * bx - ls * by
    my = y + ls * bx + lc * by
    return mx, my


def summarize(field, scan, pose, name, **kw):
    sc = score_scan_at_pose(field, scan, pose[0], pose[1], pose[2], **kw)
    prep = prepare_scan(scan, beam_stride=kw.get('beam_stride', 6))
    yaw_off = kw.get('yaw_offset', math.pi)
    mx, my = project(pose[0], pose[1], pose[2], prep, yaw_offset=yaw_off)
    dists = field.sample_dist_batch(mx, my)
    ix = np.floor((mx - field.origin_x) / field.resolution).astype(np.int32)
    iy = np.floor((my - field.origin_y) / field.resolution).astype(np.int32)
    oob = (ix < 0) | (iy < 0) | (ix >= field.width) | (iy >= field.height)
    finite = dists[np.isfinite(dists) & (dists < 999)]
    return {
        'name': name,
        'pose': list(pose),
        'laser_score': float(sc.laser_score),
        'matched_ratio': float(sc.matched_ratio),
        'mean_dist': float(sc.mean_dist),
        'p90_dist': float(sc.p90_dist),
        'valid_beams': int(sc.valid_beams),
        'beam_count': int(len(scan.ranges)),
        'accepted': bool(sc.accepted),
        'reason': sc.reason,
        'oob_beams': int(np.count_nonzero(oob)),
        'dist_gt_1m': int(np.count_nonzero(dists > 1.0)),
        'dist_gt_5m': int(np.count_nonzero(dists > 5.0)),
        'dist_eq_1e3': int(np.count_nonzero(dists >= 999)),
        'median_dist': float(np.median(dists)),
        'p50': float(np.percentile(dists, 50)),
        'p75': float(np.percentile(dists, 75)),
        'p95': float(np.percentile(dists, 95)),
        'mean_inlier_le_1m': float(np.mean(dists[dists <= 1.0])) if np.any(dists <= 1.0) else None,
        'frac_le_0.25': float(np.mean(dists <= 0.25)),
        'frac_le_1.0': float(np.mean(dists <= 1.0)),
    }


def crop_overlay(field, poses, out_png, title, half_m=6.0):
    import cv2

    occ = field.occ
    # visualize: occupied black, free white, unknown light gray already in occ as 0
    img = np.full(occ.shape, 230, dtype=np.uint8)
    img[occ > 0] = 30
    # world window around first pose
    cx, cy = poses[0][1][0], poses[0][1][1]
    x0, x1 = cx - half_m, cx + half_m
    y0, y1 = cy - half_m, cy + half_m
    ix0 = int((x0 - field.origin_x) / field.resolution)
    iy0 = int((y0 - field.origin_y) / field.resolution)
    ix1 = int((x1 - field.origin_x) / field.resolution)
    iy1 = int((y1 - field.origin_y) / field.resolution)
    ix0, iy0 = max(0, ix0), max(0, iy0)
    ix1, iy1 = min(field.width - 1, ix1), min(field.height - 1, iy1)
    crop = img[iy0:iy1 + 1, ix0:ix1 + 1]
    vis = cv2.cvtColor(crop, cv2.COLOR_GRAY2BGR)
    colors = {
        'charger': (0, 0, 255),
        'correct': (0, 180, 0),
        'wrong': (255, 80, 0),
        'amcl': (0, 165, 255),
    }
    scale = 2
    vis = cv2.resize(vis, (crop.shape[1] * scale, crop.shape[0] * scale), interpolation=cv2.INTER_NEAREST)

    def w2p(wx, wy):
        px = (wx - field.origin_x) / field.resolution - ix0
        py = (wy - field.origin_y) / field.resolution - iy0
        return int(px * scale), int(py * scale)

    scan = load_scan()
    prep = prepare_scan(scan, beam_stride=6)
    for name, pose in poses:
        mx, my = project(pose[0], pose[1], pose[2], prep)
        col = colors.get(name, (255, 0, 255))
        ox, oy = w2p(pose[0], pose[1])
        # laser origin == base xy (URDF xy=0); draw heading of base and of lidar (base+pi)
        cv2.circle(vis, (ox, oy), 5, col, -1)
        lyaw = pose[2] + math.pi
        lx, ly = w2p(pose[0] + 0.4 * math.cos(lyaw), pose[1] + 0.4 * math.sin(lyaw))
        cv2.arrowedLine(vis, (ox, oy), (lx, ly), col, 1, tipLength=0.3)
        step = max(1, len(mx) // 180)
        for i in range(0, len(mx), step):
            px, py = w2p(float(mx[i]), float(my[i]))
            if 0 <= px < vis.shape[1] and 0 <= py < vis.shape[0]:
                cv2.circle(vis, (px, py), 1, col, -1)
    # flip vertically so +y is up for human reading (occupancy row 0 is origin/min y)
    vis = cv2.flip(vis, 0)
    cv2.putText(vis, title, (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (20, 20, 20), 1, cv2.LINE_AA)
    cv2.imwrite(str(out_png), vis)
    return str(out_png)


def main() -> None:
    scan = load_scan()
    grid = load_grid()
    field = DistanceField(grid)
    ranges = np.asarray(scan.ranges, dtype=np.float64)
    finite = np.isfinite(ranges)
    in_window = finite & (ranges >= scan.range_min) & (ranges <= scan.range_max)
    near_max = in_window & (ranges >= scan.range_max - 0.05)
    near_min = in_window & (ranges <= scan.range_min + 0.02)
    zeros = finite & (ranges == 0)
    scan_stats = {
        'frame_id': scan.header.frame_id,
        'stamp': scan.header.stamp.sec + scan.header.stamp.nanosec * 1e-9,
        'angle_min': scan.angle_min,
        'angle_max': scan.angle_max,
        'angle_increment': scan.angle_increment,
        'angle_span': scan.angle_max - scan.angle_min,
        'n': int(ranges.size),
        'range_min': scan.range_min,
        'range_max': scan.range_max,
        'finite': int(np.count_nonzero(finite)),
        'nan_or_inf': int(np.count_nonzero(~finite)),
        'in_range_window': int(np.count_nonzero(in_window)),
        'near_range_max': int(np.count_nonzero(near_max)),
        'near_range_min': int(np.count_nonzero(near_min)),
        'zero': int(np.count_nonzero(zeros)),
        'median_valid_m': float(np.median(ranges[in_window])) if np.any(in_window) else None,
        'p10_valid_m': float(np.percentile(ranges[in_window], 10)) if np.any(in_window) else None,
        'p90_valid_m': float(np.percentile(ranges[in_window], 90)) if np.any(in_window) else None,
        'units_note': 'ranges treated as meters; angle_increment rad; no mm scaling in laser_verify',
    }

    meta = json.loads((OUT / 'capture_meta.json').read_text(encoding='utf-8'))
    amcl = tuple(meta['amcl_pose_at_capture'])

    rows = []
    named = [
        ('charger_waypoint', CHARGER),
        ('pre_scramble_correct', CORRECT),
        ('amcl_at_capture', amcl),
        ('correct_px0.10', (CORRECT[0] + 0.10, CORRECT[1], CORRECT[2])),
        ('correct_mx0.10', (CORRECT[0] - 0.10, CORRECT[1], CORRECT[2])),
        ('correct_py0.10', (CORRECT[0], CORRECT[1] + 0.10, CORRECT[2])),
        ('correct_my0.10', (CORRECT[0], CORRECT[1] - 0.10, CORRECT[2])),
        ('correct_px0.20', (CORRECT[0] + 0.20, CORRECT[1], CORRECT[2])),
        ('correct_mx0.20', (CORRECT[0] - 0.20, CORRECT[1], CORRECT[2])),
        ('correct_py0.20', (CORRECT[0], CORRECT[1] + 0.20, CORRECT[2])),
        ('correct_my0.20', (CORRECT[0], CORRECT[1] - 0.20, CORRECT[2])),
        ('correct_yaw_p5deg', (CORRECT[0], CORRECT[1], CORRECT[2] + math.radians(5))),
        ('correct_yaw_m5deg', (CORRECT[0], CORRECT[1], CORRECT[2] - math.radians(5))),
        ('correct_yaw_p10deg', (CORRECT[0], CORRECT[1], CORRECT[2] + math.radians(10))),
        ('correct_yaw_m10deg', (CORRECT[0], CORRECT[1], CORRECT[2] - math.radians(10))),
        ('wrong_corridor', WRONG),
        ('boot_d_control_pose_on_this_scan', BOOT_D),
    ]
    for name, pose in named:
        rows.append(summarize(field, scan, pose, name))

    # local surface: dx, dy, dyaw
    xs = np.linspace(CORRECT[0] - 0.4, CORRECT[0] + 0.4, 17)
    ys = np.linspace(CORRECT[1] - 0.4, CORRECT[1] + 0.4, 17)
    yaws = np.array([CORRECT[2] + math.radians(d) for d in (-15, -10, -5, 0, 5, 10, 15)])
    surface = []
    best = None
    for yaw in yaws:
        for y in ys:
            for x in xs:
                sc = score_scan_at_pose(field, scan, float(x), float(y), float(yaw))
                rec = {
                    'x': float(x),
                    'y': float(y),
                    'yaw': float(yaw),
                    'laser_score': float(sc.laser_score),
                    'matched_ratio': float(sc.matched_ratio),
                    'mean_dist': float(sc.mean_dist),
                    'valid_beams': int(sc.valid_beams),
                }
                surface.append(rec)
                if best is None or rec['laser_score'] > best['laser_score'] or (
                    rec['laser_score'] == best['laser_score'] and rec['mean_dist'] < best['mean_dist']
                ):
                    best = rec

    # extrinsic ablation on the correct pose only — diagnostic, not a production change
    prep = prepare_scan(scan, beam_stride=6)
    ablations = []
    for label, off, rev in (
        ('hardcoded_yaw_plus_pi', math.pi, False),
        ('yaw_plus_0', 0.0, False),
        ('yaw_minus_pi', -math.pi, False),
        ('yaw_plus_pi_beams_reversed', math.pi, True),
        ('yaw_plus_0_beams_reversed', 0.0, True),
    ):
        mx, my = project(CORRECT[0], CORRECT[1], CORRECT[2], prep, yaw_offset=off, reverse_beams=rev)
        dists = field.sample_dist_batch(mx, my)
        matched = float(np.mean(dists <= 0.25))
        mean_d = float(np.mean(dists))
        score = matched * float(np.clip(1.0 - mean_d / 1.0, 0.0, 1.0))
        ablations.append({
            'label': label,
            'matched_ratio': matched,
            'mean_dist': mean_d,
            'median_dist': float(np.median(dists)),
            'score_if_formula': score,
            'oob_or_1e3': int(np.count_nonzero(dists >= 999)),
        })

    # beam dump at correct pose
    mx, my = project(CORRECT[0], CORRECT[1], CORRECT[2], prep)
    dists = field.sample_dist_batch(mx, my)
    # angle of each prepared beam in lidar frame
    # reconstruct from prepare_scan indexing
    ranges = np.asarray(scan.ranges, dtype=np.float64)
    idx = np.arange(0, len(ranges), 6, dtype=np.int32)
    r = ranges[idx]
    valid = np.isfinite(r) & (r >= scan.range_min) & (r <= scan.range_max)
    idx = idx[valid]
    r = r[valid]
    ang = float(scan.angle_min) + idx.astype(np.float64) * float(scan.angle_increment)
    order = np.argsort(dists)[::-1]
    worst = []
    for i in order[:15]:
        worst.append({
            'beam_index': int(idx[i]),
            'angle_rad': float(ang[i]),
            'angle_deg': float(math.degrees(ang[i])),
            'range_m': float(r[i]),
            'map_xy': [float(mx[i]), float(my[i])],
            'dist_to_occ': float(dists[i]),
        })

    # histogram
    bins = [0, 0.25, 0.5, 1.0, 2.0, 5.0, 10.0, 50.0, 1000.0, 1e9]
    hist = []
    for a, b in zip(bins[:-1], bins[1:]):
        hist.append({'lo': a, 'hi': b, 'n': int(np.count_nonzero((dists >= a) & (dists < b)))})

    # mean contribution
    total = float(np.sum(dists))
    contrib = {
        'n': int(dists.size),
        'sum_dist': total,
        'sum_from_gt_1m': float(np.sum(dists[dists > 1.0])),
        'sum_from_gt_5m': float(np.sum(dists[dists > 5.0])),
        'sum_from_1e3': float(np.sum(dists[dists >= 999])),
        'frac_mean_from_gt_1m': float(np.sum(dists[dists > 1.0]) / total) if total else None,
    }

    overlay_dir = OUT / 'overlays'
    overlay_dir.mkdir(exist_ok=True)
    overlays = {
        'charger': crop_overlay(field, [('charger', CHARGER)], overlay_dir / 'overlay_charger.png', 'charger waypoint', 4.0),
        'correct': crop_overlay(field, [('correct', CORRECT)], overlay_dir / 'overlay_correct.png', 'pre-scramble correct', 4.0),
        'wrong': crop_overlay(field, [('wrong', WRONG)], overlay_dir / 'overlay_wrong_corridor.png', 'wrong corridor', 6.0),
        'compare': crop_overlay(
            field,
            [('correct', CORRECT), ('charger', CHARGER)],
            overlay_dir / 'overlay_correct_vs_charger.png',
            'green=correct red=charger',
            4.0,
        ),
    }

    # score heatmap at correct yaw (matched_ratio and mean_dist) as images
    import cv2
    yaw0 = CORRECT[2]
    heat_score = np.zeros((len(ys), len(xs)), dtype=np.float64)
    heat_match = np.zeros_like(heat_score)
    heat_mean = np.zeros_like(heat_score)
    for rec in surface:
        if abs(rec['yaw'] - yaw0) > 1e-9:
            continue
        ix = int(round((rec['x'] - xs[0]) / (xs[1] - xs[0])))
        iy = int(round((rec['y'] - ys[0]) / (ys[1] - ys[0])))
        if 0 <= iy < heat_score.shape[0] and 0 <= ix < heat_score.shape[1]:
            heat_score[iy, ix] = rec['laser_score']
            heat_match[iy, ix] = rec['matched_ratio']
            heat_mean[iy, ix] = rec['mean_dist']

    def save_heat(arr, path, vmax=None, cmap_note=''):
        a = arr.copy()
        if vmax is None:
            vmax = max(float(np.max(a)), 1e-6)
        norm = np.clip(a / vmax, 0, 1)
        img = (norm * 255).astype(np.uint8)
        img = cv2.applyColorMap(img, cv2.COLORMAP_VIRIDIS)
        img = cv2.flip(img, 0)
        img = cv2.resize(img, (img.shape[1] * 24, img.shape[0] * 24), interpolation=cv2.INTER_NEAREST)
        cv2.imwrite(str(path), img)

    save_heat(heat_score, overlay_dir / 'surface_laser_score.png', vmax=0.38)
    save_heat(heat_match, overlay_dir / 'surface_matched_ratio.png', vmax=1.0)
    save_heat(heat_mean, overlay_dir / 'surface_mean_dist.png', vmax=8.0)

    report = {
        'scan_stats': scan_stats,
        'map': {
            'resolution_m_per_cell': field.resolution,
            'origin_x': field.origin_x,
            'origin_y': field.origin_y,
            'width': field.width,
            'height': field.height,
            'occupied_thresh_used': 50,
            'yaml_occupied_thresh': 0.65,
            'world_to_pixel': 'ix = floor((x-origin_x)/res); iy = floor((y-origin_y)/res); row 0 is min y',
            'pgm_note': 'scoring uses live OccupancyGrid bytes, not raw PGM row order',
        },
        'extrinsic': meta.get('tf_at_capture', {}).get('base_link_to_lidar_link'),
        'score_rows': rows,
        'best_in_local_box': best,
        'n_surface': len(surface),
        'max_laser_score_in_box': max(r['laser_score'] for r in surface),
        'max_matched_ratio_in_box': max(r['matched_ratio'] for r in surface),
        'min_mean_dist_in_box': min(r['mean_dist'] for r in surface),
        'ablations': ablations,
        'correct_pose_dist_hist': hist,
        'correct_pose_mean_contrib': contrib,
        'worst_beams_at_correct': worst,
        'overlays': overlays,
        'formula': 'score = matched_ratio * clip(1 - mean_dist/(0.25*4), 0, 1); mean_dist>=1 => score 0',
    }
    (OUT / 'score_matrix.json').write_text(json.dumps(report, indent=2), encoding='utf-8')
    print(json.dumps({
        'scan_stats': scan_stats,
        'rows_brief': [{k: r[k] for k in ('name', 'laser_score', 'matched_ratio', 'mean_dist', 'median_dist', 'oob_beams', 'dist_gt_5m', 'dist_eq_1e3')} for r in rows],
        'best': best,
        'max_score': report['max_laser_score_in_box'],
        'min_mean': report['min_mean_dist_in_box'],
        'hist': hist,
        'contrib': contrib,
        'ablations': ablations,
        'worst3': worst[:5],
    }, indent=2))


if __name__ == '__main__':
    main()
