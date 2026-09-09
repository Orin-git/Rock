#!/usr/bin/env python3
"""Retry manual-carry LOST induction. No /initialpose. No nav goal."""

from __future__ import annotations

import importlib.util
import json
import time
from pathlib import Path

import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan

from xw_phase2c.laser_prior_verify import verify_prior_in_window

OUT = Path('/ros2_ws/bench/phase2c_c4b33_2026-09-09')
POSE_A = (-3.994337181152926, -1.0782251538151457, 3.0716712881674635)


def load_cascade():
    path = Path('/ros2_ws/src/xw_phase2c/scripts/phase2c_c4b32_cascade.py')
    spec = importlib.util.spec_from_file_location('c4b32_cascade', path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def main() -> None:
    C = load_cascade()
    rclpy.init()
    n = C.Watch()
    map_msg = {'m': None}
    scan_msg = {'s': None}

    def on_map(msg):
        map_msg['m'] = msg

    def on_scan(msg):
        scan_msg['s'] = msg

    map_qos = QoSProfile(
        depth=1,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
        reliability=ReliabilityPolicy.RELIABLE,
        history=HistoryPolicy.KEEP_LAST,
    )
    n.create_subscription(OccupancyGrid, 'map', on_map, map_qos)
    n.create_subscription(LaserScan, '/scan', on_scan, 10)
    t0 = time.monotonic()
    while time.monotonic() - t0 < 8.0 and (map_msg['m'] is None or scan_msg['s'] is None):
        n.spin(0.1)
    laser = None
    if map_msg['m'] is not None and scan_msg['s'] is not None:
        laser = verify_prior_in_window(POSE_A, scan_msg['s'], map_msg['m'])
    report = {
        'pose_A': {'x': POSE_A[0], 'y': POSE_A[1], 'yaw': POSE_A[2]},
        'pre_laser': laser,
        'snap0': n.snap(),
        'last_good': C.read_pose_file(),
    }
    if not laser or not isinstance(laser.get('laser_score'), (int, float)) or float(laser['laser_score']) >= 0.38:
        report['pass'] = False
        report['reason'] = 'live_scan_still_matches_A_or_no_scan'
        (OUT / 'manual_carry_retry.json').write_text(json.dumps(report, indent=2, default=str), encoding='utf-8')
        print(json.dumps({'pass': False, 'reason': report['reason'], 'laser': None if not laser else laser.get('laser_score')}), flush=True)
        n.destroy_node()
        rclpy.shutdown()
        return

    nav = n.set_mode(2, json.dumps({'map_name': 'vp'}))
    n.wait_until(lambda: n.nav_en is True, 20.0)
    n.spin(0.5)
    n.lost_results.clear()
    n.initialposes.clear()
    n.cmd_samples.clear()
    n.goals.clear()
    n.owners.clear()
    n.loc_states.clear()
    t_induce = time.time()
    n.inject_status(3, 6.0)
    n.wait_until(lambda: n.loc in ('LOST', 'RECOVERING', 'NEED_OPERATOR') or bool(n.lost_results), 20.0)
    n.wait_until(lambda: bool(n.lost_results) and n.lost_results[-1].get('final') in ('READY', 'UNKNOWN'), 180.0)
    n.spin(1.0)
    lr = n.lost_results[-1] if n.lost_results else {}
    report.update(C._cascade(lr))
    report.update({
        'nav': nav,
        'lost_result': lr,
        'loc': n.loc,
        'blocked': n.blocked,
        'saw_lost': any(s.get('loc') == 'LOST' for s in n.loc_states),
        'initialpose_count': len(n.initialposes),
        'initialposes': n.initialposes[-8:],
        'owners': C._owners_ok(n.owners),
        'motion_count': len([s for s in n.cmd_samples if s.get('t', 0) >= t_induce]),
        'amcl_end': n.amcl,
        'nav_en': n.nav_en,
        'follow_en': n.follow_en,
        'recharge_en': n.recharge_en,
    })
    (OUT / 'manual_carry_retry.json').write_text(json.dumps(report, indent=2, default=str), encoding='utf-8')
    print(json.dumps({
        'pass_fields': True,
        'r1': report.get('r1_code'),
        'r2': report.get('r2_code'),
        'r3': report.get('r3_code'),
        'final': report.get('final'),
        'laser_pre': laser.get('laser_score'),
        'loc': n.loc,
    }), flush=True)
    n.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
