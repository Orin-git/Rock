#!/usr/bin/env python3
"""Phase2C-C4A.4 physical P1 negative.

Robot must already be off the charger. Injects charger_prior_available=true
on the existing soft-prior topic (dev-only publisher; production logic unchanged)
and runs /xw/boot/run. Expects P1 laser REJECT. Does not change 0.38.
"""

from __future__ import annotations

import json
import math
import time
from pathlib import Path

import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Int8, String
from std_srvs.srv import Trigger

from xw_interfaces.msg import PowerState
from xw_phase2c.laser_prior_verify import verify_prior_in_window, verify_pose_with_laser
from xw_global_reloc.laser_verify import DistanceField, prepare_scan

OUT = Path('/ros2_ws/bench/phase2c_c4a4_p1_safety_2026-09-08')
CHARGER = (1.8663955491712294, -0.05958837147746455, -3.1286646850836126)
THR = 0.38

_LATCH = QoSProfile(
    depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE
)
_AMCL = QoSProfile(
    depth=1,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
)
_MAP = QoSProfile(
    depth=1,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
)


def yaw_of(q) -> float:
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


def account(field, scan, x, y, yaw) -> dict:
    from xw_global_reloc.laser_verify import score_scan_at_pose

    ranges = list(scan.ranges)
    total = len(ranges)
    prep = prepare_scan(scan, beam_stride=6)
    sc = score_scan_at_pose(field, scan, x, y, yaw, min_laser_score=THR, prepared=prep)
    lyaw = yaw + math.pi
    lc, ls = math.cos(lyaw), math.sin(lyaw)
    mx = x + lc * prep.bx - ls * prep.by
    my = y + ls * prep.bx + lc * prep.by
    in_map = field.in_map_mask(mx, my)
    in_n = int(in_map.sum()) if prep.n_valid else 0
    matched = 0
    if in_n:
        d = field.sample_dist_batch(mx[in_map], my[in_map])
        matched = int((d <= 0.25).sum())
    nan_inf = sum(1 for r in ranges if not math.isfinite(r))
    return {
        'total_beams': total,
        'nan_inf_beams': nan_inf,
        'valid_beams': int(prep.n_valid),
        'in_map_beams': in_n,
        'out_of_map_beams': int(prep.n_valid - in_n),
        'matched_beams': matched,
        'matched_ratio': float(sc.matched_ratio),
        'mean_dist': float(sc.mean_dist),
        'laser_score': float(sc.laser_score),
        'reason': sc.reason,
        'accepted': bool(sc.accepted),
        'coverage_gate': 'in_map_beams >= 20',
        'coverage_pass': in_n >= 20,
    }


class Neg(Node):
    def __init__(self) -> None:
        super().__init__('c4a4_p1_negative')
        self.scan = None
        self.map = None
        self.amcl = None
        self.power = None
        self.loc = None
        self.prior = None
        self.result = None
        self.create_subscription(LaserScan, '/scan', lambda m: setattr(self, 'scan', m), 10)
        self.create_subscription(OccupancyGrid, '/map', lambda m: setattr(self, 'map', m), _MAP)
        from geometry_msgs.msg import PoseWithCovarianceStamped
        self.create_subscription(
            PoseWithCovarianceStamped, 'amcl_pose', lambda m: setattr(self, 'amcl', m), _AMCL
        )
        self.create_subscription(PowerState, '/xw/power', lambda m: setattr(self, 'power', m), 10)
        self.create_subscription(Int8, '/xw/localization_status', lambda m: setattr(self, 'loc', int(m.data)), _LATCH)
        self.create_subscription(
            Bool, '/xw/localization/charger_prior_available', lambda m: setattr(self, 'prior', bool(m.data)), _LATCH
        )
        self.create_subscription(String, '/xw/boot/result', self._on_result, 10)
        self.prior_pub = self.create_publisher(Bool, '/xw/localization/charger_prior_available', _LATCH)
        self.cli = self.create_client(Trigger, '/xw/boot/run')

    def _on_result(self, msg: String) -> None:
        self.result = msg.data

    def spin_sec(self, sec: float) -> None:
        end = time.monotonic() + sec
        while time.monotonic() < end and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)

    def inject_prior(self) -> None:
        self.prior_pub.publish(Bool(data=True))


def main() -> int:
    OUT.mkdir(parents=True, exist_ok=True)
    rclpy.init()
    node = Neg()
    t0 = time.monotonic()
    while time.monotonic() - t0 < 20 and (node.scan is None or node.map is None or node.power is None or node.amcl is None):
        node.spin_sec(0.1)
    if node.scan is None or node.map is None or node.power is None or node.amcl is None:
        print(json.dumps({'error': 'missing_sensors', 'scan': node.scan is not None, 'map': node.map is not None}))
        node.destroy_node()
        rclpy.shutdown()
        return 2

    pose = node.amcl.pose.pose
    amcl = [float(pose.position.x), float(pose.position.y), float(yaw_of(pose.orientation))]
    dist = math.hypot(amcl[0] - CHARGER[0], amcl[1] - CHARGER[1])
    charging = bool(node.power.charging)
    docked = bool(node.power.docked)
    off_dock = (not charging) and (not docked) and dist > 1.0
    field = DistanceField(node.map)
    raw = account(field, node.scan, *CHARGER)
    exact = verify_pose_with_laser(CHARGER, node.scan, node.map, min_score=THR, field=field)
    window = verify_prior_in_window(CHARGER, node.scan, node.map, min_score=THR, field=field)

    # Keep the injected prior latched while the cascade runs.
    for _ in range(5):
        node.inject_prior()
        node.spin_sec(0.1)

    boot = {'error': 'not_called'}
    pre_reject = (not window.get('ok')) and float(window.get('laser_score') or 0.0) < THR
    if not off_dock:
        boot = {'error': 'refused_not_off_dock', 'dist_m': dist, 'charging': charging, 'docked': docked}
    elif not pre_reject:
        boot = {
            'error': 'refused_seed_pre_score_accept',
            'laser_score': window.get('laser_score'),
            'note': 'would have been a P1 false accept; cascade not seeded',
        }
    else:
        if not node.cli.wait_for_service(timeout_sec=15.0):
            boot = {'error': 'boot_unavailable'}
        else:
            node.result = None
            node.inject_prior()
            fut = node.cli.call_async(Trigger.Request())
            t1 = time.monotonic()
            while time.monotonic() - t1 < 180 and rclpy.ok():
                node.inject_prior()
                node.spin_sec(0.1)
                if fut.done() and node.result:
                    break
            parsed = {}
            if node.result:
                try:
                    parsed = json.loads(node.result)
                except json.JSONDecodeError:
                    parsed = {'raw': node.result}
            boot = {
                'svc': {'ok': bool(fut.done() and fut.result() and fut.result().success), 'msg': getattr(fut.result(), 'message', None) if fut.done() and fut.result() else None},
                'final': parsed.get('final'),
                'selected_path': parsed.get('selected_path'),
                'stages': parsed.get('stages'),
                'amcl_pose': parsed.get('amcl_pose'),
            }

    p1 = None
    for s in boot.get('stages') or []:
        if isinstance(s, dict) and s.get('stage') == 'P1':
            p1 = s
            break

    p1_reject = bool(
        p1
        and p1.get('result') in ('laser_reject', 'fail')
        and (p1.get('score') is None or float(p1.get('score') or 0.0) < THR)
    )
    not_selected = boot.get('selected_path') not in ('P1',)
    verdict = 'PASS' if off_dock and pre_reject and p1_reject and not_selected and boot.get('selected_path') != 'P1' else 'FAIL'
    if boot.get('error'):
        verdict = 'FAIL'

    scan_path = OUT / 'offdock_scan.json'
    scan_path.write_text(json.dumps({
        'angle_min': float(node.scan.angle_min),
        'angle_max': float(node.scan.angle_max),
        'angle_increment': float(node.scan.angle_increment),
        'range_min': float(node.scan.range_min),
        'range_max': float(node.scan.range_max),
        'frame_id': str(node.scan.header.frame_id),
        'ranges': [float(r) for r in node.scan.ranges],
    }) + '\n', encoding='utf-8')

    row = {
        'trial': 'P1_OFFDOCK_CHARGER_PRIOR',
        'physical_gt_not_charger': off_dock,
        'charging': charging,
        'docked': docked,
        'amcl_pose': amcl,
        'charger_waypoint': {'x': CHARGER[0], 'y': CHARGER[1], 'yaw': CHARGER[2]},
        'dist_to_charger_m': dist,
        'loc_status': node.loc,
        'charger_prior_available_injected': True,
        'charger_prior_seen': node.prior,
        'threshold': THR,
        'raw_exact': raw,
        'exact_verify': {
            'ok': exact.get('ok'),
            'laser_score': exact.get('laser_score'),
            'matched_ratio': exact.get('matched_ratio'),
            'valid_beams': exact.get('valid_beams'),
            'mean_dist': exact.get('mean_dist'),
            'reason': exact.get('reason'),
        },
        'window': {
            'ok': window.get('ok'),
            'refined': window.get('refined'),
            'laser_score': window.get('laser_score'),
            'exact_laser_score': window.get('exact_laser_score'),
            'pose': window.get('pose'),
            'reason': window.get('reason'),
        },
        'boot': boot,
        'p1_stage': p1,
        'verdict': verdict,
    }
    (OUT / 'live_negative.json').write_text(json.dumps(row, indent=2, default=str) + '\n', encoding='utf-8')
    print(json.dumps({
        'verdict': verdict,
        'off_dock': off_dock,
        'dist_m': dist,
        'charging': charging,
        'docked': docked,
        'raw_score': raw['laser_score'],
        'window_ok': window.get('ok'),
        'window_score': window.get('laser_score'),
        'p1': p1,
        'selected_path': boot.get('selected_path'),
        'final': boot.get('final'),
        'error': boot.get('error'),
    }, indent=2, default=str))
    node.destroy_node()
    rclpy.shutdown()
    return 0 if verdict == 'PASS' else 1


if __name__ == '__main__':
    raise SystemExit(main())
