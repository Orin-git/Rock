#!/usr/bin/env python3
"""Same-moment AMCL vs P3 dry-run. Does not publish /initialpose. Threshold unchanged."""

from __future__ import annotations

import json
import math
import os
import time
from pathlib import Path
from typing import Any, Dict, Optional

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int8
from tf2_ros import Buffer, TransformException, TransformListener

from xw_interfaces.srv import Relocalize
from xw_phase2c.last_good_pose import compute_map_hash, yaw_from_quat
from xw_phase2c.laser_prior_verify import MIN_LASER_SCORE, verify_pose_with_laser

OUT = Path(os.environ.get('C4B22_OUT', '/ros2_ws/bench/phase2c_c4b22_2026-09-09'))
MAP = os.environ.get('C4B22_MAP', 'vp')


def _stamp(msg) -> float:
    return float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9


class Cap(Node):
    def __init__(self) -> None:
        super().__init__('c4b22_mismatch')
        latch = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
        )
        self.amcl: Optional[PoseWithCovarianceStamped] = None
        self.scan: Optional[LaserScan] = None
        self.map: Optional[OccupancyGrid] = None
        self.status: Optional[int] = None
        self.create_subscription(PoseWithCovarianceStamped, 'amcl_pose', self._amcl, latch)
        self.create_subscription(Int8, '/xw/localization_status', self._st, latch)
        scan_qos = QoSProfile(
            depth=5,
            durability=DurabilityPolicy.VOLATILE,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
        )
        self.create_subscription(LaserScan, '/scan', self._scan, scan_qos)
        self.create_subscription(LaserScan, '/scan', self._scan, 10)
        self.create_subscription(OccupancyGrid, '/map', self._map, latch)
        self.tf = Buffer()
        self._tl = TransformListener(self.tf, self, spin_thread=False)
        self.reloc = self.create_client(Relocalize, '/xw/relocalize')

    def _amcl(self, msg: PoseWithCovarianceStamped) -> None:
        self.amcl = msg

    def _st(self, msg: Int8) -> None:
        self.status = int(msg.data)

    def _scan(self, msg: LaserScan) -> None:
        self.scan = msg

    def _map(self, msg: OccupancyGrid) -> None:
        self.map = msg

    def spin(self, sec: float) -> None:
        end = time.monotonic() + sec
        while time.monotonic() < end and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)

    def tf_pose(self, parent: str, child: str) -> Dict[str, Any]:
        try:
            tf = self.tf.lookup_transform(parent, child, rclpy.time.Time())
        except TransformException as exc:
            return {'ok': False, 'error': str(exc)}
        t = tf.transform.translation
        q = tf.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        age = None
        if not (tf.header.stamp.sec == 0 and tf.header.stamp.nanosec == 0):
            age = (self.get_clock().now() - rclpy.time.Time.from_msg(tf.header.stamp)).nanoseconds * 1e-9
        return {
            'ok': True,
            'x': float(t.x),
            'y': float(t.y),
            'yaw': float(yaw),
            'stamp': _stamp(tf),
            'age_sec': age,
        }


def main() -> None:
    OUT.mkdir(parents=True, exist_ok=True)
    rclpy.init()
    n = Cap()
    try:
        n.spin(8.0)
        if n.amcl is None or n.scan is None or n.map is None:
            raise SystemExit(f'missing amcl={n.amcl is not None} scan={n.scan is not None} map={n.map is not None}')
        p = n.amcl.pose.pose
        c = n.amcl.pose.covariance
        pose = (float(p.position.x), float(p.position.y), yaw_from_quat(p.orientation))
        now = time.time()
        scan_stamp = _stamp(n.scan)
        laser = verify_pose_with_laser(pose, n.scan, n.map, min_score=MIN_LASER_SCORE)
        amcl = {
            'x': pose[0],
            'y': pose[1],
            'yaw': pose[2],
            'cov_xy': max(float(c[0]), float(c[7])),
            'cov_yaw': float(c[35]),
            'stamp': _stamp(n.amcl),
            'age_sec': now - _stamp(n.amcl),
        }
        report: Dict[str, Any] = {
            'captured_wall': now,
            'loc_status': n.status,
            'amcl': amcl,
            'map_odom': n.tf_pose('map', 'odom'),
            'map_base': n.tf_pose('map', 'base_link'),
            'map_hash': compute_map_hash('/ros2_ws/maps', MAP),
            'scan_stamp': scan_stamp,
            'scan_age_sec': now - scan_stamp,
            'scan_frame': n.scan.header.frame_id,
            'map_frame': n.map.header.frame_id,
            'amcl_laser': laser,
            'threshold': MIN_LASER_SCORE,
        }
        (OUT / 'mismatch_scan_stamp.txt').write_text(f'{scan_stamp}\n', encoding='utf-8')
        if not n.reloc.wait_for_service(timeout_sec=8.0):
            report['p3'] = {'ok': False, 'error': 'relocalize_unavailable'}
        else:
            req = Relocalize.Request()
            req.map_name = MAP
            req.force_visual = True
            req.max_candidates = 10
            req.apply_initial_pose = False
            req.allow_motion = False
            fut = n.reloc.call_async(req)
            t0 = time.monotonic()
            while time.monotonic() - t0 < 70.0 and not fut.done():
                n.spin(0.2)
            if not fut.done() or fut.result() is None:
                report['p3'] = {'ok': False, 'error': 'timeout'}
            else:
                res = fut.result()
                pp = res.pose.pose.pose
                cand = (float(pp.position.x), float(pp.position.y), yaw_from_quat(pp.orientation))
                dxy = math.hypot(cand[0] - pose[0], cand[1] - pose[1])
                dyaw = abs(math.atan2(math.sin(cand[2] - pose[2]), math.cos(cand[2] - pose[2])))
                diag = {}
                try:
                    diag = json.loads(res.diagnostics_json or '{}')
                except json.JSONDecodeError:
                    diag = {'raw': res.diagnostics_json}
                report['p3'] = {
                    'ok': bool(res.success),
                    'result_code': int(res.result_code),
                    'laser_score': float(res.laser_score),
                    'visual_score': float(res.visual_score),
                    'candidate': {'x': cand[0], 'y': cand[1], 'yaw': cand[2]},
                    'delta_xy_m': dxy,
                    'delta_yaw_rad': dyaw,
                    'apply_initial_pose': False,
                    'decision': diag.get('decision'),
                    'reason': diag.get('reason'),
                    'keyframe_id': diag.get('keyframe_id'),
                }
        amcl_score = float(laser.get('laser_score') or 0.0)
        p3_score = float((report.get('p3') or {}).get('laser_score') or 0.0)
        dxy = float((report.get('p3') or {}).get('delta_xy_m') or 999.0)
        if amcl_score < MIN_LASER_SCORE and p3_score >= MIN_LASER_SCORE and dxy > 0.35:
            case = 'A'
        elif amcl_score < MIN_LASER_SCORE and p3_score < MIN_LASER_SCORE:
            case = 'B'
        elif p3_score >= MIN_LASER_SCORE and dxy <= 0.35 and amcl_score < MIN_LASER_SCORE:
            case = 'C'
        elif amcl_score >= MIN_LASER_SCORE:
            case = 'CONSISTENT'
        else:
            case = 'OTHER'
        report['case'] = case
        (OUT / 'mismatch.json').write_text(json.dumps(report, indent=2, default=str), encoding='utf-8')
        print(json.dumps({
            'case': case,
            'loc_status': n.status,
            'amcl': amcl,
            'amcl_laser': laser.get('laser_score'),
            'matched_ratio': laser.get('matched_ratio'),
            'valid_beams': laser.get('valid_beams'),
            'p3': report.get('p3'),
        }, indent=2, default=str))
    finally:
        n.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
