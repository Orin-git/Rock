#!/usr/bin/env python3
"""Capture one dock /scan + map/TF/poses, then score that SAME scan under two AMCL states.

Does not lower the laser threshold. Does not call the boot cascade.
Publishes one wrong corridor /initialpose only for the independence probe, then restores
the pose recorded at capture.
"""
from __future__ import annotations

import hashlib
import json
import math
import os
import shutil
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, Quaternion
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from tf2_ros import Buffer, TransformListener

OUT = Path('/ros2_ws/bench/phase2c_c4a3_charger_laser_2026-09-08')
MAPS = Path('/ros2_ws/maps')
CHARGER = (1.8663955491712294, -0.05958837147746455, -3.1286646850836126)
PRE_SCRAMBLE = (1.7993268507431137, 0.01590625307226233, -2.8414570739649156)
WRONG_CORRIDOR = (-8.93, 1.60, 0.0)
LATCH = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)


def yaw_of(q: Quaternion) -> float:
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


def quat_from_yaw(yaw: float) -> Quaternion:
    q = Quaternion()
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q


def pose_of(msg: PoseWithCovarianceStamped):
    p = msg.pose.pose
    return [float(p.position.x), float(p.position.y), float(yaw_of(p.orientation))]


def tf_dict(buf: Buffer, parent: str, child: str):
    try:
        t = buf.lookup_transform(parent, child, rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=0.4))
    except Exception as e:  # noqa: BLE001
        return {'ok': False, 'error': str(e), 'parent': parent, 'child': child}
    tr = t.transform.translation
    q = t.transform.rotation
    return {
        'ok': True,
        'parent': parent,
        'child': child,
        'stamp_sec': t.header.stamp.sec + t.header.stamp.nanosec * 1e-9,
        'xyz': [float(tr.x), float(tr.y), float(tr.z)],
        'quat_xyzw': [float(q.x), float(q.y), float(q.z), float(q.w)],
        'yaw': float(yaw_of(q)),
    }


def scan_payload(scan: LaserScan) -> dict:
    ranges = [float(r) for r in scan.ranges]
    return {
        'header': {
            'stamp_sec': int(scan.header.stamp.sec),
            'stamp_nanosec': int(scan.header.stamp.nanosec),
            'stamp': float(scan.header.stamp.sec) + float(scan.header.stamp.nanosec) * 1e-9,
            'frame_id': str(scan.header.frame_id),
        },
        'angle_min': float(scan.angle_min),
        'angle_max': float(scan.angle_max),
        'angle_increment': float(scan.angle_increment),
        'time_increment': float(scan.time_increment),
        'scan_time': float(scan.scan_time),
        'range_min': float(scan.range_min),
        'range_max': float(scan.range_max),
        'ranges': ranges,
        'n': len(ranges),
    }


def map_payload(mp: OccupancyGrid) -> dict:
    info = mp.info
    q = info.origin.orientation
    data = bytes(int(v) & 0xFF for v in mp.data)
    return {
        'frame_id': str(mp.header.frame_id),
        'stamp_sec': int(mp.header.stamp.sec),
        'resolution': float(info.resolution),
        'width': int(info.width),
        'height': int(info.height),
        'origin_xyz': [float(info.origin.position.x), float(info.origin.position.y), float(info.origin.position.z)],
        'origin_yaw': float(yaw_of(q)),
        'data_sha256': hashlib.sha256(data).hexdigest(),
        'n': len(mp.data),
    }


class Capture(Node):
    def __init__(self) -> None:
        super().__init__('c4a3_dock_capture')
        self.box = {}
        self.create_subscription(OccupancyGrid, '/map', lambda m: self.box.__setitem__('map', m), LATCH)
        self.create_subscription(LaserScan, '/scan', lambda m: self.box.__setitem__('scan', m), 10)
        self.create_subscription(PoseWithCovarianceStamped, 'amcl_pose', lambda m: self.box.__setitem__('amcl', m), LATCH)
        self.create_subscription(Bool, '/xw/power/charging', lambda m: self.box.__setitem__('charging', bool(m.data)), 10)
        self.tf = Buffer()
        self.tfl = TransformListener(self.tf, self)
        self.pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)

    def wait_fresh(self, timeout=12.0) -> bool:
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.box.get('map') and self.box.get('scan') and self.box.get('amcl'):
                return True
        return False

    def publish_pose(self, xyz_yaw, cov_xy=0.05) -> None:
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = float(xyz_yaw[0])
        msg.pose.pose.position.y = float(xyz_yaw[1])
        msg.pose.pose.orientation = quat_from_yaw(float(xyz_yaw[2]))
        msg.pose.covariance[0] = cov_xy
        msg.pose.covariance[7] = cov_xy
        msg.pose.covariance[35] = 0.05
        for _ in range(5):
            self.pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.05)
            time.sleep(0.05)

    def wait_amcl_near(self, xy, timeout=8.0, tol=1.5):
        t0 = time.monotonic()
        last = None
        while time.monotonic() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.05)
            amcl = self.box.get('amcl')
            if amcl is None:
                continue
            last = pose_of(amcl)
            if math.hypot(last[0] - xy[0], last[1] - xy[1]) < tol:
                return last
        return last


def main() -> None:
    OUT.mkdir(parents=True, exist_ok=True)
    rclpy.init()
    n = Capture()
    ok = n.wait_fresh(15.0)
    # extra spins so static TF arrives
    t0 = time.monotonic()
    while time.monotonic() - t0 < 2.0:
        rclpy.spin_once(n, timeout_sec=0.05)
    scan = n.box.get('scan')
    mp = n.box.get('map')
    amcl = n.box.get('amcl')
    if not ok or scan is None or mp is None:
        print(json.dumps({'ok': False, 'have_map': bool(mp), 'have_scan': bool(scan), 'have_amcl': bool(amcl)}))
        n.destroy_node()
        rclpy.shutdown()
        raise SystemExit(2)

    amcl_a = pose_of(amcl) if amcl else None
    tfs = {
        'base_link_to_lidar_link': tf_dict(n.tf, 'base_link', 'lidar_link'),
        'lidar_link_to_base_link': tf_dict(n.tf, 'lidar_link', 'base_link'),
        'base_footprint_to_base_link': tf_dict(n.tf, 'base_footprint', 'base_link'),
        'base_link_to_base_footprint': tf_dict(n.tf, 'base_link', 'base_footprint'),
        'map_to_base_link_live': tf_dict(n.tf, 'map', 'base_link'),
        'map_to_odom_live': tf_dict(n.tf, 'map', 'odom'),
        'odom_to_base_link_live': tf_dict(n.tf, 'odom', 'base_link'),
        'map_to_lidar_link_live': tf_dict(n.tf, 'map', 'lidar_link'),
    }
    payload = scan_payload(scan)
    meta = {
        'captured_unix': time.time(),
        'charging': n.box.get('charging'),
        'amcl_pose_at_capture': amcl_a,
        'charger_waypoint': list(CHARGER),
        'pre_scramble_pose': list(PRE_SCRAMBLE),
        'wrong_corridor_pose': list(WRONG_CORRIDOR),
        'map': map_payload(mp),
        'tf_at_capture': tfs,
        'scan_header': payload['header'],
        'note': 'laser verify must score candidate*static extrinsic only; live map TF recorded but not used for scoring',
    }
    (OUT / 'scan.json').write_text(json.dumps(payload), encoding='utf-8')
    (OUT / 'capture_meta.json').write_text(json.dumps(meta, indent=2), encoding='utf-8')
    # occupancy raw for offline rescoring (same bytes the live node would score)
    import numpy as np
    grid = np.asarray(mp.data, dtype=np.int16)
    np.save(OUT / 'map_data.npy', grid)
    np.savez_compressed(
        OUT / 'map_grid.npz',
        data=grid,
        resolution=np.float64(mp.info.resolution),
        origin_x=np.float64(mp.info.origin.position.x),
        origin_y=np.float64(mp.info.origin.position.y),
        width=np.int32(mp.info.width),
        height=np.int32(mp.info.height),
    )
    for name in ('vp.yaml', 'vp.pgm'):
        src = MAPS / name
        if src.is_file():
            shutil.copy2(src, OUT / name)
    yaml_text = (MAPS / 'vp.yaml').read_text(encoding='utf-8') if (MAPS / 'vp.yaml').is_file() else ''
    pgm = (MAPS / 'vp.pgm').read_bytes() if (MAPS / 'vp.pgm').is_file() else b''
    (OUT / 'map_files_hash.json').write_text(json.dumps({
        'vp_yaml_sha256': hashlib.sha256(yaml_text.encode()).hexdigest() if yaml_text else None,
        'vp_pgm_sha256': hashlib.sha256(pgm).hexdigest() if pgm else None,
        'vp_yaml': yaml_text,
        'live_grid_sha256': meta['map']['data_sha256'],
    }, indent=2), encoding='utf-8')

    # Score SAME saved scan now (AMCL state A), via the production function. No TF lookup inside it.
    from xw_global_reloc.laser_verify import DistanceField
    from xw_phase2c.laser_prior_verify import verify_pose_with_laser

    field = DistanceField(mp)
    score_a = verify_pose_with_laser(PRE_SCRAMBLE, scan, mp, field=field, min_score=0.38)
    meta_a = {
        'state': 'A_amcl_at_capture',
        'amcl_pose': amcl_a,
        'candidate': list(PRE_SCRAMBLE),
        'scan_stamp': payload['header']['stamp'],
        'score': {k: score_a.get(k) for k in ('ok', 'reason', 'laser_score', 'matched_ratio', 'valid_beams', 'mean_dist')},
    }

    # State B: wrong corridor seed. Score the SAME LaserScan object, not a new /scan.
    n.publish_pose(WRONG_CORRIDOR, cov_xy=0.25)
    amcl_b = n.wait_amcl_near(WRONG_CORRIDOR, timeout=10.0, tol=2.0)
    tfs_b = {
        'map_to_base_link_live': tf_dict(n.tf, 'map', 'base_link'),
        'map_to_odom_live': tf_dict(n.tf, 'map', 'odom'),
        'odom_to_base_link_live': tf_dict(n.tf, 'odom', 'base_link'),
    }
    score_b = verify_pose_with_laser(PRE_SCRAMBLE, scan, mp, field=field, min_score=0.38)
    # also score a freshly received scan after the bad seed, to show live stream vs saved frame
    fresh = None
    t1 = time.monotonic()
    stamp0 = payload['header']['stamp']
    while time.monotonic() - t1 < 3.0:
        rclpy.spin_once(n, timeout_sec=0.05)
        s2 = n.box.get('scan')
        if s2 is not None:
            st = float(s2.header.stamp.sec) + float(s2.header.stamp.nanosec) * 1e-9
            if abs(st - stamp0) > 1e-6:
                fresh = s2
                break
    score_fresh = None
    if fresh is not None:
        score_fresh = verify_pose_with_laser(PRE_SCRAMBLE, fresh, mp, field=field, min_score=0.38)

    # Restore capture pose so we do not leave the robot scrambled.
    restore = amcl_a if amcl_a is not None else list(PRE_SCRAMBLE)
    n.publish_pose(restore, cov_xy=0.05)
    amcl_restored = n.wait_amcl_near(restore, timeout=8.0, tol=1.5)

    sa = score_a.get('laser_score')
    sb = score_b.get('laser_score')
    indep = {
        'same_scan_stamp': payload['header']['stamp'],
        'same_candidate': list(PRE_SCRAMBLE),
        'A': meta_a,
        'B': {
            'state': 'B_wrong_corridor_initialpose',
            'published_initialpose': list(WRONG_CORRIDOR),
            'amcl_pose_after': amcl_b,
            'live_tf_after': tfs_b,
            'score_same_saved_scan': {k: score_b.get(k) for k in ('ok', 'reason', 'laser_score', 'matched_ratio', 'valid_beams', 'mean_dist')},
            'score_fresh_scan_after_bad_amcl': None if score_fresh is None else {
                k: score_fresh.get(k) for k in ('ok', 'reason', 'laser_score', 'matched_ratio', 'valid_beams', 'mean_dist')
            },
        },
        'scores_equal': sa == sb and score_a.get('mean_dist') == score_b.get('mean_dist'),
        'delta_laser_score': None if sa is None or sb is None else float(sb) - float(sa),
        'restored_pose_target': restore,
        'amcl_after_restore': amcl_restored,
        'conclusion': 'INDEPENDENT' if (sa == sb) else 'CONTAMINATED',
    }
    (OUT / 'amcl_independence.json').write_text(json.dumps(indep, indent=2), encoding='utf-8')
    print(json.dumps({
        'ok': True,
        'charging': n.box.get('charging'),
        'amcl_A': amcl_a,
        'amcl_B': amcl_b,
        'score_A': indep['A']['score'],
        'score_B_same_scan': indep['B']['score_same_saved_scan'],
        'conclusion': indep['conclusion'],
        'scan_frame': payload['header']['frame_id'],
        'n': payload['n'],
        'tf_lidar': tfs['base_link_to_lidar_link'],
    }, indent=2, default=str))
    n.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
