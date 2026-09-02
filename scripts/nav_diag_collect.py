#!/usr/bin/env python3
"""Lightweight navigation diagnostic sampler (low CPU).

Samples cached topic/TF state at fixed rate (default 2 Hz) and writes CSV.
Run inside the ROS container while navigating; stop with Ctrl+C when done.

Usage:
  source /ros2_ws/scripts/ros_env.sh
  python3 /ros2_ws/scripts/nav_diag_collect.py --out /ros2_ws/log/nav_diag_run
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import os
import signal
import sys
import time
from dataclasses import dataclass, field
from typing import Optional

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu, LaserScan, PointCloud2
from std_msgs.msg import Bool, Int8
from tf2_ros import Buffer, TransformException, TransformListener


def _yaw_from_quat(x: float, y: float, z: float, w: float) -> float:
    siny = 2.0 * (w * z + x * y)
    cosy = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny, cosy)


def _stamp_age_sec(node: Node, header_stamp) -> float:
    if header_stamp.sec == 0 and header_stamp.nanosec == 0:
        return float('nan')
    t_msg = header_stamp.sec + header_stamp.nanosec * 1e-9
    t_now = node.get_clock().now().nanoseconds * 1e-9
    return t_now - t_msg


@dataclass
class Latest:
    amcl: Optional[PoseWithCovarianceStamped] = None
    odom: Optional[Odometry] = None
    odom_wheel: Optional[Odometry] = None
    cmd_nav: Optional[Twist] = None
    cmd_sm: Optional[Twist] = None
    imu: Optional[Imu] = None
    scan: Optional[LaserScan] = None
    pc_up: Optional[PointCloud2] = None
    pc_down: Optional[PointCloud2] = None
    loc_status: Optional[int] = None
    pc_nav_en: Optional[bool] = None
    counts: dict = field(default_factory=lambda: {
        'amcl': 0, 'odom': 0, 'odom_wheel': 0, 'cmd_nav': 0, 'cmd_sm': 0,
        'imu': 0, 'scan': 0, 'pc_up': 0, 'pc_down': 0,
    })


class NavDiagCollector(Node):
    CSV_HEADER = [
        't', 'wall_t',
        'amcl_x', 'amcl_y', 'amcl_yaw', 'amcl_cov_xx', 'amcl_cov_yy', 'amcl_cov_yaw',
        'amcl_age',
        'odom_x', 'odom_y', 'odom_yaw', 'odom_vx', 'odom_wz', 'odom_age',
        'wheel_x', 'wheel_y', 'wheel_yaw', 'wheel_vx',
        'cmd_nav_vx', 'cmd_nav_wz', 'cmd_sm_vx', 'cmd_sm_wz',
        'imu_wz',
        'loc_status',
        'scan_valid_pct', 'scan_min_m', 'scan_max_m', 'scan_age',
        'pc_up_n', 'pc_down_n', 'pc_nav_en',
        'tf_map_ok', 'tf_map_x', 'tf_map_y', 'tf_map_yaw', 'tf_map_age',
        'tf_odom_ok', 'tf_odom_x', 'tf_odom_y', 'tf_odom_yaw',
        'drift_map_odom_m', 'drift_map_odom_yaw_deg',
    ]

    def __init__(self, out_dir: str, hz: float) -> None:
        super().__init__('nav_diag_collector')
        self._out_dir = out_dir
        self._hz = max(0.5, min(hz, 10.0))
        self._latest = Latest()
        self._start_wall = time.time()
        self._start_ros = self.get_clock().now().nanoseconds * 1e-9
        self._sample_idx = 0
        self._running = True

        os.makedirs(out_dir, exist_ok=True)
        self._csv_path = os.path.join(out_dir, 'samples.csv')
        self._csv_file = open(self._csv_path, 'w', newline='')
        self._writer = csv.writer(self._csv_file)
        self._writer.writerow(self.CSV_HEADER)

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        default_qos = 10

        self._tf = Buffer()
        self._tf_listener = TransformListener(self._tf, self)

        self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose', self._on_amcl, default_qos)
        self.create_subscription(Odometry, '/odom', self._on_odom, default_qos)
        self.create_subscription(Odometry, '/odom/wheel', self._on_wheel, default_qos)
        self.create_subscription(Twist, '/cmd_vel_nav', self._on_cmd_nav, default_qos)
        self.create_subscription(Twist, '/cmd_vel_smoothed', self._on_cmd_sm, default_qos)
        self.create_subscription(Imu, '/imu/data', self._on_imu, default_qos)
        self.create_subscription(LaserScan, '/scan', self._on_scan, sensor_qos)
        self.create_subscription(
            PointCloud2, '/camera/front_up/depth/points_nav',
            self._on_pc_up, sensor_qos)
        self.create_subscription(
            PointCloud2, '/camera/front_down/depth/points_nav',
            self._on_pc_down, sensor_qos)
        self.create_subscription(Int8, '/xw/localization_status', self._on_loc, default_qos)
        self.create_subscription(Bool, '/xw/camera/pointcloud_enabled', self._on_pc_en, default_qos)

        self.create_timer(1.0 / self._hz, self._sample)
        self.get_logger().info(
            f'nav_diag_collect → {self._csv_path} @ {self._hz:.1f} Hz (Ctrl+C to stop)')

    def _touch(self, key: str) -> None:
        self._latest.counts[key] = self._latest.counts.get(key, 0) + 1

    def _on_amcl(self, msg: PoseWithCovarianceStamped) -> None:
        self._latest.amcl = msg
        self._touch('amcl')

    def _on_odom(self, msg: Odometry) -> None:
        self._latest.odom = msg
        self._touch('odom')

    def _on_wheel(self, msg: Odometry) -> None:
        self._latest.odom_wheel = msg
        self._touch('odom_wheel')

    def _on_cmd_nav(self, msg: Twist) -> None:
        self._latest.cmd_nav = msg
        self._touch('cmd_nav')

    def _on_cmd_sm(self, msg: Twist) -> None:
        self._latest.cmd_sm = msg
        self._touch('cmd_sm')

    def _on_imu(self, msg: Imu) -> None:
        self._latest.imu = msg
        self._touch('imu')

    def _on_scan(self, msg: LaserScan) -> None:
        self._latest.scan = msg
        self._touch('scan')

    def _on_pc_up(self, msg: PointCloud2) -> None:
        self._latest.pc_up = msg
        self._touch('pc_up')

    def _on_pc_down(self, msg: PointCloud2) -> None:
        self._latest.pc_down = msg
        self._touch('pc_down')

    def _on_loc(self, msg: Int8) -> None:
        self._latest.loc_status = int(msg.data)

    def _on_pc_en(self, msg: Bool) -> None:
        self._latest.pc_nav_en = bool(msg.data)

    def _pose_row(self, msg: Optional[PoseWithCovarianceStamped]):
        if msg is None:
            return [float('nan')] * 6 + [float('nan')]
        p = msg.pose.pose.position
        o = msg.pose.pose.orientation
        yaw = _yaw_from_quat(o.x, o.y, o.z, o.w)
        c = msg.pose.covariance
        age = _stamp_age_sec(self, msg.header.stamp)
        return [p.x, p.y, yaw, c[0], c[7], c[35], age]

    def _odom_row(self, msg: Optional[Odometry]):
        if msg is None:
            return [float('nan')] * 6
        p = msg.pose.pose.position
        o = msg.pose.pose.orientation
        yaw = _yaw_from_quat(o.x, o.y, o.z, o.w)
        age = _stamp_age_sec(self, msg.header.stamp)
        return [p.x, p.y, yaw, msg.twist.twist.linear.x, msg.twist.twist.angular.z, age]

    def _wheel_row(self, msg: Optional[Odometry]):
        if msg is None:
            return [float('nan')] * 4
        p = msg.pose.pose.position
        o = msg.pose.pose.orientation
        yaw = _yaw_from_quat(o.x, o.y, o.z, o.w)
        return [p.x, p.y, yaw, msg.twist.twist.linear.x]

    def _scan_row(self, msg: Optional[LaserScan]):
        if msg is None:
            return [float('nan')] * 4
        valid = 0
        total = 0
        mn = float('inf')
        mx = 0.0
        for r in msg.ranges:
            total += 1
            if not math.isfinite(r):
                continue
            if r < msg.range_min or r > msg.range_max:
                continue
            valid += 1
            mn = min(mn, r)
            mx = max(mx, r)
        pct = (100.0 * valid / total) if total else float('nan')
        if mn == float('inf'):
            mn = float('nan')
        age = _stamp_age_sec(self, msg.header.stamp)
        return [pct, mn, mx, age]

    def _pc_n(self, msg: Optional[PointCloud2]) -> float:
        if msg is None:
            return float('nan')
        return float(msg.width * msg.height)

    def _tf_row(self, parent: str, child: str):
        try:
            tf = self._tf.lookup_transform(parent, child, rclpy.time.Time())
            t = tf.transform.translation
            q = tf.transform.rotation
            yaw = _yaw_from_quat(q.x, q.y, q.z, q.w)
            age = _stamp_age_sec(self, tf.header.stamp)
            return 1, t.x, t.y, yaw, age
        except TransformException:
            return 0, float('nan'), float('nan'), float('nan'), float('nan')

    def _sample(self) -> None:
        if not self._running:
            return
        t_ros = self.get_clock().now().nanoseconds * 1e-9 - self._start_ros
        t_wall = time.time() - self._start_wall
        L = self._latest

        amcl = self._pose_row(L.amcl)
        odom = self._odom_row(L.odom)
        wheel = self._wheel_row(L.odom_wheel)

        cmd = L.cmd_nav
        cmd_sm = L.cmd_sm
        cmd_nav_vx = cmd.linear.x if cmd else float('nan')
        cmd_nav_wz = cmd.angular.z if cmd else float('nan')
        cmd_sm_vx = cmd_sm.linear.x if cmd_sm else float('nan')
        cmd_sm_wz = cmd_sm.angular.z if cmd_sm else float('nan')

        imu_wz = L.imu.angular_velocity.z if L.imu else float('nan')
        loc = L.loc_status if L.loc_status is not None else float('nan')
        scan = self._scan_row(L.scan)
        pc_up_n = self._pc_n(L.pc_up)
        pc_down_n = self._pc_n(L.pc_down)
        pc_en = int(L.pc_nav_en) if L.pc_nav_en is not None else float('nan')

        tf_map = self._tf_row('map', 'base_link')
        tf_odom = self._tf_row('odom', 'base_link')

        drift_m = float('nan')
        drift_yaw = float('nan')
        if tf_map[0] and tf_odom[0]:
            dx = tf_map[1] - tf_odom[1]
            dy = tf_map[2] - tf_odom[2]
            drift_m = math.hypot(dx, dy)
            dyaw = tf_map[3] - tf_odom[3]
            while dyaw > math.pi:
                dyaw -= 2 * math.pi
            while dyaw < -math.pi:
                dyaw += 2 * math.pi
            drift_yaw = math.degrees(dyaw)

        row = [
            f'{t_ros:.3f}', f'{t_wall:.3f}',
            *amcl, *odom, *wheel,
            cmd_nav_vx, cmd_nav_wz, cmd_sm_vx, cmd_sm_wz,
            imu_wz, loc,
            *scan, pc_up_n, pc_down_n, pc_en,
            *tf_map, *tf_odom[:4],
            drift_m, drift_yaw,
        ]
        self._writer.writerow(row)
        self._sample_idx += 1
        if self._sample_idx % 20 == 0:
            self._csv_file.flush()
            self.get_logger().info(
                f'samples={self._sample_idx} t={t_ros:.0f}s '
                f'amcl=({amcl[0]:.2f},{amcl[1]:.2f}) cmd_vx={cmd_nav_vx:.3f} wz={cmd_nav_wz:.3f}')

    def finalize(self) -> None:
        self._running = False
        self._csv_file.flush()
        self._csv_file.close()
        meta = {
            'start_wall': self._start_wall,
            'duration_sec': time.time() - self._start_wall,
            'sample_hz': self._hz,
            'sample_count': self._sample_idx,
            'msg_counts': self._latest.counts,
            'csv': self._csv_path,
        }
        meta_path = os.path.join(self._out_dir, 'meta.json')
        with open(meta_path, 'w', encoding='utf-8') as f:
            json.dump(meta, f, indent=2)
        self.get_logger().info(f'done: {self._sample_idx} samples → {self._out_dir}')


def main() -> None:
    parser = argparse.ArgumentParser(description='Lightweight nav diagnostic sampler')
    parser.add_argument('--out', required=True, help='Output directory for CSV/meta')
    parser.add_argument('--hz', type=float, default=2.0, help='Sample rate (default 2 Hz)')
    args = parser.parse_args()

    rclpy.init()
    node = NavDiagCollector(args.out, args.hz)

    def _shutdown(*_):
        node.finalize()
        node.destroy_node()
        rclpy.shutdown()

    signal.signal(signal.SIGINT, _shutdown)
    signal.signal(signal.SIGTERM, _shutdown)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            _shutdown()


if __name__ == '__main__':
    main()
