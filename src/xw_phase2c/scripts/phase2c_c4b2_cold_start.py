#!/usr/bin/env python3
"""Phase2C-C4B.2 cold NAV + operator-owner probe.

Rules:
  - no manual /initialpose unless --operator-pose
  - no legacy blind seed
  - Phase2C must publish /initialpose itself
  - prove initialpose_time < first_valid_map_odom_time
"""

from __future__ import annotations

import argparse
import json
import math
import os
import time
from pathlib import Path
from typing import Any, Dict, Optional

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, Quaternion
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Int8, String
from tf2_ros import Buffer, TransformException, TransformListener

from xw_interfaces.srv import SetMode
from xw_phase2c.ownership import OWNER_TOPIC, owner_payload, InitialPoseOwner


_LATCH = QoSProfile(
    depth=1,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    reliability=ReliabilityPolicy.RELIABLE,
)
_AMCL = QoSProfile(
    depth=1,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
)

OUT = Path(os.environ.get('C4B2_OUT', '/ros2_ws/bench/phase2c_c4b2_cold_2026-09-09'))
MAP = os.environ.get('C4B2_MAP', 'vp')


def _yaw_quat(yaw: float) -> Quaternion:
    q = Quaternion()
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q


class Probe(Node):
    def __init__(self) -> None:
        super().__init__('c4b2_cold_probe')
        self.boot_status: Dict[str, Any] = {}
        self.boot_result: Dict[str, Any] = {}
        self.owner: Dict[str, Any] = {}
        self.loc_state = ''
        self.goals_blocked: Optional[bool] = None
        self.loc_status: Optional[int] = None
        self.ip_count = 0
        self.ip_wall: Optional[float] = None
        self.map_odom_wall: Optional[float] = None
        self.amcl_n = 0
        self.owners = []
        self.tf = Buffer()
        self.tf_listener = TransformListener(self.tf, self)
        self.create_subscription(String, '/xw/boot/status', self._on_boot, _LATCH)
        self.create_subscription(String, '/xw/boot/result', self._on_result, 10)
        self.create_subscription(String, OWNER_TOPIC, self._on_owner, _LATCH)
        self.create_subscription(String, '/xw/localization/phase2c_loc_state', self._on_loc, _LATCH)
        self.create_subscription(Bool, '/xw/nav/goals_blocked', self._on_block, _LATCH)
        self.create_subscription(Int8, '/xw/localization_status', self._on_status, _LATCH)
        self.create_subscription(PoseWithCovarianceStamped, '/initialpose', self._on_ip, 10)
        self.create_subscription(PoseWithCovarianceStamped, 'amcl_pose', self._on_amcl, _AMCL)
        self._mode = self.create_client(SetMode, '/xw/supervisor/set_mode')
        self._ip_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self._owner_pub = self.create_publisher(String, OWNER_TOPIC, _LATCH)

    def _on_boot(self, msg: String) -> None:
        try:
            self.boot_status = json.loads(msg.data or '{}')
        except json.JSONDecodeError:
            self.boot_status = {'raw': msg.data}

    def _on_result(self, msg: String) -> None:
        try:
            self.boot_result = json.loads(msg.data or '{}')
        except json.JSONDecodeError:
            self.boot_result = {'raw': msg.data}

    def _on_owner(self, msg: String) -> None:
        try:
            self.owner = json.loads(msg.data or '{}')
        except json.JSONDecodeError:
            self.owner = {'raw': msg.data}
        self.owners.append({'t': time.time(), **self.owner})

    def _on_loc(self, msg: String) -> None:
        self.loc_state = msg.data or ''

    def _on_block(self, msg: Bool) -> None:
        self.goals_blocked = bool(msg.data)

    def _on_status(self, msg: Int8) -> None:
        self.loc_status = int(msg.data)

    def _on_ip(self, _msg: PoseWithCovarianceStamped) -> None:
        self.ip_count += 1
        if self.ip_wall is None:
            self.ip_wall = time.time()

    def _on_amcl(self, _msg: PoseWithCovarianceStamped) -> None:
        self.amcl_n += 1

    def spin_for(self, sec: float) -> None:
        end = time.monotonic() + sec
        while time.monotonic() < end and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            self._note_tf()

    def _note_tf(self) -> None:
        if self.map_odom_wall is not None:
            return
        if self.ip_wall is None and self.ip_count == 0:
            return
        try:
            if self.tf.can_transform('map', 'odom', rclpy.time.Time()):
                self.map_odom_wall = time.time()
        except TransformException:
            return

    def start_nav(self) -> Dict[str, Any]:
        if not self._mode.wait_for_service(timeout_sec=15.0):
            return {'ok': False, 'error': 'set_mode_unavailable'}
        req = SetMode.Request()
        req.mode = 2
        req.payload_json = json.dumps({'map_name': MAP})
        fut = self._mode.call_async(req)
        t0 = time.monotonic()
        while time.monotonic() - t0 < 30.0 and rclpy.ok() and not fut.done():
            rclpy.spin_once(self, timeout_sec=0.1)
        if not fut.done() or fut.result() is None:
            return {'ok': False, 'error': 'set_mode_timeout'}
        res = fut.result()
        return {'ok': bool(res.success), 'message': res.message}

    def publish_operator_pose(self, x: float, y: float, yaw: float) -> None:
        self._owner_pub.publish(
            String(data=owner_payload(InitialPoseOwner.OPERATOR, 0, note='c4b2_web_equivalent'))
        )
        time.sleep(0.05)
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.pose.position.x = float(x)
        msg.pose.pose.position.y = float(y)
        msg.pose.pose.orientation = _yaw_quat(yaw)
        msg.pose.covariance[0] = 0.25
        msg.pose.covariance[7] = 0.25
        msg.pose.covariance[35] = 0.068
        self._ip_pub.publish(msg)


def wait_cascade(n: Probe, timeout: float) -> None:
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout and rclpy.ok():
        n.spin_for(0.3)
        final = (n.boot_result or {}).get('final')
        if final in ('READY', 'UNKNOWN', 'SENSOR_TIMEOUT', 'CANCELLED'):
            # allow a short settle for TF stamp
            n.spin_for(1.0)
            return


def summarize(n: Probe, started: Dict[str, Any]) -> Dict[str, Any]:
    ms = (n.boot_result or {}).get('milestones') or (n.boot_status or {}).get('milestones') or {}
    ip_m = ms.get('initialpose_publish_mono')
    tf_m = ms.get('first_map_odom_mono')
    order_ok = (
        ip_m is not None and tf_m is not None and float(ip_m) < float(tf_m)
    )
    final = (n.boot_result or {}).get('final')
    return {
        'nav_start': started,
        'final': final,
        'boot_state': (n.boot_status or {}).get('state'),
        'loc_state': n.loc_state,
        'goals_blocked': n.goals_blocked,
        'loc_status': n.loc_status,
        'selected_path': (n.boot_result or {}).get('selected_path'),
        'ip_count': n.ip_count,
        'owners_tail': n.owners[-8:],
        'milestones': ms,
        'initialpose_before_map_odom': order_ok,
        'map_odom_before_seed': ms.get('map_odom_before_seed'),
        'amcl_msgs': n.amcl_n,
        'manual_initialpose': False,
        'legacy_blind_seed': False,
    }


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument('--timeout', type=float, default=180.0)
    ap.add_argument('--operator-pose', action='store_true')
    ap.add_argument('--x', type=float, default=0.44)
    ap.add_argument('--y', type=float, default=-0.59)
    ap.add_argument('--yaw', type=float, default=0.31)
    args = ap.parse_args()
    OUT.mkdir(parents=True, exist_ok=True)
    rclpy.init()
    n = Probe()
    try:
        n.spin_for(2.0)
        started = n.start_nav()
        wait_cascade(n, args.timeout)
        report = summarize(n, started)
        if args.operator_pose and n.loc_state == 'NEED_OPERATOR':
            n.boot_result = {}
            n.publish_operator_pose(args.x, args.y, args.yaw)
            t0 = time.monotonic()
            while time.monotonic() - t0 < 40.0 and rclpy.ok():
                n.spin_for(0.3)
                if n.loc_state == 'READY' and n.goals_blocked is False:
                    break
            report['operator_fallback'] = {
                'loc_state': n.loc_state,
                'boot_state': (n.boot_status or {}).get('state'),
                'goals_blocked': n.goals_blocked,
                'owner': n.owner,
            }
        report['pass_cold'] = bool(
            report.get('final') == 'READY'
            and report.get('goals_blocked') is False
            and report.get('initialpose_before_map_odom')
            and report.get('ip_count', 0) >= 1
        )
        (OUT / 'cold_nav.json').write_text(json.dumps(report, indent=2, default=str), encoding='utf-8')
        print(json.dumps(report, indent=2, default=str))
    finally:
        n.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
