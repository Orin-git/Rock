#!/usr/bin/env python3
"""BOOT A: charging-only evidence, one wrong /initialpose, then P1 cascade.

IR/docked are ignored. No set_mode, no node restart, no reinitialize loop.
"""

from __future__ import annotations

import json
import math
import os
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Int8, String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener

from xw_interfaces.msg import PowerState

OUT = Path('/ros2_ws/bench/phase2c_c4a2_physical_2026-09-08')
CHARGER = (1.8663955491712294, -0.05958837147746455, -3.1286646850836126)
# Far from charger so a correct residual pose cannot be scored as recovery.
SCRAMBLE = (-8.936784667454088, 1.5960588981494703, -2.195708002205529)
# Trial-node startup itself lifts load; abort only in the 15–20 pollution band.
LOAD_MAX = 15.5
LOAD_ABORT = 16.0

LATCH = QoSProfile(
    depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE
)
AMCL = QoSProfile(
    depth=1,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
)


def load1() -> float:
    return float(open('/proc/loadavg').read().split()[0])


def yaw_of(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def quat(yaw: float):
    from geometry_msgs.msg import Quaternion

    q = Quaternion()
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q


class BootA(Node):
    def __init__(self) -> None:
        super().__init__('xw_phase2c_boot_a_scramble')
        self.map = None
        self.scan = None
        self.amcl = None
        self.power = PowerState()
        self.prior = None
        self.diag = ''
        self.loc = -1
        self.boot_result = None
        self.tf = Buffer()
        self._tfl = TransformListener(self.tf, self)
        self.create_subscription(OccupancyGrid, '/map', lambda m: setattr(self, 'map', m), LATCH)
        self.create_subscription(LaserScan, '/scan', lambda m: setattr(self, 'scan', m), 10)
        self.create_subscription(PoseWithCovarianceStamped, 'amcl_pose', lambda m: setattr(self, 'amcl', m), AMCL)
        self.create_subscription(PowerState, '/xw/power', lambda m: setattr(self, 'power', m), 10)
        self.create_subscription(Bool, '/xw/localization/charger_prior_available', lambda m: setattr(self, 'prior', bool(m.data)), LATCH)
        self.create_subscription(String, '/xw/localization/charger_prior_diag', lambda m: setattr(self, 'diag', m.data), LATCH)
        self.create_subscription(Int8, '/xw/localization_status', lambda m: setattr(self, 'loc', int(m.data)), LATCH)
        self.create_subscription(String, '/xw/boot/result', lambda m: setattr(self, 'boot_result', m.data), 10)
        self.pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self.boot_cli = self.create_client(Trigger, '/xw/boot/run')

    def spin_sec(self, sec: float) -> None:
        t0 = time.monotonic()
        while time.monotonic() - t0 < sec and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)

    def xy(self):
        if not self.amcl:
            return None
        p = self.amcl.pose.pose
        return (float(p.position.x), float(p.position.y), yaw_of(p.orientation))

    def tf_ok(self) -> bool:
        try:
            self.tf.lookup_transform('map', 'odom', rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=0.3))
            self.tf.lookup_transform('odom', 'base_link', rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=0.3))
            return True
        except TransformException:
            return False

    def scramble(self) -> None:
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.pose.position.x = SCRAMBLE[0]
        msg.pose.pose.position.y = SCRAMBLE[1]
        msg.pose.pose.orientation = quat(SCRAMBLE[2])
        msg.pose.covariance[0] = 0.25
        msg.pose.covariance[7] = 0.25
        msg.pose.covariance[35] = 0.1
        for _ in range(3):
            self.pose_pub.publish(msg)
            self.spin_sec(0.2)


def main() -> int:
    row = {
        'trial': 'BOOT_A',
        'ir_policy': 'ignored; charging state only',
        'scramble_pose': {'x': SCRAMBLE[0], 'y': SCRAMBLE[1], 'yaw': SCRAMBLE[2]},
    }
    rclpy.init()
    n = BootA()
    # Power is not latched; /map latch can arrive late. Wait until both are real.
    t_wait = time.monotonic()
    while time.monotonic() - t_wait < 15.0:
        n.spin_sec(0.25)
        if n.map is not None and n.scan is not None and n.amcl is not None and bool(n.power.charging):
            break
    load = load1()
    row['prereq'] = {
        'map': n.map is not None,
        'scan': n.scan is not None,
        'amcl': n.amcl is not None,
        'tf': n.tf_ok(),
        'load': load,
        'load_max': LOAD_MAX,
        'charging': bool(n.power.charging),
        'docked_ignored': bool(n.power.docked),
        'battery': float(n.power.battery_percent),
        'prior': n.prior,
        'loc': n.loc,
        'pose_before': n.xy(),
        'boot_svc': n.boot_cli.service_is_ready() or n.boot_cli.wait_for_service(timeout_sec=3.0),
    }
    print('PRE', json.dumps(row['prereq'], default=str), flush=True)
    ok = (
        row['prereq']['map']
        and row['prereq']['scan']
        and row['prereq']['amcl']
        and row['prereq']['tf']
        and row['prereq']['boot_svc']
        and bool(n.power.charging)
        and load <= LOAD_MAX
    )
    if not ok:
        row['score'] = 'FAIL'
        row['reason'] = 'prereq_failed'
        if not n.power.charging:
            row['reason'] = 'no_charging_evidence'
        row['diag'] = n.diag
        _save(row)
        n.destroy_node()
        rclpy.shutdown()
        return 1

    n.scramble()
    t0 = time.monotonic()
    moved = False
    while time.monotonic() - t0 < 12.0:
        n.spin_sec(0.3)
        pose = n.xy()
        if pose and math.hypot(pose[0] - CHARGER[0], pose[1] - CHARGER[1]) > 2.0:
            moved = True
            break
    row['pose_after_scramble'] = n.xy()
    row['scramble_moved'] = moved
    print('SCRAMBLE', row['pose_after_scramble'], 'moved', moved, 'load', load1(), flush=True)
    if not moved:
        row['score'] = 'FAIL'
        row['reason'] = 'scramble_did_not_move_amcl'
        _save(row)
        n.destroy_node()
        rclpy.shutdown()
        return 1

    if load1() > LOAD_ABORT or n.map is None:
        row['score'] = 'FAIL'
        row['reason'] = 'load_or_map_gate'
        row['load_after_scramble'] = load1()
        _save(row)
        n.destroy_node()
        rclpy.shutdown()
        return 1

    n.boot_result = None
    fut = n.boot_cli.call_async(Trigger.Request())
    t0 = time.monotonic()
    while time.monotonic() - t0 < 180.0 and rclpy.ok():
        n.spin_sec(0.05)
        if fut.done() and n.boot_result:
            break
    n.spin_sec(1.0)
    parsed = {}
    if n.boot_result:
        try:
            parsed = json.loads(n.boot_result)
        except json.JSONDecodeError:
            parsed = {'raw': n.boot_result}
    stages = parsed.get('stages') or []
    row['boot'] = {
        'final': parsed.get('final'),
        'selected_path': parsed.get('selected_path'),
        'stages': stages,
        'amcl_pose': parsed.get('amcl_pose'),
        'total_sec': parsed.get('total_boot_localization_sec'),
    }
    row['p1_stage'] = next((s for s in stages if str(s.get('stage')) == 'P1'), None)
    row['pose_after'] = n.xy()
    row['loc_after'] = n.loc
    row['load_after'] = load1()
    row['power'] = {
        'charging': bool(n.power.charging),
        'docked_ignored': bool(n.power.docked),
        'battery': float(n.power.battery_percent),
    }
    boot = row['boot']
    if (
        boot.get('final') == 'READY'
        and boot.get('selected_path') == 'P1'
        and n.power.charging
    ):
        row['score'] = 'PASS'
        row['blind_seed_proof'] = (
            'charging-only soft prior; IR ignored; scrambled AMCL then P1 laser seed'
        )
    else:
        row['score'] = 'FAIL'
        row['reason'] = 'p1_not_ready'
    _save(row)
    print(json.dumps({'score': row['score'], 'path': boot.get('selected_path'), 'final': boot.get('final')}, indent=2), flush=True)
    n.destroy_node()
    rclpy.shutdown()
    return 0 if row['score'] == 'PASS' else 1


def _save(row) -> None:
    OUT.mkdir(parents=True, exist_ok=True)
    path = OUT / 'trial_BOOT_A.json'
    path.write_text(json.dumps(row, indent=2, default=str) + '\n', encoding='utf-8')
    ledger_path = OUT / 'ledger.json'
    ledger = {}
    if ledger_path.is_file():
        try:
            ledger = json.loads(ledger_path.read_text(encoding='utf-8'))
        except json.JSONDecodeError:
            ledger = {}
    ledger.setdefault('trials', {})['BOOT_A'] = {
        'score': row.get('score'),
        'file': str(path),
        'stamp': time.time(),
        'note': 'charging-only; IR ignored; one scramble then P1',
    }
    ledger_path.write_text(json.dumps(ledger, indent=2) + '\n', encoding='utf-8')
    print('saved', path, row.get('score'), flush=True)


if __name__ == '__main__':
    raise SystemExit(main())
