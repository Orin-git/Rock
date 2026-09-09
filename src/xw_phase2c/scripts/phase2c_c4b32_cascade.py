#!/usr/bin/env python3
"""Phase2C-C4B3.2 LOST cascade observer.

Does not retune laser/AMCL/ORB. Does not publish operator /initialpose
except via the official Web API. False-good uses one AMCL seed to make
the current pose disagree with the live scan; that seed is the test
stimulus, not a recovery owner.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import time
import urllib.request
from pathlib import Path
from typing import Any, Dict, List, Optional

import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Quaternion, Twist
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Int8, String

from xw_interfaces.msg import PowerState
from xw_interfaces.srv import SetMode

OUT = Path(os.environ.get('C4B32_OUT', '/ros2_ws/bench/phase2c_c4b32_2026-09-09'))
MAP = os.environ.get('C4B32_MAP', 'vp')
WEB = os.environ.get('C4B32_WEB', 'http://127.0.0.1:9000/api/initialpose')
POSE_FILE = Path(os.environ.get('XW_MAPS', '/ros2_ws/maps')) / MAP / 'state' / 'last_good_pose.yaml'

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


def _yaw(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def read_pose_file() -> Dict[str, Any]:
    try:
        import yaml

        if not POSE_FILE.is_file():
            return {'missing': True, 'path': str(POSE_FILE)}
        data = yaml.safe_load(POSE_FILE.read_text(encoding='utf-8')) or {}
        data['path'] = str(POSE_FILE)
        return data
    except Exception as exc:  # noqa: BLE001
        return {'error': str(exc), 'path': str(POSE_FILE)}


def web_initialpose(x: float, y: float, yaw: float) -> Dict[str, Any]:
    body = json.dumps({'x': x, 'y': y, 'yaw': yaw, 'frame_id': 'map'}).encode()
    req = urllib.request.Request(
        WEB, data=body, headers={'Content-Type': 'application/json'}, method='POST'
    )
    try:
        with urllib.request.urlopen(req, timeout=8) as resp:
            raw = resp.read().decode()
            return {'http': resp.status, 'body': json.loads(raw)}
    except Exception as exc:  # noqa: BLE001
        return {'http': 0, 'error': str(exc)}


def write_json(name: str, payload: Dict[str, Any]) -> None:
    OUT.mkdir(parents=True, exist_ok=True)
    (OUT / name).write_text(json.dumps(payload, indent=2, default=str), encoding='utf-8')
    brief = {
        k: payload[k]
        for k in (
            'pass', 'selected_recovery_path', 'final', 'reason', 'r1_code', 'r2_code',
            'r3_code', 'laser_score', 'seed_count', 'loc', 'blocked',
        )
        if k in payload
    }
    print(json.dumps(brief, indent=2, default=str), flush=True)


class Watch(Node):
    def __init__(self) -> None:
        super().__init__('c4b32_cascade')
        self.boot: Dict[str, Any] = {}
        self.loc = ''
        self.blocked: Optional[bool] = None
        self.status: Optional[int] = None
        self.amcl: Optional[Dict[str, float]] = None
        self.power: Dict[str, Any] = {}
        self.owners: List[Dict[str, Any]] = []
        self.loc_states: List[Dict[str, Any]] = []
        self.canonical: List[Dict[str, Any]] = []
        self.results: List[Dict[str, Any]] = []
        self.lost_results: List[Dict[str, Any]] = []
        self.initialposes: List[Dict[str, Any]] = []
        self.cmd_samples: List[Dict[str, float]] = []
        self.goals: List[Dict[str, Any]] = []
        self.nav_en: Optional[bool] = None
        self.follow_en: Optional[bool] = None
        self.recharge_en: Optional[bool] = None
        self.create_subscription(String, '/xw/boot/status', self._boot, _LATCH)
        self.create_subscription(String, '/xw/boot/result', self._result, 20)
        self.create_subscription(String, '/xw/localization/phase2c_loc_state', self._loc, _LATCH)
        self.create_subscription(String, '/xw/localization/phase2c_state', self._can, _LATCH)
        self.create_subscription(String, '/xw/localization/phase2c_lost_result', self._lost, 20)
        self.create_subscription(Bool, '/xw/nav/goals_blocked', self._block, _LATCH)
        self.create_subscription(Int8, '/xw/localization_status', self._status, _LATCH)
        self.create_subscription(PoseWithCovarianceStamped, 'amcl_pose', self._amcl, _AMCL)
        self.create_subscription(String, '/xw/localization/initialpose_owner', self._owner, _LATCH)
        self.create_subscription(PoseWithCovarianceStamped, '/initialpose', self._ipose, 20)
        self.create_subscription(PowerState, '/xw/power', self._power, 10)
        self.create_subscription(Twist, '/xw/cmd/nav', self._cmd, 20)
        self.create_subscription(PoseStamped, '/xw/goal_pose', self._goal_in, 20)
        self.create_subscription(Bool, '/xw/nav/enable', self._nav, _LATCH)
        self.create_subscription(Bool, '/xw/follow/enable', self._follow, _LATCH)
        self.create_subscription(Bool, '/xw/recharge/enable', self._recharge, _LATCH)
        self._mode = self.create_client(SetMode, '/xw/supervisor/set_mode')
        self._goal = self.create_publisher(PoseStamped, '/xw/goal_pose', 10)
        self._status_pub = self.create_publisher(Int8, '/xw/localization_status', _LATCH)
        self._ip_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)

    def _boot(self, msg: String) -> None:
        try:
            self.boot = json.loads(msg.data or '{}')
        except json.JSONDecodeError:
            self.boot = {'raw': msg.data}

    def _result(self, msg: String) -> None:
        try:
            self.results.append(json.loads(msg.data or '{}'))
        except json.JSONDecodeError:
            self.results.append({'raw': msg.data})

    def _loc(self, msg: String) -> None:
        self.loc = msg.data or ''
        self.loc_states.append({'t': time.time(), 'loc': self.loc, 'blocked': self.blocked})

    def _can(self, msg: String) -> None:
        try:
            data = json.loads(msg.data or '{}')
        except json.JSONDecodeError:
            data = {'raw': msg.data}
        self.canonical.append({'t': time.time(), **data})

    def _lost(self, msg: String) -> None:
        try:
            self.lost_results.append(json.loads(msg.data or '{}'))
        except json.JSONDecodeError:
            self.lost_results.append({'raw': msg.data})

    def _block(self, msg: Bool) -> None:
        self.blocked = bool(msg.data)

    def _status(self, msg: Int8) -> None:
        self.status = int(msg.data)

    def _amcl(self, msg: PoseWithCovarianceStamped) -> None:
        p = msg.pose.pose
        c = msg.pose.covariance
        self.amcl = {
            'x': float(p.position.x),
            'y': float(p.position.y),
            'yaw': _yaw(p.orientation),
            'cov_xy': max(float(c[0]), float(c[7])),
            'cov_yaw': float(c[35]),
            't': time.time(),
        }

    def _owner(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data or '{}')
        except json.JSONDecodeError:
            payload = {'raw': msg.data}
        self.owners.append({'t': time.time(), **payload})

    def _ipose(self, msg: PoseWithCovarianceStamped) -> None:
        p = msg.pose.pose
        self.initialposes.append({
            't': time.time(),
            'x': float(p.position.x),
            'y': float(p.position.y),
            'yaw': _yaw(p.orientation),
            'frame': msg.header.frame_id,
        })

    def _goal_in(self, msg: PoseStamped) -> None:
        self.goals.append({
            't': time.time(),
            'x': float(msg.pose.position.x),
            'y': float(msg.pose.position.y),
        })

    def _power(self, msg: PowerState) -> None:
        self.power = {'charging': bool(msg.charging), 'docked': bool(msg.docked)}

    def _cmd(self, msg: Twist) -> None:
        lin = math.hypot(float(msg.linear.x), float(msg.linear.y))
        ang = abs(float(msg.angular.z))
        if lin > 0.02 or ang > 0.05:
            self.cmd_samples.append({'t': time.time(), 'lin': lin, 'ang': ang})

    def _nav(self, msg: Bool) -> None:
        self.nav_en = bool(msg.data)

    def _follow(self, msg: Bool) -> None:
        self.follow_en = bool(msg.data)

    def _recharge(self, msg: Bool) -> None:
        self.recharge_en = bool(msg.data)

    def spin(self, sec: float) -> None:
        end = time.monotonic() + sec
        while time.monotonic() < end and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)

    def wait_until(self, pred, timeout: float) -> bool:
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            if pred():
                return True
        return False

    def snap(self) -> Dict[str, Any]:
        return {
            't': time.time(),
            'boot': self.boot,
            'loc': self.loc,
            'blocked': self.blocked,
            'status': self.status,
            'amcl': self.amcl,
            'owner': self.owners[-1] if self.owners else None,
            'nav_en': self.nav_en,
        }

    def set_mode(self, mode: int, payload: str = '') -> Dict[str, Any]:
        if not self._mode.wait_for_service(timeout_sec=15.0):
            return {'ok': False, 'error': 'set_mode_unavailable'}
        req = SetMode.Request()
        req.mode = int(mode)
        req.payload_json = payload
        fut = self._mode.call_async(req)
        t0 = time.monotonic()
        while time.monotonic() - t0 < 30.0 and rclpy.ok() and not fut.done():
            rclpy.spin_once(self, timeout_sec=0.1)
        if not fut.done() or fut.result() is None:
            return {'ok': False, 'error': 'set_mode_timeout'}
        res = fut.result()
        return {'ok': bool(res.success), 'message': res.message}

    def publish_goal(self, x: float, y: float, yaw: float) -> None:
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.orientation.z = math.sin(float(yaw) * 0.5)
        msg.pose.orientation.w = math.cos(float(yaw) * 0.5)
        self._goal.publish(msg)

    def publish_seed(self, x: float, y: float, yaw: float) -> None:
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.pose.position.x = float(x)
        msg.pose.pose.position.y = float(y)
        q = Quaternion()
        q.z = math.sin(float(yaw) * 0.5)
        q.w = math.cos(float(yaw) * 0.5)
        msg.pose.pose.orientation = q
        msg.pose.covariance[0] = 0.25
        msg.pose.covariance[7] = 0.25
        msg.pose.covariance[35] = 0.15
        self._ip_pub.publish(msg)

    def inject_status(self, code: int, seconds: float) -> None:
        msg = Int8()
        msg.data = int(code)
        end = time.monotonic() + seconds
        while time.monotonic() < end and rclpy.ok():
            self._status_pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.05)


def _owners_ok(owners: List[Dict[str, Any]]) -> Dict[str, Any]:
    names = [o.get('owner') for o in owners if o.get('owner')]
    return {
        'owners': names[-16:],
        'legacy_seen': any(n == 'legacy' for n in names),
        'boot_and_lost': 'boot' in names and 'lost' in names,
    }


def start_nav(n: Watch) -> Dict[str, Any]:
    n.initialposes.clear()
    idle = n.set_mode(0)
    n.spin(1.0)
    started = n.set_mode(2, json.dumps({'map_name': MAP}))
    return {'idle': idle, 'nav': started, 'last_good': read_pose_file()}


def _cascade(lr: Dict[str, Any]) -> Dict[str, Any]:
    stages = lr.get('stages') or []
    return {
        'selected_recovery_path': lr.get('selected_recovery_path'),
        'r1_code': lr.get('r1_code'),
        'r2_code': lr.get('r2_code'),
        'r3_code': lr.get('r3_code'),
        'R1_SKIP_REASON': lr.get('R1_SKIP_REASON'),
        'laser_score': lr.get('laser_score'),
        'seed_count': lr.get('seed_count'),
        'sequential_fallback_seed_count': lr.get('sequential_fallback_seed_count'),
        'final': lr.get('final'),
        'reason': lr.get('reason'),
        'total_sec': lr.get('total_sec'),
        'timings': lr.get('timings'),
        'resume_policy': lr.get('resume_policy'),
        'stages': [
            {k: s.get(k) for k in (
                'code', 'stage', 'reason', 'R1_SKIP_REASON', 'laser_score',
                'coverage_ok', 'seeded', 'handoff', 'ready', 'valid_beams',
            )}
            for s in stages
        ],
    }


def run_lost(n: Watch, kind: str) -> None:
    name = {
        'positive': 'lost_positive.json',
        'false_good': 'false_good.json',
        'negative': 'lost_negative.json',
    }[kind]
    report: Dict[str, Any] = {
        'kind': kind,
        'snap0': n.snap(),
        'last_good': read_pose_file(),
        'induction': 'localization_status=3 debounce while nav goal active',
    }
    ok_loc = ('READY',) if kind == 'positive' else ('READY', 'DEGRADED')
    n.wait_until(lambda: n.loc in ok_loc and n.blocked is False and n.amcl is not None, 20.0)
    if not (n.loc in ok_loc and n.blocked is False and n.amcl is not None):
        report['pass'] = False
        report['reason'] = 'not_ready_before_lost'
        report['snap0'] = n.snap()
        write_json(name, report)
        return
    true_pose = dict(n.amcl)
    n.lost_results.clear()
    n.initialposes.clear()
    n.cmd_samples.clear()
    n.goals.clear()
    n.owners.clear()
    n.publish_goal(float(true_pose['x']) + 0.6, float(true_pose['y']), float(true_pose['yaw']))
    n.spin(1.5)
    if kind == 'false_good':
        # Stimulus only: move AMCL belief off the live scan. Not a recovery seed.
        # In-map but far from the live scan. Out-of-map would SKIP R1, not laser-reject.
        sx, sy, syaw = 2.4, 0.8, 0.0
        n.publish_seed(sx, sy, syaw)
        n.wait_until(
            lambda: n.amcl is not None and math.hypot(n.amcl['x'] - sx, n.amcl['y'] - sy) < 1.5,
            8.0,
        )
        report['stimulus_pose'] = {'x': sx, 'y': sy, 'yaw': syaw}
        report['amcl_after_stimulus'] = n.amcl
    t_induce = time.time()
    n.inject_status(3, 2.2)
    n.wait_until(lambda: n.loc in ('LOST', 'RECOVERING', 'NEED_OPERATOR') or bool(n.lost_results), 25.0)
    n.wait_until(lambda: bool(n.lost_results) and n.lost_results[-1].get('final') in ('READY', 'UNKNOWN'), 180.0)
    n.spin(1.5)
    lr = n.lost_results[-1] if n.lost_results else {}
    casc = _cascade(lr)
    resume = lr.get('resume_policy') or {}
    motion = [s for s in n.cmd_samples if s['t'] >= t_induce]
    replans = [g for g in n.goals if g['t'] >= t_induce]
    r1 = next((s for s in (lr.get('stages') or []) if s.get('stage') == 'R1'), {})
    report.update({
        **casc,
        'lost_result': lr,
        'snapshot': lr.get('snapshot'),
        'loc': n.loc,
        'blocked': n.blocked,
        'saw_lost': any(s.get('loc') == 'LOST' for s in n.loc_states),
        'saw_need_operator': any(s.get('loc') == 'NEED_OPERATOR' for s in n.loc_states),
        'initialpose_count': len(n.initialposes),
        'initialposes': n.initialposes[-6:],
        'owners': _owners_ok(n.owners),
        'cmd_motion_samples': motion[-8:],
        'replan_count': len(replans),
        'true_pose': true_pose,
        'amcl_end': n.amcl,
        'r1_seeded': bool(r1.get('seeded')),
    })
    if kind == 'positive':
        path = lr.get('selected_recovery_path')
        report['pass'] = bool(
            lr.get('final') == 'READY'
            and path in ('R1', 'R2')
            and lr.get('r3_code') in ('', None)
            and resume.get('nav') == 'replan_published'
            and n.loc == 'READY'
            and n.blocked is False
            and report['replan_count'] >= 1
            and isinstance(lr.get('laser_score'), (int, float))
            and float(lr.get('laser_score')) >= 0.38
            and int(lr.get('seed_count') or 0) == 1
            and int(lr.get('sequential_fallback_seed_count') or 0) == 0
            and not report['owners']['legacy_seen']
        )
    elif kind == 'false_good':
        # Wrong current pose must not become READY via R1. R2/R3 fallback allowed.
        report['pass'] = bool(
            lr.get('r1_code') == 'R1_CURRENT_REJECT'
            and not r1.get('seeded')
            and r1.get('handoff') != 'READY'
            and lr.get('selected_recovery_path') != 'R1'
            and not report['owners']['legacy_seen']
        )
    else:
        report['pass'] = bool(
            lr.get('final') == 'UNKNOWN'
            and lr.get('r3_code') == 'R3_VISUAL_UNKNOWN'
            and lr.get('r1_code') in ('R1_CURRENT_REJECT', 'R1_CURRENT_SKIP')
            and lr.get('r2_code') in ('R2_LAST_GOOD_REJECT', 'R2_LAST_GOOD_SKIP')
            and n.loc == 'NEED_OPERATOR'
            and n.blocked is True
            and resume.get('nav') == 'forbidden'
            and len(motion) == 0
            and report['saw_need_operator']
        )
    write_json(name, report)


def run_operator(n: Watch, x: float, y: float, yaw: float) -> None:
    report: Dict[str, Any] = {
        'pose': {'x': x, 'y': y, 'yaw': yaw},
        'snap0': n.snap(),
    }
    n.wait_until(lambda: n.loc != '' and n.blocked is not None, 8.0)
    if n.loc != 'NEED_OPERATOR' or n.blocked is not True:
        report['pass'] = False
        report['reason'] = 'not_in_need_operator'
        report['loc'] = n.loc
        report['blocked'] = n.blocked
        write_json('operator_ready.json', report)
        return
    n.results.clear()
    n.initialposes.clear()
    web = web_initialpose(x, y, yaw)
    saw_verifying = False
    t0 = time.monotonic()
    while time.monotonic() - t0 < 28.0:
        n.spin(0.2)
        if n.boot.get('state') == 'VERIFYING_OPERATOR_POSE' or n.loc == 'VERIFYING_OPERATOR_POSE':
            saw_verifying = True
        if n.loc == 'READY' and n.blocked is False:
            break
    n.spin(1.0)
    owners = [o for o in n.owners if o.get('owner') == 'operator']
    report.update({
        'web': web,
        'saw_verifying': saw_verifying,
        'saw_owner_operator': bool(owners),
        'loc': n.loc,
        'blocked': n.blocked,
        'initialpose_count': len(n.initialposes),
        'canonical_tail': n.canonical[-6:],
    })
    report['pass'] = bool(
        saw_verifying
        and bool(owners)
        and n.loc == 'READY'
        and n.blocked is False
    )
    write_json('operator_ready.json', report)


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument(
        '--phase',
        required=True,
        choices=['snapshot', 'lost_pos', 'false_good', 'lost_neg', 'operator_good'],
    )
    ap.add_argument('--x', type=float, default=0.0)
    ap.add_argument('--y', type=float, default=0.0)
    ap.add_argument('--yaw', type=float, default=0.0)
    args = ap.parse_args()
    OUT.mkdir(parents=True, exist_ok=True)
    rclpy.init()
    n = Watch()
    try:
        n.spin(1.2)
        if args.phase == 'snapshot':
            write_json('snapshot.json', {'snap': n.snap(), 'last_good': read_pose_file()})
        elif args.phase == 'lost_pos':
            if not (n.loc == 'READY' and n.blocked is False):
                start_nav(n)
                n.wait_until(lambda: n.loc == 'READY' and n.blocked is False, 90.0)
            run_lost(n, 'positive')
        elif args.phase == 'false_good':
            run_lost(n, 'false_good')
        elif args.phase == 'lost_neg':
            run_lost(n, 'negative')
        elif args.phase == 'operator_good':
            run_operator(n, args.x, args.y, args.yaw)
    finally:
        n.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
