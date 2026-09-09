#!/usr/bin/env python3
"""Phase2C-C4B3.1 state-machine closure observer.

Does not retune laser/AMCL. Does not repeat relocated P3 or charger P1.
Does not publish /initialpose except via the official Web API.
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
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Int8, String

from xw_interfaces.msg import PowerState
from xw_interfaces.srv import SetMode

OUT = Path(os.environ.get('C4B31_OUT', '/ros2_ws/bench/phase2c_c4b31_2026-09-09'))
MAP = os.environ.get('C4B31_MAP', 'vp')
WEB = os.environ.get('C4B31_WEB', 'http://127.0.0.1:9000/api/initialpose')
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
            'pass', 'selected_path', 'final', 'reason', 'p2_result', 'p2_score',
            'p3_ran', 'saw_verifying', 'loc', 'blocked',
        )
        if k in payload
    }
    print(json.dumps(brief, indent=2, default=str), flush=True)


class Watch(Node):
    def __init__(self) -> None:
        super().__init__('c4b31_closure')
        self.boot: Dict[str, Any] = {}
        self.loc = ''
        self.blocked: Optional[bool] = None
        self.status: Optional[int] = None
        self.amcl: Optional[Dict[str, float]] = None
        self.power: Dict[str, Any] = {}
        self.owners: List[Dict[str, Any]] = []
        self.states: List[Dict[str, Any]] = []
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

    def _boot(self, msg: String) -> None:
        try:
            self.boot = json.loads(msg.data or '{}')
        except json.JSONDecodeError:
            self.boot = {'raw': msg.data}
        self.states.append({
            't': time.time(),
            'state': self.boot.get('state'),
            'busy': self.boot.get('busy'),
        })

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
        self.power = {
            'charging': bool(msg.charging),
            'docked': bool(msg.docked),
        }

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

    def snap(self) -> Dict[str, Any]:
        return {
            't': time.time(),
            'boot': self.boot,
            'loc': self.loc,
            'blocked': self.blocked,
            'status': self.status,
            'amcl': self.amcl,
            'owner': self.owners[-1] if self.owners else None,
            'canonical': self.canonical[-1] if self.canonical else None,
            'nav_en': self.nav_en,
        }

    def publish_goal(self, x: float, y: float, yaw: float) -> None:
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.orientation.z = math.sin(float(yaw) * 0.5)
        msg.pose.orientation.w = math.cos(float(yaw) * 0.5)
        self._goal.publish(msg)

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
    }


def _p2_diag(result: Dict[str, Any]) -> Dict[str, Any]:
    stages = result.get('stages') or []
    accepted = next((s for s in stages if s.get('stage') == 'P2_ACCEPTED'), {})
    p2 = next((s for s in reversed(stages) if s.get('stage') == 'P2'), {})
    amcl = p2.get('amcl') if isinstance(p2.get('amcl'), dict) else {}
    p3 = [s for s in stages if s.get('stage') == 'P3']
    return {
        'accepted': accepted,
        'p2': {
            'result': p2.get('result'),
            'reason': p2.get('reason'),
            'score': p2.get('score', p2.get('laser_score')),
            'candidate_rejected': p2.get('candidate_rejected'),
            'handoff_failed': p2.get('handoff_failed'),
        },
        'amcl': {
            'tf_ok_after_sec': amcl.get('tf_ok_after_sec'),
            'status_0_after_sec': amcl.get('status_0_after_sec'),
            'stable_for_sec': amcl.get('stable_for_sec'),
            'latched_pose_used': amcl.get('latched_pose_used'),
            'near_candidate': amcl.get('near_candidate'),
            'far_latched_rejected': amcl.get('far_latched_rejected'),
            'loc_status': amcl.get('loc_status'),
            'tf_ok': amcl.get('tf_ok'),
            'amcl_convergence_sec': amcl.get('amcl_convergence_sec'),
        },
        'p3_ran': bool(p3),
        'sequential_fallback_seed_count': result.get('sequential_fallback_seed_count'),
        'selected_path': result.get('selected_path'),
        'final': result.get('final'),
        'saw_wait_settle': any(s.get('state') == 'WAIT_AMCL_SETTLE' for s in []),
    }


def wait_cascade(n: Watch, timeout: float) -> Dict[str, Any]:
    n.results.clear()
    t0 = time.monotonic()
    n.wait_until(
        lambda: bool(n.boot.get('busy')) or n.boot.get('state') in (
            'PRE_LOCALIZATION_READY', 'TRY_CHARGER', 'TRY_LAST_GOOD', 'WAIT_AMCL_SETTLE',
            'TRY_VISUAL_LASER', 'POST_SEED_AMCL_READY',
        ),
        min(25.0, timeout),
    )
    n.wait_until(
        lambda: (not n.boot.get('busy')) and n.boot.get('state') in (
            'READY', 'UNKNOWN', 'SENSOR_TIMEOUT'
        ),
        timeout,
    )
    n.spin(1.2)
    result = n.results[-1] if n.results else {}
    return {'elapsed_sec': round(time.monotonic() - t0, 2), 'result': result}


def start_nav(n: Watch) -> Dict[str, Any]:
    n.initialposes.clear()
    n.results.clear()
    n.cmd_samples.clear()
    idle = n.set_mode(0)
    n.spin(1.5)
    started = n.set_mode(2, json.dumps({'map_name': MAP}))
    return {'idle': idle, 'nav': started, 'last_good': read_pose_file()}


def run_p2(n: Watch) -> None:
    lg = read_pose_file()
    if not lg.get('laser_verified'):
        write_json('dedicated_p2.json', {'pass': False, 'reason': 'last_good_not_laser_verified', 'last_good': lg})
        return
    started = start_nav(n)
    watched = wait_cascade(n, 160.0)
    result = watched.get('result') or {}
    diag = _p2_diag(result)
    saw_settle = any(s.get('state') == 'WAIT_AMCL_SETTLE' for s in n.states)
    owners = _owners_ok(n.owners)
    report = {
        'started': started,
        'last_good': lg,
        'elapsed_sec': watched.get('elapsed_sec'),
        'saw_wait_amcl_settle': saw_settle,
        'p3_ran': diag['p3_ran'],
        'selected_path': diag['selected_path'] or n.boot.get('selected_path'),
        'final': diag['final'] or n.boot.get('state'),
        'p2_result': diag['p2'].get('result'),
        'p2_score': diag['p2'].get('score'),
        'p2_reason': diag['p2'].get('reason'),
        'candidate_rejected': diag['p2'].get('candidate_rejected'),
        'handoff_failed': diag['p2'].get('handoff_failed'),
        'amcl_settle': diag['amcl'],
        'sequential_fallback_seed_count': diag['sequential_fallback_seed_count'],
        'initialpose_count': len(n.initialposes),
        'initialposes': n.initialposes[-4:],
        'owners': owners,
        'loc': n.loc,
        'blocked': n.blocked,
        'canonical_tail': n.canonical[-8:],
        'states_tail': n.states[-12:],
        'result': result,
    }
    score = diag['p2'].get('score')
    report['pass'] = bool(
        report['selected_path'] == 'P2'
        and report['final'] == 'READY'
        and n.loc == 'READY'
        and n.blocked is False
        and not diag['p3_ran']
        and report['p2_result'] == 'READY'
        and isinstance(score, (int, float)) and float(score) >= 0.38
        and saw_settle
        and len(n.initialposes) == 1
        and not owners['legacy_seen']
        and not diag['p2'].get('candidate_rejected')
    )
    write_json('dedicated_p2.json', report)


def run_lost(n: Watch, kind: str) -> None:
    report: Dict[str, Any] = {
        'kind': kind,
        'snap0': n.snap(),
        'induction': 'localization_status=3 debounce while nav goal active',
        'scene': 'verified_high_score' if kind == 'positive' else 'separate_unknown_induction',
    }
    if kind == 'positive':
        n.wait_until(lambda: n.loc == 'READY' and n.blocked is False and n.amcl is not None, 15.0)
    if kind == 'positive' and not (n.loc == 'READY' and n.blocked is False):
        report['pass'] = False
        report['reason'] = 'not_ready_before_positive_lost'
        report['snap0'] = n.snap()
        write_json('lost_positive.json', report)
        return
    pose = n.amcl or {'x': 0.0, 'y': 0.0, 'yaw': 0.0}
    n.lost_results.clear()
    n.initialposes.clear()
    n.cmd_samples.clear()
    n.goals.clear()
    goal_n0 = len(n.goals)
    n.publish_goal(float(pose['x']) + 0.6, float(pose['y']), float(pose['yaw']))
    n.spin(2.0)
    t_induce = time.time()
    n.inject_status(3, 2.2)
    n.wait_until(lambda: n.loc in ('LOST', 'RECOVERING', 'NEED_OPERATOR') or bool(n.lost_results), 25.0)
    n.wait_until(lambda: bool(n.lost_results) and n.lost_results[-1].get('final') in ('READY', 'UNKNOWN'), 180.0)
    n.spin(2.0)
    lr = n.lost_results[-1] if n.lost_results else {}
    resume = lr.get('resume_policy') or {}
    snap = lr.get('snapshot') or {}
    motion = [s for s in n.cmd_samples if s['t'] >= t_induce]
    replans = [g for g in n.goals if g['t'] >= t_induce]
    report.update({
        'lost_result': lr,
        'final': lr.get('final'),
        'reason': lr.get('reason'),
        'resume_policy': resume,
        'snapshot': snap,
        'loc': n.loc,
        'blocked': n.blocked,
        'saw_lost': any(s.get('loc') == 'LOST' for s in n.loc_states),
        'saw_need_operator': any(s.get('loc') == 'NEED_OPERATOR' for s in n.loc_states),
        'initialpose_count': len(n.initialposes),
        'owners': _owners_ok(n.owners),
        'cmd_motion_samples': motion[-8:],
        'replan_count': len(replans),
        'goal_delta': len(n.goals) - goal_n0,
        'canonical_tail': n.canonical[-8:],
        'follow_en': n.follow_en,
        'recharge_en': n.recharge_en,
    })
    if kind == 'positive':
        report['pass'] = bool(
            lr.get('final') == 'READY'
            and snap.get('task_type') in ('navigate', 'none')
            and resume.get('nav') == 'replan_published'
            and n.loc == 'READY'
            and n.blocked is False
            and report['replan_count'] >= 1
            and not report['owners']['legacy_seen']
        )
        write_json('lost_positive.json', report)
    else:
        report['pass'] = bool(
            lr.get('final') == 'UNKNOWN'
            and n.loc == 'NEED_OPERATOR'
            and n.blocked is True
            and resume.get('nav') == 'forbidden'
            and resume.get('follow') == 'forbidden'
            and resume.get('recharge') == 'forbidden'
            and n.follow_en is not True
            and n.recharge_en is not True
            and len(motion) == 0
            and report['saw_need_operator']
        )
        write_json('lost_negative.json', report)


def run_operator(n: Watch, x: float, y: float, yaw: float, name: str, expect_ready: bool) -> None:
    report: Dict[str, Any] = {
        'pose': {'x': x, 'y': y, 'yaw': yaw},
        'expect_ready': expect_ready,
        'snap0': n.snap(),
    }
    n.wait_until(lambda: n.loc != '' and n.blocked is not None, 8.0)
    if n.loc != 'NEED_OPERATOR' or n.blocked is not True:
        report['pass'] = False
        report['reason'] = 'not_in_need_operator'
        report['loc'] = n.loc
        report['blocked'] = n.blocked
        write_json(name, report)
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
        if expect_ready and n.loc == 'READY' and n.blocked is False:
            break
        if (not expect_ready) and n.loc == 'NEED_OPERATOR' and n.boot.get('state') in (
            'UNKNOWN', 'SENSOR_TIMEOUT', 'READY'
        ) and saw_verifying and (time.monotonic() - t0) > 8.0:
            # Stay until verify window ends or returns NEED_OPERATOR.
            if n.boot.get('state') != 'VERIFYING_OPERATOR_POSE' and n.loc == 'NEED_OPERATOR':
                break
    n.spin(1.0)
    op = next((r for r in reversed(n.results) if r.get('source') == 'operator'), {})
    owners = [o for o in n.owners if o.get('owner') == 'operator']
    far = op.get('far_latched_rejected', 0)
    report.update({
        'web': web,
        'saw_verifying': saw_verifying,
        'saw_owner_operator': bool(owners),
        'operator_result': op,
        'far_latched_rejected': far,
        'loc': n.loc,
        'blocked': n.blocked,
        'initialpose_count': len(n.initialposes),
        'canonical_tail': n.canonical[-6:],
        'owners_tail': n.owners[-6:],
    })
    if expect_ready:
        report['pass'] = bool(
            saw_verifying
            and bool(owners)
            and n.loc == 'READY'
            and n.blocked is False
            and n.boot.get('state') == 'READY'
        )
    else:
        report['pass'] = bool(
            saw_verifying
            and bool(owners)
            and n.loc == 'NEED_OPERATOR'
            and n.blocked is True
            and n.boot.get('state') != 'READY'
        )
    write_json(name, report)


def late_latch(n: Watch) -> None:
    """New subscriber must see the current incident, not a stale BOOT READY."""
    seen = {'loc': None, 'blocked': None, 'canonical': None}

    class Late(Node):
        def __init__(self) -> None:
            super().__init__('c4b31_late_latch')
            self.create_subscription(String, '/xw/localization/phase2c_loc_state', self._loc, _LATCH)
            self.create_subscription(Bool, '/xw/nav/goals_blocked', self._block, _LATCH)
            self.create_subscription(String, '/xw/localization/phase2c_state', self._can, _LATCH)

        def _loc(self, msg: String) -> None:
            if seen['loc'] is None:
                seen['loc'] = msg.data

        def _block(self, msg: Bool) -> None:
            if seen['blocked'] is None:
                seen['blocked'] = bool(msg.data)

        def _can(self, msg: String) -> None:
            if seen['canonical'] is None:
                try:
                    seen['canonical'] = json.loads(msg.data or '{}')
                except json.JSONDecodeError:
                    seen['canonical'] = {'raw': msg.data}

    late = Late()
    t0 = time.monotonic()
    while time.monotonic() - t0 < 3.0 and rclpy.ok():
        rclpy.spin_once(late, timeout_sec=0.1)
        rclpy.spin_once(n, timeout_sec=0.05)
    late.destroy_node()
    report = {
        'late': seen,
        'live_loc': n.loc,
        'live_blocked': n.blocked,
        'pass': seen['loc'] == n.loc and seen['blocked'] == n.blocked and seen['loc'] not in (None, ''),
    }
    write_json('late_latch.json', report)


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument(
        '--phase',
        required=True,
        choices=['p2', 'lost_pos', 'lost_neg', 'operator_good', 'operator_bad', 'late_latch', 'snapshot'],
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
        elif args.phase == 'p2':
            run_p2(n)
        elif args.phase == 'lost_pos':
            run_lost(n, 'positive')
        elif args.phase == 'lost_neg':
            run_lost(n, 'negative')
        elif args.phase == 'operator_good':
            run_operator(n, args.x, args.y, args.yaw, 'operator_ready.json', True)
        elif args.phase == 'operator_bad':
            run_operator(n, args.x, args.y, args.yaw, 'operator_wrong.json', False)
        elif args.phase == 'late_latch':
            late_latch(n)
    finally:
        n.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
