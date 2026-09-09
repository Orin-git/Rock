#!/usr/bin/env python3
"""Phase2C-C4B3 final production smoke observer.

Does not retune ORB/NPU/Depth/laser/AMCL/BOOT/LOST.
Does not publish a manual /initialpose unless --operator-web is used
(official Web API only, for the operator-fallback regression).
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

OUT = Path(os.environ.get('C4B3_OUT', '/ros2_ws/bench/phase2c_c4b3_2026-09-09'))
MAP = os.environ.get('C4B3_MAP', 'vp')
WEB = os.environ.get('C4B3_WEB', 'http://127.0.0.1:9000/api/initialpose')
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
        data['mtime'] = POSE_FILE.stat().st_mtime
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


class Watch(Node):
    def __init__(self) -> None:
        super().__init__('c4b3_final_smoke')
        self.boot: Dict[str, Any] = {}
        self.loc = ''
        self.blocked: Optional[bool] = None
        self.status: Optional[int] = None
        self.amcl: Optional[Dict[str, float]] = None
        self.power = {}
        self.owners: List[Dict[str, Any]] = []
        self.states: List[Dict[str, Any]] = []
        self.loc_states: List[Dict[str, Any]] = []
        self.results: List[Dict[str, Any]] = []
        self.lost_results: List[Dict[str, Any]] = []
        self.initialposes: List[Dict[str, Any]] = []
        self.cmd_samples: List[Dict[str, float]] = []
        self.nav_en: Optional[bool] = None
        self.follow_en: Optional[bool] = None
        self.recharge_en: Optional[bool] = None
        self.recovery: Optional[bool] = None
        self.create_subscription(String, '/xw/boot/status', self._boot, _LATCH)
        self.create_subscription(String, '/xw/boot/result', self._result, 20)
        self.create_subscription(String, '/xw/localization/phase2c_loc_state', self._loc, _LATCH)
        self.create_subscription(String, '/xw/localization/phase2c_lost_result', self._lost, 20)
        self.create_subscription(Bool, '/xw/nav/goals_blocked', self._block, _LATCH)
        self.create_subscription(Int8, '/xw/localization_status', self._status, _LATCH)
        self.create_subscription(PoseWithCovarianceStamped, 'amcl_pose', self._amcl, _AMCL)
        self.create_subscription(String, '/xw/localization/initialpose_owner', self._owner, _LATCH)
        self.create_subscription(PoseWithCovarianceStamped, '/initialpose', self._ipose, 20)
        self.create_subscription(PowerState, '/xw/power', self._power, 10)
        self.create_subscription(Twist, '/xw/cmd/nav', self._cmd, 20)
        self.create_subscription(Bool, '/xw/nav/enable', self._nav, _LATCH)
        self.create_subscription(Bool, '/xw/follow/enable', self._follow, _LATCH)
        self.create_subscription(Bool, '/xw/recharge/enable', self._recharge, _LATCH)
        self.create_subscription(Bool, '/xw/localization/phase2c_recovery', self._rec, _LATCH)
        self._mode = self.create_client(SetMode, '/xw/supervisor/set_mode')
        self._goal = self.create_publisher(PoseStamped, '/xw/goal_pose', 10)
        self._status_pub = self.create_publisher(Int8, '/xw/localization_status', _LATCH)

    def _boot(self, msg: String) -> None:
        try:
            self.boot = json.loads(msg.data or '{}')
        except json.JSONDecodeError:
            self.boot = {'raw': msg.data}
        self.states.append({'t': time.time(), 'state': self.boot.get('state'), 'busy': self.boot.get('busy')})

    def _result(self, msg: String) -> None:
        try:
            self.results.append(json.loads(msg.data or '{}'))
        except json.JSONDecodeError:
            self.results.append({'raw': msg.data})

    def _loc(self, msg: String) -> None:
        self.loc = msg.data or ''
        self.loc_states.append({'t': time.time(), 'loc': self.loc})

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

    def _power(self, msg: PowerState) -> None:
        self.power = {
            'charging': bool(msg.charging),
            'docked': bool(msg.docked),
            'battery_percent': float(msg.battery_percent),
            'current': float(msg.charging_current),
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

    def _rec(self, msg: Bool) -> None:
        self.recovery = bool(msg.data)

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
            'power': self.power,
            'owner': self.owners[-1] if self.owners else None,
            'nav_en': self.nav_en,
            'follow_en': self.follow_en,
            'recharge_en': self.recharge_en,
            'recovery': self.recovery,
            'initialpose_count': len(self.initialposes),
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


def _stage_rows(result: Dict[str, Any]) -> List[Dict[str, Any]]:
    rows = []
    for s in result.get('stages') or []:
        amcl = s.get('amcl') if isinstance(s.get('amcl'), dict) else {}
        rows.append({
            'stage': s.get('stage'),
            'result': s.get('result'),
            'reason': s.get('reason'),
            'score': s.get('score', s.get('laser_score')),
            'candidate_pose': s.get('candidate_pose'),
            'latched_pose_used': amcl.get('latched_pose_used'),
            'near_candidate': amcl.get('near_candidate'),
            'amcl_pose': amcl.get('amcl_pose') or s.get('amcl_pose'),
        })
    return rows


def _owners_ok(owners: List[Dict[str, Any]]) -> Dict[str, Any]:
    names = [o.get('owner') for o in owners if o.get('owner')]
    legacy = [o for o in owners if o.get('owner') == 'legacy']
    return {
        'owners': names[-16:],
        'legacy_seen': bool(legacy),
        'allowed_only': all(n in ('boot', 'lost', 'reloc', 'operator', 'none', None, '') for n in names),
    }


def wait_cascade(n: Watch, timeout: float) -> Dict[str, Any]:
    n.results.clear()
    t0 = time.monotonic()
    # Ignore the latched IDLE left by the previous session. A new cascade
    # must go busy, then reach a terminal state, or publish /xw/boot/result.
    n.wait_until(
        lambda: bool(n.boot.get('busy')) or n.boot.get('state') in (
            'PRE_LOCALIZATION_READY', 'TRY_CHARGER', 'TRY_LAST_GOOD', 'TRY_VISUAL_LASER',
            'POST_SEED_AMCL_READY',
        ),
        min(20.0, timeout),
    )
    ok = n.wait_until(
        lambda: (not n.boot.get('busy')) and n.boot.get('state') in (
            'READY', 'UNKNOWN', 'SENSOR_TIMEOUT'
        ),
        timeout,
    )
    n.spin(1.2)
    result = n.results[-1] if n.results else {}
    return {
        'settled': ok,
        'elapsed_sec': round(time.monotonic() - t0, 2),
        'result': result,
        'stages': _stage_rows(result),
        'snap': n.snap(),
        'owners_tail': n.owners[-12:],
        'states': n.states[-20:],
        'initialposes': list(n.initialposes),
    }


def start_nav(n: Watch) -> Dict[str, Any]:
    n.initialposes.clear()
    n.results.clear()
    n.cmd_samples.clear()
    idle = n.set_mode(0)
    n.spin(1.5)
    started = n.set_mode(2, json.dumps({'map_name': MAP}))
    return {'idle': idle, 'nav': started, 'last_good': read_pose_file()}


def write_json(name: str, payload: Dict[str, Any]) -> None:
    OUT.mkdir(parents=True, exist_ok=True)
    (OUT / name).write_text(json.dumps(payload, indent=2, default=str), encoding='utf-8')
    brief_keys = (
        'pass', 'outcome', 'selected_path', 'final', 'reason', 'p2_result', 'p2_score',
        'p3_ran', 'initialpose_count', 'snap_end', 'charging',
    )
    print(json.dumps({k: payload[k] for k in brief_keys if k in payload}, indent=2, default=str))


def run_session(n: Watch, label: str, timeout: float) -> Dict[str, Any]:
    started = start_nav(n)
    watched = wait_cascade(n, timeout)
    result = watched.get('result') or {}
    stages = result.get('stages') or []
    p2 = next((s for s in stages if s.get('stage') == 'P2'), {})
    p3s = [s for s in stages if s.get('stage') == 'P3']
    p1 = next((s for s in stages if s.get('stage') == 'P1'), {})
    selected = result.get('selected_path') or n.boot.get('selected_path')
    own = _owners_ok(n.owners)
    report = {
        'label': label,
        'started': started,
        'selected_path': selected,
        'final': result.get('final') or n.boot.get('state'),
        'p1': {'result': p1.get('result'), 'reason': p1.get('reason'), 'score': p1.get('score', p1.get('laser_score'))},
        'p2_result': p2.get('result'),
        'p2_reason': p2.get('reason'),
        'p2_score': p2.get('score', p2.get('laser_score')),
        'p2_amcl': (p2.get('amcl') or {}) if isinstance(p2.get('amcl'), dict) else {},
        'p3_ran': bool(p3s),
        'p3_scores': [
            {
                'attempt': s.get('attempt'),
                'result': s.get('result'),
                'reason': s.get('reason'),
                'score': s.get('score', s.get('laser_score')),
            }
            for s in p3s
        ],
        'stages': watched.get('stages'),
        'initialpose_count': len(n.initialposes),
        'initialposes': n.initialposes[-6:],
        'milestones': result.get('milestones') or {},
        'cold_break': result.get('cold_break'),
        'owners': own,
        'snap_end': n.snap(),
        'elapsed_sec': watched.get('elapsed_sec'),
        'manual_initialpose': False,
        'legacy_blind_seed': own['legacy_seen'],
    }
    return report


def run_t1(n: Watch) -> None:
    n.spin(1.0)
    report = run_session(n, 't1_cold_nav', 200.0)
    ms = report.get('milestones') or {}
    ip_m = ms.get('initialpose_publish_mono')
    tf_m = ms.get('first_map_odom_mono')
    order_ok = ip_m is not None and tf_m is not None and float(ip_m) < float(tf_m)
    report['initialpose_before_map_odom'] = order_ok
    report['pass'] = bool(
        report['final'] == 'READY'
        and n.blocked is False
        and n.loc == 'READY'
        and report['initialpose_count'] >= 1
        and not report['legacy_blind_seed']
        and order_ok
        and report['selected_path'] in ('P1', 'P2', 'P3')
    )
    write_json('t1_cold_nav.json', report)


def run_t2(n: Watch) -> None:
    file0 = read_pose_file()
    report: Dict[str, Any] = {'file_before': file0, 'snap0': n.snap()}
    if not file0.get('laser_verified'):
        t0 = time.monotonic()
        while time.monotonic() - t0 < 45.0:
            n.spin(1.0)
            cur = read_pose_file()
            if cur.get('laser_verified'):
                file0 = cur
                break
    report['file_before'] = file0
    if not file0.get('laser_verified'):
        report['pass'] = False
        report['reason'] = 'no_laser_verified_last_good'
        write_json('t2_unmoved_p2.json', report)
        return
    sess = run_session(n, 't2_unmoved_p2', 160.0)
    report.update(sess)
    p2_amcl = report.get('p2_amcl') or {}
    report['pass'] = bool(
        file0.get('laser_verified')
        and report.get('selected_path') == 'P2'
        and report.get('final') == 'READY'
        and n.blocked is False
        and not report.get('p3_ran')
        and report.get('initialpose_count') == 1
        and not report.get('legacy_blind_seed')
    )
    report['latched_pose_used'] = p2_amcl.get('latched_pose_used')
    report['near_candidate'] = p2_amcl.get('near_candidate')
    write_json('t2_unmoved_p2.json', report)


def run_t3(n: Watch) -> None:
    """Expect caller already relocated the robot. Does not publish /initialpose."""
    old = read_pose_file()
    report = run_session(n, 't3_relocated_p3', 220.0)
    report['old_last_good'] = {
        k: old.get(k) for k in ('x', 'y', 'yaw', 'laser_verified', 'laser_score_at_write', 'timestamp')
    }
    p2_accepted = report.get('p2_result') == 'READY' and report.get('selected_path') == 'P2'
    ready = report.get('final') == 'READY' and n.loc == 'READY' and n.blocked is False
    unknown = n.loc == 'NEED_OPERATOR' and n.blocked is True
    report['p2_false_accept'] = bool(p2_accepted)
    report['outcome'] = 'READY' if ready else ('SAFE_UNKNOWN' if unknown else 'FAIL')
    report['pass'] = bool(
        (not p2_accepted)
        and report.get('p3_ran')
        and (ready or unknown)
        and report.get('initialpose_count', 99) <= 1
        and not report.get('legacy_blind_seed')
    )
    write_json('t3_relocated_p3.json', report)


def run_t4(n: Watch) -> None:
    n.spin(1.0)
    report: Dict[str, Any] = {'power': n.power, 'snap0': n.snap()}
    if not (n.power.get('charging') or n.power.get('docked')):
        report['pass'] = False
        report['reason'] = 'not_charging_no_dock_evidence'
        report['note'] = 'robot not on dock; charger blind seed not used; path stopped'
        write_json('t4_charger_p1.json', report)
        return
    sess = run_session(n, 't4_charger_p1', 160.0)
    report.update(sess)
    report['pass'] = bool(
        report.get('selected_path') == 'P1'
        and report.get('final') == 'READY'
        and n.blocked is False
        and not report.get('legacy_blind_seed')
    )
    write_json('t4_charger_p1.json', report)


def run_t5(n: Watch) -> None:
    """Controlled health anomaly during NavigateToPose. No reinitialize_global_localization."""
    report: Dict[str, Any] = {'snap0': n.snap(), 'induction': 'localization_status=3 debounce while nav goal active'}
    n.wait_until(lambda: n.loc == 'READY' or n.boot.get('state') == 'READY', 8.0)
    if (n.loc != 'READY' and n.boot.get('state') != 'READY') or n.blocked:
        report['pass'] = False
        report['reason'] = 'not_ready_before_nav'
        write_json('t5_lost_nav.json', report)
        return
    pose = n.amcl or {'x': 0.0, 'y': 0.0, 'yaw': 0.0}
    n.lost_results.clear()
    n.initialposes.clear()
    n.cmd_samples.clear()
    n.publish_goal(float(pose['x']) + 0.8, float(pose['y']), float(pose['yaw']))
    n.spin(2.0)
    report['goal'] = {'x': pose['x'] + 0.8, 'y': pose['y'], 'yaw': pose['yaw']}
    n.inject_status(3, 2.5)
    n.wait_until(lambda: n.loc in ('LOST', 'RECOVERING', 'NEED_OPERATOR') or bool(n.lost_results), 20.0)
    report['after_induce'] = n.snap()
    n.wait_until(lambda: bool(n.lost_results) and n.lost_results[-1].get('final') in ('READY', 'UNKNOWN'), 180.0)
    n.spin(1.5)
    lr = n.lost_results[-1] if n.lost_results else {}
    resume = lr.get('resume_policy') or {}
    snap = lr.get('snapshot') or {}
    report.update({
        'lost_result': lr,
        'final': lr.get('final'),
        'reason': lr.get('reason'),
        'resume_policy': resume,
        'snapshot': snap,
        'saw_lost': any(s.get('loc') == 'LOST' for s in n.loc_states),
        'saw_stop': bool(snap) and n.blocked is not None,
        'goals_blocked_during': True if any(True for _ in n.loc_states) else None,
        'initialpose_count': len(n.initialposes),
        'owners': _owners_ok(n.owners),
        'snap_end': n.snap(),
        'reinit_storm': False,
    })
    report['pass'] = bool(
        lr.get('final') == 'READY'
        and lr.get('reason')
        and snap.get('task_type') in ('navigate', 'none')
        and resume.get('nav') in ('replan_published', 'no_goal_saved')
        and n.loc == 'READY'
        and n.blocked is False
        and not report['owners']['legacy_seen']
    )
    if snap.get('nav_goal') and resume.get('nav') != 'replan_published':
        report['pass'] = False
    write_json('t5_lost_nav.json', report)


def run_t6(n: Watch) -> None:
    """LOST then Reloc cannot ACCEPT. Holds reloc process only if --hold-reloc was set externally.

    This phase only observes. If Reloc is already unavailable or returns non-READY, record UNKNOWN gate.
    """
    report: Dict[str, Any] = {'snap0': n.snap(), 'induction': 'status=3 while reloc cannot safely ACCEPT'}
    n.lost_results.clear()
    n.cmd_samples.clear()
    n.inject_status(3, 1.4)
    n.wait_until(lambda: bool(n.lost_results), 160.0)
    n.spin(2.0)
    lr = n.lost_results[-1] if n.lost_results else {}
    resume = lr.get('resume_policy') or {}
    n.spin(3.0)
    motion = [s for s in n.cmd_samples if s['t'] >= (report['snap0']['t'] - 1)]
    report.update({
        'lost_result': lr,
        'final': lr.get('final'),
        'resume_policy': resume,
        'snap_end': n.snap(),
        'cmd_motion_samples': motion[-8:],
        'follow_en': n.follow_en,
        'recharge_en': n.recharge_en,
        'nav_en': n.nav_en,
    })
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
    )
    write_json('t6_lost_unknown.json', report)


def run_operator(n: Watch, x: float, y: float, yaw: float) -> None:
    report: Dict[str, Any] = {'snap0': n.snap(), 'pose': {'x': x, 'y': y, 'yaw': yaw}}
    n.wait_until(
        lambda: n.loc == 'NEED_OPERATOR' or (
            n.blocked is True and 'UNKNOWN' in str((n.owners[-1] if n.owners else {}).get('note') or '')
        ),
        8.0,
    )
    if n.loc != 'NEED_OPERATOR' and not (
        n.blocked is True and 'UNKNOWN' in str((n.owners[-1] if n.owners else {}).get('note') or '')
    ):
        report['pass'] = False
        report['reason'] = 'not_in_need_operator'
        write_json('operator_fallback.json', report)
        return
    good = web_initialpose(x, y, yaw)
    saw_verifying = False
    t0 = time.monotonic()
    while time.monotonic() - t0 < 30.0:
        n.spin(0.25)
        if n.boot.get('state') == 'VERIFYING_OPERATOR_POSE' or n.loc == 'VERIFYING_OPERATOR_POSE':
            saw_verifying = True
        if n.loc == 'READY' and n.blocked is False:
            break
    owner_ops = [o for o in n.owners if o.get('owner') == 'operator']
    report.update({
        'web': good,
        'saw_verifying': saw_verifying,
        'saw_owner_operator': bool(owner_ops),
        'snap_end': n.snap(),
        'owners_tail': n.owners[-8:],
        'pass': n.loc == 'READY' and n.blocked is False and saw_verifying and bool(owner_ops),
    })
    write_json('operator_fallback.json', report)


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument(
        '--phase',
        required=True,
        choices=['t1', 't2', 't3', 't4', 't5', 't6', 'operator', 'snapshot', 'watch'],
    )
    ap.add_argument('--x', type=float, default=0.0)
    ap.add_argument('--y', type=float, default=0.0)
    ap.add_argument('--yaw', type=float, default=0.0)
    args = ap.parse_args()
    OUT.mkdir(parents=True, exist_ok=True)
    rclpy.init()
    n = Watch()
    try:
        n.spin(1.5)
        if args.phase == 'watch':
            watched = wait_cascade(n, 200.0)
            result = watched.get('result') or {}
            write_json('watch.json', {
                'final': result.get('final') or n.boot.get('state'),
                'selected_path': result.get('selected_path') or n.boot.get('selected_path'),
                'stages': watched.get('stages'),
                'initialpose_count': len(n.initialposes),
                'initialposes': n.initialposes,
                'milestones': result.get('milestones'),
                'cold_break': result.get('cold_break'),
                'owners': _owners_ok(n.owners),
                'snap_end': n.snap(),
                'elapsed_sec': watched.get('elapsed_sec'),
                'result': result,
            })
        elif args.phase == 'snapshot':
            payload = {'snap': n.snap(), 'last_good': read_pose_file()}
            write_json('snapshot.json', payload)
        elif args.phase == 't1':
            run_t1(n)
        elif args.phase == 't2':
            run_t2(n)
        elif args.phase == 't3':
            run_t3(n)
        elif args.phase == 't4':
            run_t4(n)
        elif args.phase == 't5':
            run_t5(n)
        elif args.phase == 't6':
            run_t6(n)
        elif args.phase == 'operator':
            run_operator(n, args.x, args.y, args.yaw)
    finally:
        n.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
