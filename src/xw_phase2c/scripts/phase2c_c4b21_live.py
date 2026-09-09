#!/usr/bin/env python3
"""Phase2C-C4B.2.1 targeted live closure: operator recovery + dedicated P2.

Does not change laser 0.38 / AMCL / ORB. Test C (physical move) is a separate call.
"""

from __future__ import annotations

import json
import math
import os
import time
import urllib.request
from pathlib import Path
from typing import Any, Dict, List, Optional

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Int8, String

from xw_interfaces.srv import SetMode

OUT = Path(os.environ.get('C4B21_OUT', '/ros2_ws/bench/phase2c_c4b21_live_2026-09-09'))
WEB = os.environ.get('C4B21_WEB', 'http://127.0.0.1:9000/api/initialpose')
MAP = os.environ.get('C4B21_MAP', 'vp')
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
        super().__init__('c4b21_watch')
        self.boot: Dict[str, Any] = {}
        self.loc = ''
        self.blocked: Optional[bool] = None
        self.status: Optional[int] = None
        self.amcl: Optional[Dict[str, float]] = None
        self.owners: List[Dict[str, Any]] = []
        self.states: List[Dict[str, Any]] = []
        self.loc_states: List[Dict[str, Any]] = []
        self.create_subscription(String, '/xw/boot/status', self._boot, _LATCH)
        self.create_subscription(String, '/xw/localization/phase2c_loc_state', self._loc, _LATCH)
        self.create_subscription(Bool, '/xw/nav/goals_blocked', self._block, _LATCH)
        self.create_subscription(Int8, '/xw/localization_status', self._status, _LATCH)
        self.create_subscription(PoseWithCovarianceStamped, 'amcl_pose', self._amcl, _AMCL)
        self.create_subscription(String, '/xw/localization/initialpose_owner', self._owner, _LATCH)
        self.create_subscription(String, '/xw/boot/result', self._result, 20)
        self.create_subscription(PoseWithCovarianceStamped, '/initialpose', self._ipose, 20)
        self.results: List[Dict[str, Any]] = []
        self.initialposes: List[Dict[str, Any]] = []
        self._mode = self.create_client(SetMode, '/xw/supervisor/set_mode')

    def _boot(self, msg: String) -> None:
        try:
            self.boot = json.loads(msg.data or '{}')
        except json.JSONDecodeError:
            return
        row = {
            't': time.time(),
            'state': self.boot.get('state'),
            'busy': self.boot.get('busy'),
            'sid': self.boot.get('session_id'),
            'path': self.boot.get('selected_path'),
        }
        if not self.states or self.states[-1].get('state') != row['state'] or self.states[-1].get('sid') != row['sid']:
            self.states.append(row)

    def _loc(self, msg: String) -> None:
        self.loc = msg.data or ''
        row = {'t': time.time(), 'loc': self.loc}
        if not self.loc_states or self.loc_states[-1].get('loc') != self.loc:
            self.loc_states.append(row)

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
        }

    def _owner(self, msg: String) -> None:
        try:
            d = json.loads(msg.data or '{}')
        except json.JSONDecodeError:
            d = {'raw': msg.data}
        self.owners.append({'t': time.time(), **d})

    def _result(self, msg: String) -> None:
        try:
            d = json.loads(msg.data or '{}')
        except json.JSONDecodeError:
            d = {'raw': msg.data}
        self.results.append({'t': time.time(), **d} if isinstance(d, dict) else {'t': time.time(), 'raw': d})

    def _ipose(self, msg: PoseWithCovarianceStamped) -> None:
        p = msg.pose.pose
        self.initialposes.append({
            't': time.time(),
            'x': float(p.position.x),
            'y': float(p.position.y),
            'yaw': _yaw(p.orientation),
            'frame': msg.header.frame_id,
        })

    def spin(self, sec: float) -> None:
        end = time.monotonic() + sec
        while time.monotonic() < end and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)

    def wait_until(self, pred, timeout: float, label: str) -> bool:
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout and rclpy.ok():
            self.spin(0.2)
            if pred():
                return True
        self.get_logger().warn(f'timeout {label} boot={self.boot.get("state")} loc={self.loc}')
        return False

    def set_mode(self, mode: int, payload: str = '') -> Dict[str, Any]:
        if not self._mode.wait_for_service(timeout_sec=10.0):
            return {'ok': False, 'error': 'set_mode_unavailable'}
        req = SetMode.Request()
        req.mode = int(mode)
        req.payload_json = payload
        fut = self._mode.call_async(req)
        t0 = time.monotonic()
        while time.monotonic() - t0 < 15.0 and not fut.done():
            rclpy.spin_once(self, timeout_sec=0.1)
        res = fut.result()
        return {'ok': bool(res and res.success), 'message': getattr(res, 'message', '')}

    def snap(self) -> Dict[str, Any]:
        return {
            'boot': self.boot.get('state'),
            'path': self.boot.get('selected_path'),
            'busy': self.boot.get('busy'),
            'sid': self.boot.get('session_id'),
            'loc': self.loc,
            'blocked': self.blocked,
            'loc_status': self.status,
            'amcl': self.amcl,
            'owner': self.owners[-1] if self.owners else None,
        }


def read_pose_file() -> Dict[str, Any]:
    try:
        import yaml
        return yaml.safe_load(POSE_FILE.read_text(encoding='utf-8')) or {}
    except Exception as exc:  # noqa: BLE001
        return {'error': str(exc)}


def main() -> None:
    phase = os.environ.get('C4B21_PHASE', 'operator')
    OUT.mkdir(parents=True, exist_ok=True)
    rclpy.init()
    n = Watch()
    try:
        n.spin(1.5)
        if phase == 'operator':
            run_operator(n)
        elif phase == 'p2':
            run_p2(n)
        elif phase == 'p2session':
            run_p2_session(n)
        elif phase == 'p3':
            run_p3(n)
        else:
            raise SystemExit(f'unknown phase {phase}')
    finally:
        n.destroy_node()
        rclpy.shutdown()


def run_operator(n: Watch) -> None:
    gt = dict(n.amcl or {})
    report: Dict[str, Any] = {'gt_before': gt, 'snap0': n.snap()}
    n.set_mode(0)
    n.spin(1.0)
    # New session with Reloc held down by the host killer. Stale last_good will P2-reject.
    started = n.set_mode(2, json.dumps({'map_name': MAP}))
    report['nav_start'] = started
    ok = n.wait_until(
        lambda: n.loc == 'NEED_OPERATOR' and n.blocked is True and not n.boot.get('busy'),
        80.0,
        'NEED_OPERATOR',
    )
    report['need_operator'] = {
        'ok': ok,
        'snap': n.snap(),
        'states': n.states[-12:],
        'loc_states': n.loc_states[-8:],
    }
    if not ok:
        (OUT / 'operator.json').write_text(json.dumps(report, indent=2, default=str), encoding='utf-8')
        print(json.dumps(report, indent=2, default=str))
        return

    # Wrong pose must not READY on latch-3 clear.
    bad = web_initialpose(12.0, 9.0, 0.0)
    t_bad = time.time()
    saw_verifying = False
    ready_after_bad = False
    t0 = time.monotonic()
    while time.monotonic() - t0 < 16.0:
        n.spin(0.25)
        if n.boot.get('state') == 'VERIFYING_OPERATOR_POSE' or n.loc == 'VERIFYING_OPERATOR_POSE':
            saw_verifying = True
        if n.loc == 'READY' and n.blocked is False:
            ready_after_bad = True
            break
    report['wrong_pose'] = {
        'web': bad,
        'saw_verifying': saw_verifying,
        'ready': ready_after_bad,
        'elapsed_sec': time.time() - t_bad,
        'snap': n.snap(),
        'owners_tail': n.owners[-6:],
        'pass': (not ready_after_bad) and n.blocked is True and n.loc == 'NEED_OPERATOR',
    }

    good = web_initialpose(float(gt.get('x', 0.4)), float(gt.get('y', -0.63)), float(gt.get('yaw', 0.0)))
    saw_verifying = False
    t0 = time.monotonic()
    while time.monotonic() - t0 < 25.0:
        n.spin(0.25)
        if n.boot.get('state') == 'VERIFYING_OPERATOR_POSE' or n.loc == 'VERIFYING_OPERATOR_POSE':
            saw_verifying = True
        if n.loc == 'READY' and n.blocked is False:
            break
    owner_ops = [o for o in n.owners if o.get('owner') == 'operator']
    report['good_pose'] = {
        'web': good,
        'saw_verifying': saw_verifying,
        'saw_owner_operator': bool(owner_ops),
        'snap': n.snap(),
        'owners_tail': n.owners[-8:],
        'states': n.states[-10:],
        'loc_states': n.loc_states[-8:],
        'pass': n.loc == 'READY' and n.blocked is False and saw_verifying and bool(owner_ops),
    }
    report['pass'] = bool(report['need_operator']['ok'] and report['wrong_pose']['pass'] and report['good_pose']['pass'])
    (OUT / 'operator.json').write_text(json.dumps(report, indent=2, default=str), encoding='utf-8')
    print(json.dumps(report, indent=2, default=str))


def run_p2(n: Watch) -> None:
    report: Dict[str, Any] = {'snap0': n.snap()}
    before = read_pose_file()
    report['file_before'] = {k: before.get(k) for k in ('map_name', 'map_hash', 'timestamp', 'x', 'y', 'yaw', 'quality')}
    # Wait for a fresh writer matching current AMCL.
    wrote = False
    t0 = time.monotonic()
    while time.monotonic() - t0 < 40.0:
        n.spin(1.0)
        cur = read_pose_file()
        amcl = n.amcl or {}
        if not amcl or n.status != 0:
            continue
        try:
            dx = math.hypot(float(cur.get('x', 999)) - amcl['x'], float(cur.get('y', 999)) - amcl['y'])
            age = time.time() - float(cur.get('timestamp') or 0)
        except (TypeError, ValueError):
            continue
        if dx < 0.35 and age < 60.0 and float(cur.get('quality') or 0) >= 0.35 and cur.get('map_name') == MAP:
            wrote = True
            report['file_written'] = cur
            report['amcl_at_write'] = amcl
            report['pose_delta_m'] = dx
            break
    report['writer_ok'] = wrote
    if not wrote:
        report['pass'] = False
        report['reason'] = 'no_fresh_last_good'
        (OUT / 'p2.json').write_text(json.dumps(report, indent=2, default=str), encoding='utf-8')
        print(json.dumps(report, indent=2, default=str))
        return

    n.set_mode(0)
    n.spin(1.5)
    started = n.set_mode(2, json.dumps({'map_name': MAP}))
    report['nav_start'] = started
    n.wait_until(lambda: n.boot.get('state') == 'READY' or (n.loc == 'NEED_OPERATOR' and not n.boot.get('busy')), 90.0, 'p2_session')
    n.spin(1.0)
    result = n.results[-1] if n.results else {}
    stages = result.get('stages') or []
    p2 = next((s for s in stages if s.get('stage') == 'P2'), {})
    p3s = [s for s in stages if s.get('stage') == 'P3']
    report['snap_end'] = n.snap()
    report['states'] = n.states[-16:]
    report['boot_result'] = result
    report['p2_score'] = p2.get('score', p2.get('laser_score'))
    report['p2_result'] = p2.get('result')
    report['p3_ran'] = bool(p3s)
    report['initialpose_count'] = len(n.initialposes)
    report['initialposes'] = n.initialposes[-4:]
    report['pass'] = (
        n.boot.get('state') == 'READY'
        and n.boot.get('selected_path') == 'P2'
        and n.blocked is False
        and n.loc == 'READY'
    )
    (OUT / 'p2.json').write_text(json.dumps(report, indent=2, default=str), encoding='utf-8')
    print(json.dumps(report, indent=2, default=str))


def run_p2_session(n: Watch) -> None:
    """Start NAV from an already laser_verified last_good. No web pose, no blind seed."""
    before = read_pose_file()
    report: Dict[str, Any] = {
        'file_written': before,
        'laser_verified': bool(before.get('laser_verified')),
        'snap0': n.snap(),
    }
    if not before.get('laser_verified'):
        report['pass'] = False
        report['reason'] = 'last_good_not_laser_verified'
        (OUT / 'p2_session.json').write_text(json.dumps(report, indent=2, default=str), encoding='utf-8')
        print(json.dumps(report, indent=2, default=str))
        return
    n.results.clear()
    n.initialposes.clear()
    n.set_mode(0)
    n.spin(1.5)
    started = n.set_mode(2, json.dumps({'map_name': MAP}))
    report['nav_start'] = started
    n.wait_until(
        lambda: (not n.boot.get('busy')) and n.boot.get('state') in ('READY', 'UNKNOWN', 'SENSOR_TIMEOUT'),
        90.0,
        'p2_session',
    )
    n.spin(1.0)
    result = n.results[-1] if n.results else {}
    stages = result.get('stages') or []
    p2 = next((s for s in stages if s.get('stage') == 'P2'), {})
    p3s = [s for s in stages if s.get('stage') == 'P3']
    selected = result.get('selected_path') or n.boot.get('selected_path')
    report.update({
        'snap_end': n.snap(),
        'states': n.states[-16:],
        'selected_path': selected,
        'final': result.get('final'),
        'p2_score': p2.get('score', p2.get('laser_score')),
        'p2_result': p2.get('result'),
        'p2_reason': p2.get('reason'),
        'p3_ran': bool(p3s),
        'amcl_final': result.get('amcl_pose') or n.amcl,
        'initialpose_count': len(n.initialposes),
        'initialposes': n.initialposes[-4:],
        'stages_brief': _stage_rows(result) if result else [],
    })
    report['pass'] = bool(
        report['laser_verified']
        and selected == 'P2'
        and n.boot.get('state') == 'READY'
        and n.blocked is False
        and not p3s
        and len(n.initialposes) == 1
    )
    (OUT / 'p2_session.json').write_text(json.dumps(report, indent=2, default=str), encoding='utf-8')
    print(json.dumps({k: report[k] for k in (
        'laser_verified', 'selected_path', 'final', 'p2_result', 'p2_score', 'p2_reason',
        'p3_ran', 'amcl_final', 'initialpose_count', 'pass', 'snap_end',
    ) if k in report}, indent=2, default=str))


def _stage_rows(result: Dict[str, Any]) -> List[Dict[str, Any]]:
    rows = []
    for s in result.get('stages') or []:
        rows.append({
            'stage': s.get('stage'),
            'result': s.get('result'),
            'reason': s.get('reason'),
            'score': s.get('score', s.get('laser_score')),
            'candidate_pose': s.get('candidate_pose'),
            'amcl_pose': (s.get('amcl') or {}).get('amcl_pose') if isinstance(s.get('amcl'), dict) else None,
        })
    return rows


def run_p3(n: Watch) -> None:
    """Dedicated relocated P3. Caller must already have moved the robot off last_good=A.

    Does not publish /initialpose. Does not touch last_good. Does not retune 0.38.
    """
    old = read_pose_file()
    report: Dict[str, Any] = {
        'old_last_good': {
            k: old.get(k) for k in ('map_name', 'map_hash', 'timestamp', 'x', 'y', 'yaw', 'quality')
        },
        'snap0': n.snap(),
        'note': 'no web initialpose; no blind seed; last_good file not rewritten before session',
    }
    n.results.clear()
    n.initialposes.clear()
    n.set_mode(0)
    n.spin(1.5)
    started = n.set_mode(2, json.dumps({'map_name': MAP}))
    report['nav_start'] = started
    ok = n.wait_until(
        lambda: (not n.boot.get('busy')) and n.boot.get('state') in ('READY', 'UNKNOWN', 'SENSOR_TIMEOUT'),
        200.0,
        'p3_session',
    )
    n.spin(1.5)
    result = n.results[-1] if n.results else {}
    stages = _stage_rows(result) if result else []
    p2 = next((s for s in (result.get('stages') or []) if s.get('stage') == 'P2'), {})
    p3s = [s for s in (result.get('stages') or []) if s.get('stage') == 'P3']
    selected = (result.get('selected_path') or n.boot.get('selected_path') or '')
    p2_accepted = any(s.get('stage') == 'P2' and s.get('result') == 'READY' for s in (result.get('stages') or []))
    p3_participated = any(s.get('stage') == 'P3' for s in (result.get('stages') or []))
    report.update({
        'settled': ok,
        'snap_end': n.snap(),
        'states': n.states[-20:],
        'loc_states': n.loc_states[-10:],
        'selected_path': selected,
        'final': result.get('final'),
        'stages_brief': stages,
        'p2_score': p2.get('score', p2.get('laser_score')),
        'p2_result': p2.get('result'),
        'p2_reason': p2.get('reason'),
        'p3_scores': [
            {
                'attempt': s.get('attempt'),
                'result': s.get('result'),
                'reason': s.get('reason'),
                'laser_score': s.get('score', s.get('laser_score')),
                'candidate_pose': s.get('candidate_pose'),
            }
            for s in p3s
        ],
        'amcl_final': result.get('amcl_pose') or n.amcl,
        'initialpose_count': len(n.initialposes),
        'initialposes': n.initialposes[-6:],
        'owners_tail': n.owners[-6:],
        'boot_result': result,
        'p2_false_accept': bool(p2_accepted),
        'p3_participated': p3_participated,
    })
    ready = n.boot.get('state') == 'READY' and n.loc == 'READY' and n.blocked is False
    unknown = n.loc == 'NEED_OPERATOR' and n.blocked is True and selected in ('', 'NONE', None)
    report['pass'] = bool(
        (not p2_accepted)
        and p3_participated
        and (ready or unknown)
        and len(n.initialposes) <= 1
    )
    report['outcome'] = 'READY' if ready else ('SAFE_UNKNOWN' if unknown else 'FAIL')
    (OUT / 'p3.json').write_text(json.dumps(report, indent=2, default=str), encoding='utf-8')
    print(json.dumps({k: report[k] for k in (
        'old_last_good', 'selected_path', 'final', 'p2_result', 'p2_score', 'p2_reason',
        'p3_scores', 'amcl_final', 'initialpose_count', 'p2_false_accept', 'outcome', 'pass',
        'snap_end',
    )}, indent=2, default=str))


if __name__ == '__main__':
    main()
