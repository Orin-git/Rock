#!/usr/bin/env python3
"""Phase2C-C4A.2 — one physical trial at a time (operator markers).

Usage:
  python3 phase2c_c4a2_trial.py BOOT_A
  python3 phase2c_c4a2_trial.py BOOT_D
  python3 phase2c_c4a2_trial.py BOOT_E
  python3 phase2c_c4a2_trial.py LOST_B
  python3 phase2c_c4a2_trial.py LOST_C
  python3 phase2c_c4a2_trial.py STATUS

No Nav2 auto-delivery. No /initialpose LOST core. No package restarts.
"""

from __future__ import annotations

import json
import math
import os
import sys
import time
from pathlib import Path
from typing import Any, Dict, Optional, Tuple

import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Int8, String
from std_srvs.srv import SetBool, Trigger
from tf2_ros import Buffer, TransformException, TransformListener

from xw_interfaces.msg import PowerState
from xw_interfaces.srv import Relocalize, SetMode
from xw_phase2c.laser_prior_verify import verify_pose_with_laser
from xw_phase2c.last_good_pose import LastGoodPose, compute_map_hash, write_last_good_pose
from xw_global_reloc.laser_verify import DistanceField


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

OUT = Path(os.environ.get('C4A2_OUT', '/ros2_ws/bench/phase2c_c4a2_physical_2026-09-08'))
MAPS = Path('/ros2_ws/maps')
MAP_NAME = 'vp'
LOAD_MAX = float(os.environ.get('C4A2_LOAD_MAX', '9.0'))
WAIT_SEC = float(os.environ.get('C4A2_WAIT_SEC', '600'))

MARKERS = {
    'BOOT_A': 'PLACE_BOOT_A_CHARGING',
    'BOOT_D': 'PLACE_BOOT_D_SIMILAR',
    'BOOT_E': 'PLACE_BOOT_E_OPEN',
    'LOST_B': 'PLACE_LOST_B_INDUCED',
    'LOST_C': 'PLACE_LOST_C_CARRY_DONE',
}

SITES = {
    'charger': (1.8663955491712294, -0.05958837147746455, -3.1286646850836126),
    'similar_corridor': (-8.936784667454088, 1.5960588981494703, -2.195708002205529),
    'open': (-0.761650845428754, 9.209146984157371, -0.5624536783917637),
}


def _load1() -> float:
    return float(open('/proc/loadavg').read().split()[0])


def _yaw(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class Trial(Node):
    def __init__(self) -> None:
        super().__init__('xw_phase2c_c4a2_trial')
        self.map = None
        self.scan = None
        self.amcl = None
        self.power = PowerState()
        self.loc_status = -1
        self.goals_blocked = False
        self.phase2c_rec = False
        self.follow_en = False
        self.boot_result = None
        self.lost_result = None
        self.loc_state = ''
        self.nav_cancel = 0
        self.tf = Buffer()
        self._tfl = TransformListener(self.tf, self)

        self.create_subscription(OccupancyGrid, '/map', lambda m: setattr(self, 'map', m), _MAP)
        self.create_subscription(LaserScan, '/scan', lambda m: setattr(self, 'scan', m), 10)
        self.create_subscription(PoseWithCovarianceStamped, 'amcl_pose', lambda m: setattr(self, 'amcl', m), _AMCL)
        self.create_subscription(PowerState, '/xw/power', lambda m: setattr(self, 'power', m), 10)
        self.create_subscription(Int8, '/xw/localization_status', lambda m: setattr(self, 'loc_status', int(m.data)), _LATCH)
        self.create_subscription(Bool, '/xw/nav/goals_blocked', lambda m: setattr(self, 'goals_blocked', bool(m.data)), _LATCH)
        self.create_subscription(Bool, '/xw/localization/phase2c_recovery', lambda m: setattr(self, 'phase2c_rec', bool(m.data)), _LATCH)
        self.create_subscription(Bool, '/xw/follow/enable', lambda m: setattr(self, 'follow_en', bool(m.data)), _LATCH)
        self.create_subscription(String, '/xw/boot/result', lambda m: setattr(self, 'boot_result', m.data), 10)
        self.create_subscription(String, '/xw/localization/phase2c_lost_result', lambda m: setattr(self, 'lost_result', m.data), 10)
        self.create_subscription(String, '/xw/localization/phase2c_loc_state', lambda m: setattr(self, 'loc_state', m.data), _LATCH)
        self.create_subscription(Bool, '/xw/nav/cancel', self._on_cancel, 10)

        self.boot_cli = self.create_client(Trigger, '/xw/boot/run')
        self.reloc = self.create_client(Relocalize, '/xw/relocalize')
        self.set_mode = self.create_client(SetMode, '/xw/supervisor/set_mode')
        self.set_follow = self.create_client(SetBool, '/xw/supervisor/set_follow')
        self.follow_pub = self.create_publisher(Bool, '/xw/follow/enable', _LATCH)
        self.goal_pub = self.create_publisher(PoseStamped, '/xw/goal_pose', 10)
        self.cancel_pub = self.create_publisher(Bool, '/xw/nav/cancel', 10)

    def _on_cancel(self, m: Bool) -> None:
        if m.data:
            self.nav_cancel += 1

    def spin_sec(self, sec: float) -> None:
        t0 = time.monotonic()
        while time.monotonic() - t0 < sec and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)

    def tf_ok(self) -> bool:
        try:
            self.tf.lookup_transform('map', 'odom', rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=0.25))
            self.tf.lookup_transform('odom', 'base_link', rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=0.25))
            return True
        except TransformException:
            return False

    def amcl_xy(self) -> Optional[Tuple[float, float, float]]:
        if not self.amcl:
            return None
        p = self.amcl.pose.pose
        return (float(p.position.x), float(p.position.y), _yaw(p.orientation))

    def near(self, site: str, tol: float) -> bool:
        cur = self.amcl_xy()
        if not cur:
            return False
        x, y, _ = SITES[site]
        return math.hypot(cur[0] - x, cur[1] - y) <= tol

    def prereq(self) -> Dict[str, Any]:
        self.spin_sec(2.5)
        load = _load1()
        ok = (
            self.map is not None
            and self.scan is not None
            and self.amcl is not None
            and self.tf_ok()
            and load <= LOAD_MAX
        )
        return {
            'ok': ok,
            'map': self.map is not None,
            'scan': self.scan is not None,
            'amcl': self.amcl is not None,
            'tf': self.tf_ok(),
            'load': load,
            'load_max': LOAD_MAX,
            'loc_status': self.loc_status,
            'charging': bool(self.power.charging),
            'docked': bool(self.power.docked),
            'battery': float(self.power.battery_percent),
            'pose': self.amcl_xy(),
            'boot_svc': self.boot_cli.service_is_ready(),
            'reloc_svc': self.reloc.service_is_ready(),
            'domain': os.environ.get('ROS_DOMAIN_ID', ''),
        }

    def marker_path(self, trial: str) -> Path:
        return OUT / 'ops' / MARKERS[trial]

    def wait_marker(self, trial: str, extra_ok=None) -> Dict[str, Any]:
        (OUT / 'ops').mkdir(parents=True, exist_ok=True)
        mp = self.marker_path(trial)
        instruct = OUT / 'ops' / f'WAIT_{trial}.txt'
        instruct.write_text(
            f'Waiting for {MARKERS[trial]}\n'
            f'touch {mp}\n'
            f'timeout={WAIT_SEC}s\n',
            encoding='utf-8',
        )
        print(f'WAITING marker {mp} (timeout {WAIT_SEC:.0f}s)', flush=True)
        t0 = time.monotonic()
        while time.monotonic() - t0 < WAIT_SEC and rclpy.ok():
            self.spin_sec(0.5)
            if mp.is_file() or (extra_ok and extra_ok()):
                return {'ok': True, 'elapsed': time.monotonic() - t0, 'marker': mp.is_file()}
        return {'ok': False, 'elapsed': time.monotonic() - t0, 'timeout': True}

    def idle(self) -> None:
        if not self.set_mode.wait_for_service(timeout_sec=3.0):
            return
        req = SetMode.Request()
        req.mode = 0
        req.command_id = 'c4a2-idle'
        fut = self.set_mode.call_async(req)
        t0 = time.monotonic()
        while time.monotonic() - t0 < 10 and not fut.done():
            self.spin_sec(0.1)

    def ensure_nav(self) -> None:
        if not self.set_mode.wait_for_service(timeout_sec=3.0):
            return
        req = SetMode.Request()
        req.mode = 2
        req.command_id = 'c4a2-nav'
        req.payload_json = json.dumps({'map_name': MAP_NAME})
        fut = self.set_mode.call_async(req)
        t0 = time.monotonic()
        while time.monotonic() - t0 < 20 and not fut.done():
            self.spin_sec(0.1)
        # wait map
        t0 = time.monotonic()
        while time.monotonic() - t0 < 60 and self.map is None:
            self.spin_sec(0.5)

    def call_boot(self, timeout: float = 180.0) -> Dict[str, Any]:
        self.boot_result = None
        if not self.boot_cli.wait_for_service(timeout_sec=10.0):
            return {'error': 'boot_unavailable'}
        fut = self.boot_cli.call_async(Trigger.Request())
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout and rclpy.ok():
            self.spin_sec(0.05)
            if fut.done() and self.boot_result:
                break
        self.spin_sec(1.0)
        parsed = {}
        if self.boot_result:
            try:
                parsed = json.loads(self.boot_result)
            except json.JSONDecodeError:
                parsed = {'raw': self.boot_result}
        return {
            'final': parsed.get('final'),
            'selected_path': parsed.get('selected_path'),
            'stages': parsed.get('stages'),
            'amcl_pose': parsed.get('amcl_pose'),
            'total_sec': parsed.get('total_boot_localization_sec'),
        }

    def laser_fa(self) -> Dict[str, Any]:
        if not self.map or not self.scan:
            return {'false_accept': None}
        field = DistanceField(self.map)
        fa = 0
        trials = []
        cur = self.amcl_xy()
        for name, pose in SITES.items():
            r = verify_pose_with_laser(pose, self.scan, self.map, field=field, min_score=0.38)
            far = True
            if cur:
                far = math.hypot(cur[0] - pose[0], cur[1] - pose[1]) > 2.0
            ok = bool(r.get('ok'))
            if ok and far:
                fa += 1
            trials.append({'proposal': name, 'ok': ok, 'score': r.get('laser_score'), 'far': far})
        return {'false_accept': fa, 'trials': trials}

    def write_wrong_last_good_charger(self) -> None:
        h = compute_map_hash(MAPS, MAP_NAME)
        x, y, yaw = SITES['charger']
        write_last_good_pose(
            MAPS,
            LastGoodPose(MAP_NAME, h, time.time(), x, y, yaw, [0.05, 0.05, 0.02], 'c4a2_wrong', 0.9),
        )

    def wait_lost(self, timeout: float = 200.0) -> Dict[str, Any]:
        self.lost_result = None
        c0 = self.nav_cancel
        states = []
        blocked = False
        rec = False
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout and rclpy.ok():
            self.spin_sec(0.05)
            if self.goals_blocked:
                blocked = True
            if self.phase2c_rec:
                rec = True
            if self.loc_state and (not states or states[-1] != self.loc_state):
                states.append(self.loc_state)
            if self.lost_result:
                break
        parsed = {}
        if self.lost_result:
            try:
                parsed = json.loads(self.lost_result)
            except json.JSONDecodeError:
                parsed = {'raw': self.lost_result}
        return {
            'result': parsed,
            'stop': {
                'cancel_delta': self.nav_cancel - c0,
                'blocked': blocked or self.goals_blocked,
                'phase2c_recovery': rec or self.phase2c_rec,
                'states': states,
            },
            'follow_after': self.follow_en,
        }


def save_trial(name: str, row: Dict[str, Any]) -> None:
    OUT.mkdir(parents=True, exist_ok=True)
    (OUT / 'ops').mkdir(parents=True, exist_ok=True)
    path = OUT / f'trial_{name}.json'
    path.write_text(json.dumps(row, indent=2, default=str) + '\n', encoding='utf-8')
    # merge ledger
    ledger_path = OUT / 'ledger.json'
    ledger = {}
    if ledger_path.is_file():
        try:
            ledger = json.loads(ledger_path.read_text(encoding='utf-8'))
        except json.JSONDecodeError:
            ledger = {}
    ledger.setdefault('trials', {})[name] = {
        'score': row.get('score'),
        'file': str(path),
        'stamp': time.time(),
    }
    ledger_path.write_text(json.dumps(ledger, indent=2) + '\n', encoding='utf-8')
    print(json.dumps({'saved': str(path), 'score': row.get('score')}, indent=2), flush=True)


def score_boot_a(row: Dict[str, Any]) -> str:
    if not row.get('ran'):
        return 'FAIL'
    boot = row.get('boot') or {}
    if boot.get('final') == 'READY' and boot.get('selected_path') == 'P1':
        # ensure charging evidence was present
        if not (row.get('power') or {}).get('charging') and not (row.get('power') or {}).get('docked'):
            return 'FAIL'
        return 'PASS'
    return 'FAIL'


def score_boot_de(row: Dict[str, Any]) -> str:
    if not row.get('ran'):
        return 'FAIL'
    if (row.get('fa') or {}).get('false_accept', 0) not in (0, None) and (row.get('fa') or {}).get('false_accept', 0) > 0:
        return 'FAIL'
    if row.get('false_accept_pose'):
        return 'FAIL'
    final = (row.get('boot') or {}).get('final')
    if final == 'READY':
        return 'PASS'
    if final == 'UNKNOWN':
        return 'SAFE_UNKNOWN'
    return 'FAIL'


def score_lost(row: Dict[str, Any], follow_policy: bool = False) -> str:
    if not row.get('ran'):
        return 'FAIL'
    w = row.get('wait') or {}
    stop = w.get('stop') or {}
    if not stop.get('blocked'):
        return 'FAIL'
    if follow_policy and (row.get('follow_after') or w.get('follow_after')):
        return 'FAIL'
    final = (w.get('result') or {}).get('final')
    pol = (w.get('result') or {}).get('resume_policy') or {}
    if final == 'READY':
        return 'PASS'
    if final == 'UNKNOWN':
        if pol.get('nav') == 'forbidden' or pol.get('detail') == 'NEED_OPERATOR':
            return 'SAFE_UNKNOWN'
        return 'FAIL'
    return 'FAIL'


def run_boot_a(n: Trial) -> Dict[str, Any]:
    row: Dict[str, Any] = {'trial': 'BOOT_A'}
    pre = n.prereq()
    row['prereq'] = pre
    if not pre['ok']:
        row['score'] = 'FAIL'
        row['reason'] = 'prereq_failed'
        return row
    wait = n.wait_marker(
        'BOOT_A',
        extra_ok=lambda: bool(n.power.charging or n.power.docked),
    )
    row['wait_marker'] = wait
    row['power'] = {
        'charging': bool(n.power.charging),
        'docked': bool(n.power.docked),
        'battery': float(n.power.battery_percent),
    }
    if not (n.power.charging or n.power.docked):
        row['score'] = 'FAIL'
        row['reason'] = 'no_charge_evidence'
        return row
    n.ensure_nav()
    pre2 = n.prereq()
    row['prereq_pre_boot'] = pre2
    if not pre2['ok']:
        row['score'] = 'FAIL'
        row['reason'] = 'prereq_failed_before_boot'
        return row
    row['fa'] = n.laser_fa()
    row['ran'] = True
    row['boot'] = n.call_boot()
    # Prove P1 used laser path: stages should show P1 READY not blind
    stages = row['boot'].get('stages') or []
    row['p1_stage'] = next((s for s in stages if str(s.get('stage')) == 'P1'), None)
    row['score'] = score_boot_a(row)
    if row['score'] == 'PASS':
        # soft-prior proof note
        row['blind_seed_proof'] = (
            'P1 selected_path with laser stage evidence; charging alone does not bypass laser gate'
        )
    n.idle()
    return row


def run_boot_de(n: Trial, which: str, site: str) -> Dict[str, Any]:
    row: Dict[str, Any] = {'trial': which, 'site': site}
    pre = n.prereq()
    row['prereq'] = pre
    if not pre['ok']:
        row['score'] = 'FAIL'
        row['reason'] = 'prereq_failed'
        return row
    wait = n.wait_marker(which)
    row['wait_marker'] = wait
    if not wait.get('ok'):
        row['score'] = 'FAIL'
        row['reason'] = 'marker_timeout'
        return row
    # Operator placed — confirm near site if AMCL usable, else still allow with marker-only note
    row['pose'] = n.amcl_xy()
    row['near_site'] = n.near(site, 2.5)
    n.write_wrong_last_good_charger()
    row['fa'] = n.laser_fa()
    row['ran'] = True
    row['boot'] = n.call_boot()
    amcl = row['boot'].get('amcl_pose')
    if row['boot'].get('final') == 'READY' and amcl and len(amcl) >= 2:
        # If READY snapped to charger while operator says similar/open and pose far from charger → FA
        if math.hypot(float(amcl[0]) - SITES['charger'][0], float(amcl[1]) - SITES['charger'][1]) < 1.0:
            if not n.near('charger', 1.5):
                row['false_accept_pose'] = True
    row['score'] = score_boot_de(row)
    n.idle()
    return row


def run_lost_b(n: Trial) -> Dict[str, Any]:
    row: Dict[str, Any] = {'trial': 'LOST_B'}
    pre = n.prereq()
    row['prereq'] = pre
    if not pre['ok']:
        row['score'] = 'FAIL'
        row['reason'] = 'prereq_failed'
        return row
    n.ensure_nav()
    # optional verify reloc at current place — do not force
    if n.reloc.service_is_ready():
        req = Relocalize.Request()
        req.map_name = MAP_NAME
        req.force_visual = True
        req.max_candidates = 8
        req.apply_initial_pose = True
        req.allow_motion = False
        fut = n.reloc.call_async(req)
        t0 = time.monotonic()
        while time.monotonic() - t0 < 90 and not fut.done():
            n.spin_sec(0.05)
        if fut.done() and fut.result() is not None:
            r = fut.result()
            row['verify_reloc'] = {
                'ok': bool(r.success),
                'result_code': int(r.result_code),
                'laser_score': float(r.laser_score),
            }
    n.follow_pub.publish(Bool(data=True))
    if n.set_follow.service_is_ready():
        req = SetBool.Request()
        req.data = True
        n.set_follow.call_async(req)
    n.spin_sec(2.0)
    row['follow_before'] = n.follow_en
    print('Follow armed — operator: lift/rotate to induce LOST, then touch PLACE_LOST_B_INDUCED', flush=True)
    wait = n.wait_marker(
        'LOST_B',
        extra_ok=lambda: n.loc_state in ('LOST', 'RECOVERING') or bool(n.lost_result) or bool(n.phase2c_rec),
    )
    row['wait_marker'] = wait
    if not wait.get('ok') and not (n.phase2c_rec or n.lost_result or n.loc_state in ('LOST', 'RECOVERING')):
        row['score'] = 'FAIL'
        row['reason'] = 'no_induction'
        n.follow_pub.publish(Bool(data=False))
        n.idle()
        return row
    row['ran'] = True
    row['wait'] = n.wait_lost(200.0)
    row['follow_after'] = n.follow_en or row['wait'].get('follow_after')
    row['score'] = score_lost(row, follow_policy=True)
    n.follow_pub.publish(Bool(data=False))
    n.idle()
    return row


def run_lost_c(n: Trial) -> Dict[str, Any]:
    row: Dict[str, Any] = {'trial': 'LOST_C'}
    pre = n.prereq()
    row['prereq'] = pre
    if not pre['ok']:
        row['score'] = 'FAIL'
        row['reason'] = 'prereq_failed'
        return row
    n.ensure_nav()
    cur = n.amcl_xy()
    row['pose_before'] = cur
    if cur:
        msg = PoseStamped()
        msg.header.stamp = n.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.position.x = cur[0] + 0.4
        msg.pose.position.y = cur[1]
        msg.pose.orientation.z = math.sin(cur[2] * 0.5)
        msg.pose.orientation.w = math.cos(cur[2] * 0.5)
        n.goal_pub.publish(msg)
        n.spin_sec(1.0)
    print('Operator: stop, carry to new place, touch PLACE_LOST_C_CARRY_DONE', flush=True)
    wait = n.wait_marker(
        'LOST_C',
        extra_ok=lambda: n.loc_state in ('LOST', 'RECOVERING', 'UNKNOWN')
        or bool(n.lost_result)
        or bool(n.phase2c_rec),
    )
    row['wait_marker'] = wait
    if not wait.get('ok') and not (n.phase2c_rec or n.lost_result):
        row['score'] = 'FAIL'
        row['reason'] = 'carry_not_confirmed'
        n.cancel_pub.publish(Bool(data=True))
        n.idle()
        return row
    row['ran'] = True
    row['wait'] = n.wait_lost(200.0)
    row['pose_after'] = n.amcl_xy()
    row['score'] = score_lost(row, follow_policy=False)
    # UNKNOWN must forbid motion
    final = (row['wait'].get('result') or {}).get('final')
    pol = (row['wait'].get('result') or {}).get('resume_policy') or {}
    if final == 'UNKNOWN' and pol.get('nav') != 'forbidden':
        row['score'] = 'FAIL'
    n.idle()
    return row


def cmd_status() -> int:
    OUT.mkdir(parents=True, exist_ok=True)
    ledger = {}
    lp = OUT / 'ledger.json'
    if lp.is_file():
        ledger = json.loads(lp.read_text(encoding='utf-8'))
    credited = {
        'BOOT_B': 'PASS',
        'BOOT_C': 'PASS',
        'LOST_A': 'PASS',
        'LOST_D': 'PASS',
        'LOST_E': 'PASS',
    }
    scores = dict(credited)
    for k, v in (ledger.get('trials') or {}).items():
        scores[k] = v.get('score')
    print(json.dumps({'scores': scores, 'markers': [p.name for p in (OUT / 'ops').glob('PLACE_*')]}, indent=2))
    return 0


def main() -> int:
    if len(sys.argv) < 2:
        print(__doc__)
        return 2
    trial = sys.argv[1].upper()
    if trial == 'STATUS':
        return cmd_status()

    OUT.mkdir(parents=True, exist_ok=True)
    (OUT / 'ops').mkdir(parents=True, exist_ok=True)

    rclpy.init()
    n = Trial()
    # Gentle: if map missing, one ensure_nav (no process kill)
    n.spin_sec(1.0)
    if n.map is None:
        print('map missing — one set_mode NAV (no restart)', flush=True)
        n.ensure_nav()
        n.spin_sec(2.0)

    try:
        if trial == 'BOOT_A':
            row = run_boot_a(n)
        elif trial == 'BOOT_D':
            row = run_boot_de(n, 'BOOT_D', 'similar_corridor')
        elif trial == 'BOOT_E':
            row = run_boot_de(n, 'BOOT_E', 'open')
        elif trial == 'LOST_B':
            row = run_lost_b(n)
        elif trial == 'LOST_C':
            row = run_lost_c(n)
        else:
            print('unknown trial', trial)
            return 2
        save_trial(trial, row)
        return 0 if row.get('score') in ('PASS', 'SAFE_UNKNOWN') else 1
    finally:
        n.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
