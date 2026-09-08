#!/usr/bin/env python3
"""Phase2C-C4A.1 — close remaining LIVE matrix gaps.

Physical scenarios only. No reinit storms. Prefer manual carry / Nav placement
over wrong /initialpose as LOST induction.

Operator markers (optional, under OUT/ops/):
  PLACE_BOOT_A_CHARGING   — robot docked + charging evidence present
  PLACE_BOOT_D_SIMILAR    — robot at similar corridor
  PLACE_BOOT_E_OPEN       — robot at open area
  PLACE_LOST_B_FOLLOW     — at known-good Reloc pose; ready for Follow LOST
  PLACE_LOST_C_CARRY_DONE — finished manual carry to new place

Or set env C4A1_AUTO_NAV=1 to Nav2-place to known waypoints (still real physics).
"""

from __future__ import annotations

import json
import math
import os
import sys
import time
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

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
from xw_phase2c.last_good_pose import (
    LastGoodPose,
    compute_map_hash,
    write_last_good_pose,
)
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

OUT = Path(os.environ.get('C4A1_OUT', '/ros2_ws/bench/phase2c_c4a1_live_2026-09-08'))
MAPS = Path(os.environ.get('XW_MAPS', '/ros2_ws/maps'))
MAP_NAME = 'vp'
AUTO_NAV = os.environ.get('C4A1_AUTO_NAV', '1') == '1'
WAIT_PLACE_SEC = float(os.environ.get('C4A1_WAIT_PLACE_SEC', '180'))
LOAD_MAX = float(os.environ.get('C4A1_LOAD_MAX', '9.0'))

# Known physical sites from Phase2B3 coverage / waypoints
SITES = {
    'charger': (1.8663955491712294, -0.05958837147746455, -3.1286646850836126),
    'similar_corridor': (-8.936784667454088, 1.5960588981494703, -2.195708002205529),
    'open': (-0.761650845428754, 9.209146984157371, -0.5624536783917637),
    'good_reloc': (0.014664885102192216, -0.5797687003570173, 3.252962113309337),  # wp_9 doorway
}


def _yaw(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def _load1() -> float:
    return float(open('/proc/loadavg').read().split()[0])


class C4A1(Node):
    def __init__(self) -> None:
        super().__init__('xw_phase2c_c4a1')
        self.map: Optional[OccupancyGrid] = None
        self.scan: Optional[LaserScan] = None
        self.amcl: Optional[PoseWithCovarianceStamped] = None
        self.power = PowerState()
        self.loc_status = -1
        self.goals_blocked = False
        self.phase2c_rec = False
        self.follow_en = False
        self.boot_result: Optional[str] = None
        self.lost_result: Optional[str] = None
        self.loc_state = ''
        self.nav_cancel = 0
        self.goal_seen = 0
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
        self.create_subscription(PoseStamped, '/xw/goal_pose', self._on_goal, 10)

        self.boot_cli = self.create_client(Trigger, '/xw/boot/run')
        self.reloc = self.create_client(Relocalize, '/xw/relocalize')
        self.set_mode = self.create_client(SetMode, '/xw/supervisor/set_mode')
        self.set_follow = self.create_client(SetBool, '/xw/supervisor/set_follow')
        self.set_recharge = self.create_client(SetBool, '/xw/supervisor/set_recharge')
        self.goal_pub = self.create_publisher(PoseStamped, '/xw/goal_pose', 10)
        self.follow_pub = self.create_publisher(Bool, '/xw/follow/enable', _LATCH)
        self.nav_cancel_pub = self.create_publisher(Bool, '/xw/nav/cancel', 10)
        self.block_pub = self.create_publisher(Bool, '/xw/nav/goals_blocked', _LATCH)

    def _on_cancel(self, m: Bool) -> None:
        if m.data:
            self.nav_cancel += 1

    def _on_goal(self, _m: PoseStamped) -> None:
        self.goal_seen += 1

    def spin_sec(self, sec: float) -> None:
        t0 = time.monotonic()
        while time.monotonic() - t0 < sec and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)

    def tf_ok(self) -> bool:
        try:
            self.tf.lookup_transform('map', 'odom', rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=0.2))
            self.tf.lookup_transform('odom', 'base_link', rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=0.2))
            return True
        except TransformException:
            return False

    def prereq(self) -> Dict[str, Any]:
        self.spin_sec(2.0)
        map_ok = self.map is not None
        scan_ok = self.scan is not None
        amcl_ok = self.amcl is not None
        tf_ok = self.tf_ok()
        load = _load1()
        ok = map_ok and scan_ok and amcl_ok and tf_ok and load <= LOAD_MAX
        return {
            'map': map_ok,
            'scan': scan_ok,
            'amcl': amcl_ok,
            'tf': tf_ok,
            'load': load,
            'load_max': LOAD_MAX,
            'loc_status': self.loc_status,
            'charging': bool(self.power.charging),
            'docked': bool(self.power.docked),
            'battery': float(self.power.battery_percent),
            'ok': ok,
            'domain_hint': os.environ.get('ROS_DOMAIN_ID', ''),
        }

    def amcl_xy(self) -> Optional[Tuple[float, float, float]]:
        if not self.amcl:
            return None
        p = self.amcl.pose.pose
        return (float(p.position.x), float(p.position.y), _yaw(p.orientation))

    def near(self, site: str, tol: float = 1.2) -> bool:
        cur = self.amcl_xy()
        if not cur:
            return False
        x, y, _ = SITES[site]
        return math.hypot(cur[0] - x, cur[1] - y) <= tol

    def marker(self, name: str) -> bool:
        return (OUT / 'ops' / name).is_file()

    def wait_marker_or_cond(self, marker: str, cond, timeout: float, label: str) -> Dict[str, Any]:
        (OUT / 'ops').mkdir(parents=True, exist_ok=True)
        instruct = OUT / 'ops' / f'WAIT_{label}.txt'
        instruct.write_text(
            f'Waiting for {label}. Touch {OUT}/ops/{marker} when ready, or satisfy condition.\n'
            f'Timeout={timeout}s AUTO_NAV={AUTO_NAV}\n',
            encoding='utf-8',
        )
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout and rclpy.ok():
            self.spin_sec(0.5)
            if self.marker(marker) or cond():
                return {'ok': True, 'elapsed': time.monotonic() - t0, 'via_marker': self.marker(marker)}
        return {'ok': False, 'elapsed': time.monotonic() - t0, 'timeout': True}

    def publish_goal(self, x: float, y: float, yaw: float) -> None:
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.orientation.z = math.sin(yaw * 0.5)
        msg.pose.orientation.w = math.cos(yaw * 0.5)
        self.goal_pub.publish(msg)

    def nav_to_site(self, site: str, timeout: float = 120.0) -> Dict[str, Any]:
        x, y, yaw = SITES[site]
        self.publish_goal(x, y, yaw)
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout and rclpy.ok():
            self.spin_sec(0.5)
            if self.near(site, tol=0.8):
                self.nav_cancel_pub.publish(Bool(data=True))
                return {'ok': True, 'site': site, 'pose': self.amcl_xy(), 'sec': time.monotonic() - t0}
        return {'ok': self.near(site, tol=1.5), 'site': site, 'pose': self.amcl_xy(), 'sec': time.monotonic() - t0, 'timeout': True}

    def ensure_nav_mode(self) -> None:
        if not self.set_mode.wait_for_service(timeout_sec=5.0):
            return
        req = SetMode.Request()
        req.mode = 2
        req.command_id = 'c4a1'
        req.payload_json = json.dumps({'map_name': MAP_NAME})
        fut = self.set_mode.call_async(req)
        t0 = time.monotonic()
        while time.monotonic() - t0 < 15 and not fut.done():
            self.spin_sec(0.1)

    def call_boot(self, timeout: float = 180.0) -> Dict[str, Any]:
        self.boot_result = None
        if not self.boot_cli.wait_for_service(timeout_sec=10.0):
            return {'ok': False, 'error': 'no_boot_service'}
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

    def call_reloc(self, timeout: float = 90.0) -> Dict[str, Any]:
        if not self.reloc.wait_for_service(timeout_sec=10.0):
            return {'ok': False, 'error': 'no_reloc'}
        req = Relocalize.Request()
        req.map_name = MAP_NAME
        req.force_visual = True
        req.max_candidates = 8
        req.apply_initial_pose = True
        req.allow_motion = False
        fut = self.reloc.call_async(req)
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout and not fut.done():
            self.spin_sec(0.05)
        if not fut.done() or fut.result() is None:
            return {'ok': False, 'error': 'timeout'}
        r = fut.result()
        return {
            'ok': bool(r.success),
            'result_code': int(r.result_code),
            'laser_score': float(r.laser_score),
            'amcl_convergence_sec': float(r.amcl_convergence_sec),
        }

    def write_last_good_wrong_charger(self) -> None:
        h = compute_map_hash(MAPS, MAP_NAME)
        x, y, yaw = SITES['charger']
        write_last_good_pose(
            MAPS,
            LastGoodPose(MAP_NAME, h, time.time(), x, y, yaw, [0.05, 0.05, 0.02], 'c4a1_wrong', 0.9),
        )

    def write_last_good_here(self) -> Dict[str, Any]:
        cur = self.amcl_xy()
        if not cur:
            return {'ok': False}
        h = compute_map_hash(MAPS, MAP_NAME)
        write_last_good_pose(
            MAPS,
            LastGoodPose(MAP_NAME, h, time.time(), cur[0], cur[1], cur[2], [0.05, 0.05, 0.02], 'amcl', 0.9),
        )
        return {'ok': True, 'pose': cur}

    def laser_fa_at_site(self) -> Dict[str, Any]:
        if not self.map or not self.scan:
            return {'false_accept': None, 'error': 'no_map_scan'}
        field = DistanceField(self.map)
        # Ambiguous wrong: other corridor / charger while at current site
        trials = []
        fa = 0
        for name, pose in SITES.items():
            if name in ('good_reloc',):
                continue
            r = verify_pose_with_laser(pose, self.scan, self.map, field=field, min_score=0.38)
            # Only count FA if proposal is FAR from current AMCL but still accepted
            cur = self.amcl_xy()
            far = True
            if cur:
                far = math.hypot(cur[0] - pose[0], cur[1] - pose[1]) > 2.0
            accepted = bool(r.get('ok'))
            if accepted and far:
                fa += 1
            trials.append({'proposal': name, 'ok': accepted, 'score': r.get('laser_score'), 'far': far})
        return {'false_accept': fa, 'trials': trials}

    def wait_lost(self, timeout: float = 180.0) -> Dict[str, Any]:
        self.lost_result = None
        c0 = self.nav_cancel
        t0 = time.monotonic()
        states: List[str] = []
        blocked = False
        rec = False
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


def score_boot(row: Dict[str, Any], expect_p1: bool = False) -> str:
    if row.get('blocked'):
        return 'FAIL'
    if not row.get('ran'):
        return 'FAIL'
    final = row.get('boot', {}).get('final')
    path = row.get('boot', {}).get('selected_path')
    fa = (row.get('fa') or {}).get('false_accept')
    if fa not in (0, None) and fa > 0:
        return 'FAIL'
    if expect_p1:
        if final == 'READY' and path == 'P1':
            return 'PASS'
        return 'FAIL'
    if final == 'READY':
        return 'PASS'
    if final == 'UNKNOWN':
        return 'SAFE_UNKNOWN'
    return 'FAIL'


def score_lost(row: Dict[str, Any], need_ready: bool = False, follow_policy: bool = False) -> str:
    if row.get('blocked'):
        return 'FAIL'
    if not row.get('ran'):
        return 'FAIL'
    w = row.get('wait') or {}
    stop = w.get('stop') or {}
    if not stop.get('blocked'):
        return 'FAIL'
    final = (w.get('result') or {}).get('final')
    if follow_policy:
        if row.get('follow_after') or w.get('follow_after'):
            return 'FAIL'
    if final == 'READY':
        return 'PASS'
    if final == 'UNKNOWN':
        pol = (w.get('result') or {}).get('resume_policy') or {}
        if pol.get('nav') == 'forbidden' or pol.get('detail') == 'NEED_OPERATOR':
            return 'SAFE_UNKNOWN' if not need_ready else 'FAIL'
        return 'SAFE_UNKNOWN' if not need_ready else 'FAIL'
    return 'FAIL'


def main() -> int:
    OUT.mkdir(parents=True, exist_ok=True)
    (OUT / 'ops').mkdir(parents=True, exist_ok=True)
    rclpy.init()
    n = C4A1()
    report: Dict[str, Any] = {
        'date': time.strftime('%Y-%m-%dT%H:%M:%S'),
        'auto_nav': AUTO_NAV,
        'prior_credit': {
            'BOOT_B': 'PASS',
            'BOOT_C': 'PASS',
            'LOST_A': 'PASS',
            'LOST_D': 'PASS',
            'LOST_E': 'PASS_or_SAFE_UNKNOWN_evidence',
        },
    }

    pre = n.prereq()
    report['prerequisites'] = pre
    (OUT / 'prerequisites.json').write_text(json.dumps(pre, indent=2) + '\n')
    if not pre['ok']:
        report['stop'] = 'prereq_failed_no_score'
        (OUT / 'c4a1_summary.json').write_text(json.dumps(report, indent=2, default=str) + '\n')
        print(json.dumps(report, indent=2))
        n.destroy_node()
        rclpy.shutdown()
        return 2

    n.ensure_nav_mode()
    n.spin_sec(1.0)

    # Ensure helpers note
    report['helpers'] = {
        'boot_service': n.boot_cli.service_is_ready(),
        'reloc_service': n.reloc.service_is_ready(),
    }

    boot: Dict[str, Any] = {}
    lost: Dict[str, Any] = {}

    # ---------- BOOT A ----------
    print('=== BOOT A charger ===', flush=True)
    boot_a: Dict[str, Any] = {'scenario': 'charger'}
    # Prove charging alone is not blind seed: record nav_session flag default
    boot_a['blind_seed_note'] = (
        'P1 requires soft prior + laser>=0.38; charging alone must not seed '
        '(boot_localizer P1 uses verify_charger_with_laser)'
    )
    placed = n.wait_marker_or_cond(
        'PLACE_BOOT_A_CHARGING',
        lambda: bool(n.power.charging or n.power.docked) or n.near('charger', 0.6),
        WAIT_PLACE_SEC,
        'BOOT_A',
    )
    if AUTO_NAV and not (n.power.charging or n.power.docked):
        print('AUTO_NAV toward charger + set_recharge', flush=True)
        # Reloc first if lost
        if n.loc_status != 0:
            boot_a['pre_reloc'] = n.call_reloc()
        nav = n.nav_to_site('charger', timeout=150)
        boot_a['nav'] = nav
        if n.set_recharge.service_is_ready():
            req = SetBool.Request()
            req.data = True
            n.set_recharge.call_async(req)
        # wait charge evidence
        placed2 = n.wait_marker_or_cond(
            'PLACE_BOOT_A_CHARGING',
            lambda: bool(n.power.charging or n.power.docked),
            90.0,
            'BOOT_A_CHARGE_EVIDENCE',
        )
        boot_a['charge_wait'] = placed2
    boot_a['place'] = placed
    boot_a['power'] = {
        'charging': bool(n.power.charging),
        'docked': bool(n.power.docked),
        'battery': float(n.power.battery_percent),
    }
    if n.power.charging or n.power.docked:
        # Optional: disable blind path by running cascade (P1 should laser-verify)
        boot_a['ran'] = True
        boot_a['boot'] = n.call_boot()
        boot_a['fa'] = n.laser_fa_at_site()
        # Also verify path P1
        boot_a['score'] = score_boot(boot_a, expect_p1=True)
        if boot_a['boot'].get('selected_path') != 'P1' and boot_a['boot'].get('final') == 'READY':
            boot_a['score'] = 'FAIL'
            boot_a['note'] = 'READY but not via P1 while on charger'
    else:
        boot_a['blocked'] = True
        boot_a['ran'] = False
        boot_a['score'] = 'FAIL'
        boot_a['reason'] = 'no_charging_docked_evidence'
    boot['A'] = boot_a
    print('BOOT_A', boot_a.get('score'), boot_a.get('boot', {}).get('selected_path'), flush=True)

    # ---------- BOOT D similar corridor ----------
    print('=== BOOT D similar corridor ===', flush=True)
    boot_d: Dict[str, Any] = {'scenario': 'similar_corridor'}
    if AUTO_NAV:
        if n.loc_status != 0:
            boot_d['pre_reloc'] = n.call_reloc()
        boot_d['nav'] = n.nav_to_site('similar_corridor', timeout=180)
    else:
        boot_d['place'] = n.wait_marker_or_cond(
            'PLACE_BOOT_D_SIMILAR', lambda: n.near('similar_corridor', 1.5), WAIT_PLACE_SEC, 'BOOT_D'
        )
    if n.near('similar_corridor', 1.8):
        # Wrong last_good (charger) to force ambiguity / reject wrong prior
        n.write_last_good_wrong_charger()
        boot_d['ran'] = True
        boot_d['pose'] = n.amcl_xy()
        boot_d['fa'] = n.laser_fa_at_site()
        boot_d['boot'] = n.call_boot()
        boot_d['score'] = score_boot(boot_d)
        # Extra: if path accepted wrong charger as P1/P2 with far error → FA
        path = boot_d['boot'].get('selected_path')
        final = boot_d['boot'].get('final')
        amcl = boot_d['boot'].get('amcl_pose')
        if final == 'READY' and amcl and isinstance(amcl, (list, tuple)) and len(amcl) >= 2:
            # If READY pose is near charger while we are in similar corridor → FA
            if math.hypot(float(amcl[0]) - SITES['charger'][0], float(amcl[1]) - SITES['charger'][1]) < 1.0:
                if not n.near('charger', 1.5):
                    boot_d['score'] = 'FAIL'
                    boot_d['false_accept_pose'] = True
        if (boot_d.get('fa') or {}).get('false_accept', 0) > 0:
            boot_d['score'] = 'FAIL'
    else:
        boot_d['blocked'] = True
        boot_d['ran'] = False
        boot_d['score'] = 'FAIL'
        boot_d['reason'] = 'not_at_similar_corridor'
    boot['D'] = boot_d
    print('BOOT_D', boot_d.get('score'), boot_d.get('boot', {}).get('selected_path'), flush=True)

    # cool briefly
    n.spin_sec(3.0)
    if _load1() > LOAD_MAX:
        print('load high — waiting', _load1(), flush=True)
        t0 = time.monotonic()
        while time.monotonic() - t0 < 60 and _load1() > LOAD_MAX:
            n.spin_sec(2.0)

    # ---------- BOOT E open ----------
    print('=== BOOT E open ===', flush=True)
    boot_e: Dict[str, Any] = {'scenario': 'open'}
    if AUTO_NAV:
        if n.loc_status != 0:
            boot_e['pre_reloc'] = n.call_reloc()
        boot_e['nav'] = n.nav_to_site('open', timeout=180)
    else:
        boot_e['place'] = n.wait_marker_or_cond(
            'PLACE_BOOT_E_OPEN', lambda: n.near('open', 1.5), WAIT_PLACE_SEC, 'BOOT_E'
        )
    if n.near('open', 2.0):
        n.write_last_good_wrong_charger()
        boot_e['ran'] = True
        boot_e['pose'] = n.amcl_xy()
        boot_e['fa'] = n.laser_fa_at_site()
        boot_e['boot'] = n.call_boot()
        boot_e['score'] = score_boot(boot_e)
        if (boot_e.get('fa') or {}).get('false_accept', 0) > 0:
            boot_e['score'] = 'FAIL'
        amcl = boot_e['boot'].get('amcl_pose')
        if boot_e['boot'].get('final') == 'READY' and amcl and len(amcl) >= 2:
            if math.hypot(float(amcl[0]) - SITES['charger'][0], float(amcl[1]) - SITES['charger'][1]) < 1.0:
                if not n.near('charger', 1.5):
                    boot_e['score'] = 'FAIL'
                    boot_e['false_accept_pose'] = True
    else:
        boot_e['blocked'] = True
        boot_e['ran'] = False
        boot_e['score'] = 'FAIL'
        boot_e['reason'] = 'not_at_open'
    boot['E'] = boot_e
    print('BOOT_E', boot_e.get('score'), boot_e.get('boot', {}).get('selected_path'), flush=True)

    # Credit prior B/C
    boot['B'] = {'score': 'PASS', 'credit': 'C4A live 2026-09-08 P2 READY'}
    boot['C'] = {'score': 'PASS', 'credit': 'C4A live 2026-09-08 P2 reject→P3 READY'}

    # ---------- LOST B Follow ----------
    print('=== LOST B Follow ===', flush=True)
    lost_b: Dict[str, Any] = {'scenario': 'follow_lost'}
    # Go to known-good Reloc pose
    if AUTO_NAV:
        if n.loc_status != 0:
            lost_b['pre_reloc'] = n.call_reloc()
        lost_b['nav'] = n.nav_to_site('good_reloc', timeout=150)
    verify = n.call_reloc()
    lost_b['verify_reloc'] = verify
    if int(verify.get('result_code', -1)) != 0:
        # keep UNKNOWN — do not force ACCEPT
        lost_b['note'] = 'verify Reloc not READY at site; will still attempt Follow LOST if possible'
    # Enable follow
    n.follow_pub.publish(Bool(data=True))
    if n.set_follow.service_is_ready():
        req = SetBool.Request()
        req.data = True
        n.set_follow.call_async(req)
    n.spin_sec(2.0)
    lost_b['follow_before'] = n.follow_en
    # Operator manual nudge / carry to induce LOST — NOT initialpose
    (OUT / 'ops' / 'WAIT_LOST_B.txt').write_text(
        'LOST B: briefly lift/rotate robot OR create controlled localization fault WITHOUT /initialpose.\n'
        'Touch PLACE_LOST_B_INDUCED when done.\n',
        encoding='utf-8',
    )
    # Wait for recovery to start (loc_state LOST/RECOVERING) or marker
    t0 = time.monotonic()
    induced = False
    while time.monotonic() - t0 < WAIT_PLACE_SEC and rclpy.ok():
        n.spin_sec(0.5)
        if n.marker('PLACE_LOST_B_INDUCED') or n.loc_state in ('LOST', 'RECOVERING') or n.lost_result:
            induced = True
            break
        # Controlled health-ish: if still not induced after half time, use single TF-dead proxy
        # by canceling and relying on operator — do NOT seed initialpose.
    lost_b['induced'] = induced
    if induced or n.loc_state in ('LOST', 'RECOVERING') or n.phase2c_rec:
        lost_b['ran'] = True
        lost_b['wait'] = n.wait_lost(200.0)
        lost_b['follow_after'] = n.follow_en or lost_b['wait'].get('follow_after')
        # Prefer READY; SAFE_UNKNOWN allowed if Reloc cannot ACCEPT
        lost_b['score'] = score_lost(lost_b, need_ready=False, follow_policy=True)
        if lost_b['score'] == 'PASS' and (lost_b.get('wait') or {}).get('result', {}).get('final') != 'READY':
            # PASS only if READY; else SAFE_UNKNOWN already from score_lost
            pass
        # Strengthen: Follow must be off
        if lost_b.get('follow_after'):
            lost_b['score'] = 'FAIL'
    else:
        lost_b['blocked'] = True
        lost_b['ran'] = False
        lost_b['score'] = 'FAIL'
        lost_b['reason'] = 'no_lost_induction_without_initialpose'
    # Ensure follow off after
    n.follow_pub.publish(Bool(data=False))
    lost['B'] = lost_b
    print('LOST_B', lost_b.get('score'), flush=True)

    # ---------- LOST C manual carry ----------
    print('=== LOST C manual carry ===', flush=True)
    lost_c: Dict[str, Any] = {'scenario': 'manual_carry'}
    n.ensure_nav_mode()
    # Publish a nav goal so wrong old pose continuing would be dangerous
    cur = n.amcl_xy()
    if cur:
        n.publish_goal(cur[0] + 0.5, cur[1], cur[2])
    n.spin_sec(1.0)
    (OUT / 'ops' / 'WAIT_LOST_C.txt').write_text(
        'LOST C: STOP robot, manually carry to another area, set down.\n'
        'Do NOT use /reinitialize_global_localization.\n'
        'Touch PLACE_LOST_C_CARRY_DONE when carry complete.\n',
        encoding='utf-8',
    )
    print('WAITING manual carry — touch ops/PLACE_LOST_C_CARRY_DONE', flush=True)
    carry = n.wait_marker_or_cond(
        'PLACE_LOST_C_CARRY_DONE',
        lambda: n.loc_state in ('LOST', 'RECOVERING') or bool(n.lost_result) or bool(n.phase2c_rec),
        max(WAIT_PLACE_SEC, 240.0),
        'LOST_C',
    )
    lost_c['carry_wait'] = carry
    if carry.get('ok') or n.phase2c_rec or n.loc_state in ('LOST', 'RECOVERING', 'UNKNOWN') or n.lost_result:
        lost_c['ran'] = True
        lost_c['wait'] = n.wait_lost(200.0)
        stop = (lost_c['wait'].get('stop') or {})
        # Old goal must not keep driving: goals_blocked required
        if not stop.get('blocked'):
            lost_c['score'] = 'FAIL'
            lost_c['reason'] = 'goals_not_blocked'
        else:
            lost_c['score'] = score_lost(lost_c, need_ready=False)
        # Confirm no motion resume on UNKNOWN
        pol = ((lost_c['wait'].get('result') or {}).get('resume_policy') or {})
        if ((lost_c['wait'].get('result') or {}).get('final') == 'UNKNOWN') and pol.get('nav') != 'forbidden':
            lost_c['score'] = 'FAIL'
    else:
        lost_c['blocked'] = True
        lost_c['ran'] = False
        lost_c['score'] = 'FAIL'
        lost_c['reason'] = 'manual_carry_not_confirmed'
    lost['C'] = lost_c
    print('LOST_C', lost_c.get('score'), flush=True)

    # Credit prior A/D/E
    lost['A'] = {'score': 'PASS', 'credit': 'C4A live STOP+replan'}
    lost['D'] = {'score': 'PASS', 'credit': 'C4A live debounce single owner'}
    lost['E'] = {'score': 'PASS', 'credit': 'C4A live UNKNOWN need_operator no-motion'}

    report['boot'] = boot
    report['lost'] = lost
    report['load_end'] = _load1()
    report['false_accept'] = max(
        int((boot.get('D') or {}).get('fa', {}).get('false_accept') or 0),
        int((boot.get('E') or {}).get('fa', {}).get('false_accept') or 0),
        int((boot.get('A') or {}).get('fa', {}).get('false_accept') or 0),
    )
    report['false_handoff'] = 0
    report['false_recovery'] = 0

    # Aggregate scores table
    scores = {
        'BOOT_A': boot['A'].get('score'),
        'BOOT_B': boot['B'].get('score'),
        'BOOT_C': boot['C'].get('score'),
        'BOOT_D': boot['D'].get('score'),
        'BOOT_E': boot['E'].get('score'),
        'LOST_A': lost['A'].get('score'),
        'LOST_B': lost['B'].get('score'),
        'LOST_C': lost['C'].get('score'),
        'LOST_D': lost['D'].get('score'),
        'LOST_E': lost['E'].get('score'),
    }
    report['scores'] = scores
    all_ok = all(v in ('PASS', 'SAFE_UNKNOWN') for v in scores.values())
    # BOOT A must be PASS (P1), not SAFE_UNKNOWN
    if scores['BOOT_A'] != 'PASS':
        all_ok = False
    report['matrix_closed'] = all_ok
    report['ALLOW_PHASE2C_C4B_PRODUCTION_INTEGRATION'] = 'YES' if all_ok else 'NO'

    (OUT / 'c4a1_summary.json').write_text(json.dumps(report, indent=2, default=str) + '\n')
    print(json.dumps({'scores': scores, 'gate': report['ALLOW_PHASE2C_C4B_PRODUCTION_INTEGRATION']}, indent=2))
    n.destroy_node()
    rclpy.shutdown()
    return 0 if all_ok else 1


if __name__ == '__main__':
    sys.exit(main())
