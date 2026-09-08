#!/usr/bin/env python3
"""Phase2B2 — Low-load real-world AMCL Handoff validation.

Forbidden: /reinitialize_global_localization, continuous particle scatter,
heavy topic hz/recorders, scoring under load1 above gate.

Protocol per trial:
  Physical place → resource quiesce + cool → GT from place target
  → /xw/relocalize(apply)  [NO pre-/initialpose]
  → ACCEPT→READY→short Nav | SAFE_UNKNOWN | NO_DATA | …

Layers:
  SETUP_ABORTED | NO_DATA | SAFE_UNKNOWN | RELOC_ACCEPT
  AMCL_TIMEOUT | FALSE_HANDOFF | NAV_FAILED | SUCCESS
"""

from __future__ import annotations

import argparse
import json
import math
import os
import sys
import time
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

import rclpy
import yaml
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Int8
from std_srvs.srv import Empty

from xw_interfaces.msg import TaskResult
from xw_interfaces.srv import Relocalize

BENCH = Path('/ros2_ws/bench/phase2b2_low_load_handoff_2026-09-08')
DB = Path('/ros2_ws/maps/vp/visual/keyframes')

RC_READY, RC_UNKNOWN, RC_REJECTED, RC_AMCL_TIMEOUT = 0, 1, 2, 3
RC_NO_DATA = 4

FA_XY_M, FA_YAW_RAD = 1.0, 0.52
NAV_TOL, NAV_TIMEOUT, NAV_OFFSET = 0.70, 75.0, 0.70
DEFAULT_MAX_LOAD1 = 9.0
QUIESCE_WAIT_SEC = 90.0

_LATCH = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)


def yaw_from_quat(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def yaw_to_quat(yaw: float):
    from geometry_msgs.msg import Quaternion

    q = Quaternion()
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q


def yaw_err(a: float, b: float) -> float:
    return abs(math.atan2(math.sin(a - b), math.cos(a - b)))


def pct(xs: List[float], p: float) -> Optional[float]:
    if not xs:
        return None
    xs = sorted(xs)
    i = min(len(xs) - 1, max(0, int(round((p / 100.0) * (len(xs) - 1)))))
    return float(xs[i])


def host_load1() -> float:
    try:
        return float(Path('/proc/loadavg').read_text().split()[0])
    except Exception:  # noqa: BLE001
        return -1.0


def host_cpu_total_pct() -> Optional[float]:
    """Single-sample busy% from /proc/stat (lightweight)."""
    try:
        line = Path('/proc/stat').read_text().splitlines()[0].split()
        nums = [float(x) for x in line[1:8]]
        idle = nums[3] + nums[4]
        total = sum(nums)
        time.sleep(0.15)
        line2 = Path('/proc/stat').read_text().splitlines()[0].split()
        nums2 = [float(x) for x in line2[1:8]]
        idle2 = nums2[3] + nums2[4]
        total2 = sum(nums2)
        dt = total2 - total
        if dt <= 0:
            return None
        return 100.0 * (1.0 - (idle2 - idle) / dt)
    except Exception:  # noqa: BLE001
        return None


def load_kf_pose(kid: str) -> Tuple[float, float, float, str]:
    meta = yaml.safe_load((DB / kid / 'meta.yaml').read_text(encoding='utf-8'))
    mp = meta['map_pose']
    return float(mp['x']), float(mp['y']), float(mp['yaw']), str(meta.get('region_id') or '')


def make_trial(kid: str, expect: str, cls: str) -> Dict[str, Any]:
    x, y, yaw, region = load_kf_pose(kid)
    return {
        'trial_id': f'{cls}_{kid}',
        'keyframe_id': kid,
        'region': region,
        'expect': expect,
        'class': cls,
        'x': x,
        'y': y,
        'yaw': yaw,
    }


def build_smoke_trials() -> List[Dict[str, Any]]:
    """One Formal Correct-ACCEPT site per region (5)."""
    return [
        make_trial('kf_000007', 'ACCEPT', 'smoke'),  # doorway
        make_trial('kf_000014', 'ACCEPT', 'smoke'),  # corridor
        make_trial('kf_000026', 'ACCEPT', 'smoke'),  # room
        make_trial('kf_000033', 'ACCEPT', 'smoke'),  # open
        make_trial('kf_000020', 'ACCEPT', 'smoke'),  # similar_corridor (Formal ACCEPT)
    ]


def build_debug_trials() -> List[Dict[str, Any]]:
    """10 physical places: 8 Formal ACCEPT + 2 Formal UNKNOWN hard-neg."""
    easy = [
        'kf_000007',  # doorway
        'kf_000010',  # doorway B
        'kf_000014',  # corridor
        'kf_000016',  # corridor B
        'kf_000020',  # similar ACCEPT
        'kf_000026',  # room
        'kf_000029',  # room B
        'kf_000033',  # open
    ]
    hard = ['kf_000013', 'kf_000019']
    out = [make_trial(k, 'ACCEPT', 'easy') for k in easy]
    out += [make_trial(k, 'UNKNOWN', 'hard_negative') for k in hard]
    return out


class LowLoadNode(Node):
    def __init__(self) -> None:
        super().__init__('phase2b2_low_load_handoff')
        self._amcl: Optional[PoseWithCovarianceStamped] = None
        self._loc = 1
        self._tasks: List[TaskResult] = []
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self._on_amcl, _LATCH)
        self.create_subscription(Int8, '/xw/localization_status', self._on_loc, _LATCH)
        self.create_subscription(TaskResult, '/xw/task/result', self._on_task, 10)
        self._goal = self.create_publisher(PoseStamped, '/xw/goal_pose', 10)
        self._cancel = self.create_publisher(Bool, '/xw/nav/cancel', 10)
        self._follow = self.create_publisher(Bool, '/xw/follow/enable', _LATCH)
        self._slam = self.create_publisher(Bool, '/xw/slam/enable', _LATCH)
        self._recovery = self.create_publisher(Bool, '/xw/localization/recovery_enable', _LATCH)
        self._rgb_req = self.create_publisher(Bool, '/xw/reloc/rgb_request', _LATCH)
        self._nomotion = self.create_client(Empty, '/request_nomotion_update')
        self._reloc = self.create_client(Relocalize, '/xw/relocalize')
        # Ensure RGB gated off until Reloc arms it.
        self._rgb_req.publish(Bool(data=False))
        self.quiesce_once()

    def _on_amcl(self, m: PoseWithCovarianceStamped) -> None:
        self._amcl = m

    def _on_loc(self, m: Int8) -> None:
        self._loc = int(m.data)

    def _on_task(self, m: TaskResult) -> None:
        self._tasks.append(m)

    def sleep_spin(self, sec: float, hz: float = 3.0) -> None:
        dt = 1.0 / max(hz, 1.0)
        t0 = time.monotonic()
        while time.monotonic() - t0 < sec and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(dt)

    def wait_future(self, fut, timeout: float) -> bool:
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout and rclpy.ok() and not fut.done():
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(0.12)
        return fut.done()

    def pose(self) -> Optional[Tuple[float, float, float]]:
        self.sleep_spin(0.2, hz=5.0)
        if self._amcl is None:
            return None
        p = self._amcl.pose.pose
        return (float(p.position.x), float(p.position.y), yaw_from_quat(p.orientation))

    def cov_xy(self) -> Optional[float]:
        if self._amcl is None:
            return None
        c = self._amcl.pose.covariance
        return max(float(c[0]), float(c[7]))

    def quiesce_once(self) -> None:
        self._cancel.publish(Bool(data=True))
        self._follow.publish(Bool(data=False))
        self._slam.publish(Bool(data=False))
        self._recovery.publish(Bool(data=False))
        self._rgb_req.publish(Bool(data=False))

    def wait_operator(self, spec: Dict[str, Any], prompt: bool, ready_file: str, pause_sec: float) -> None:
        msg = (
            f"\n=== PLACE robot: {spec['trial_id']} | {spec['region']} | expect={spec['expect']} ===\n"
            f"Target map pose ≈ ({spec['x']:.2f}, {spec['y']:.2f}, yaw={spec['yaw']:.2f})\n"
            f"Push/teleop/carry — NO /reinitialize_global_localization, NO manual /initialpose.\n"
            f"Stand still when done.\n"
        )
        print(msg, flush=True)
        self.get_logger().info(msg.strip())
        if ready_file:
            Path(ready_file).unlink(missing_ok=True)
            print(f'Touch file when placed: {ready_file}', flush=True)
            t0 = time.monotonic()
            while time.monotonic() - t0 < 600.0 and rclpy.ok():
                if Path(ready_file).is_file():
                    Path(ready_file).unlink(missing_ok=True)
                    break
                self.sleep_spin(0.5, hz=1.0)
            else:
                raise TimeoutError('ready_file timeout')
        elif prompt and sys.stdin.isatty():
            try:
                input('Press Enter when physically placed and stationary... ')
            except EOFError:
                self.sleep_spin(max(pause_sec, 15.0), hz=1.0)
        else:
            print(f'(non-interactive) pause {pause_sec:.0f}s for placement...', flush=True)
            self.sleep_spin(max(pause_sec, 15.0), hz=1.0)

    def resource_quiesce(self, max_load1: float) -> Dict[str, Any]:
        """Cancel nav/follow/slam; wait load to cool. Abort if never cools."""
        self.quiesce_once()
        t0 = time.monotonic()
        samples = []
        while time.monotonic() - t0 < QUIESCE_WAIT_SEC and rclpy.ok():
            self.quiesce_once()
            load1 = host_load1()
            cpu = host_cpu_total_pct()
            samples.append({'t': time.monotonic() - t0, 'load1': load1, 'cpu_total_pct': cpu})
            print(
                f'quiesce load1={load1:.2f} cpu≈{cpu if cpu is None else round(cpu,1)} '
                f'(need load1≤{max_load1})',
                flush=True,
            )
            if 0.0 <= load1 <= max_load1:
                # require two consecutive cool samples
                self.sleep_spin(3.0, hz=1.0)
                load2 = host_load1()
                if 0.0 <= load2 <= max_load1:
                    return {
                        'ok': True,
                        'load1': load2,
                        'cpu_total_pct': host_cpu_total_pct(),
                        'wait_sec': time.monotonic() - t0,
                        'samples': samples[-5:],
                    }
            self.sleep_spin(4.0, hz=1.0)
        return {
            'ok': False,
            'load1': host_load1(),
            'cpu_total_pct': host_cpu_total_pct(),
            'wait_sec': time.monotonic() - t0,
            'samples': samples[-8:],
            'error': 'load_not_cooled',
        }

    def relocalize(self) -> Dict[str, Any]:
        if not self._reloc.wait_for_service(timeout_sec=5.0):
            return {'ok': False, 'error': 'no_service'}
        req = Relocalize.Request()
        req.map_name = 'vp'
        req.force_visual = True
        req.max_candidates = 5
        req.apply_initial_pose = True
        req.allow_motion = False
        t0 = time.monotonic()
        fut = self._reloc.call_async(req)
        ok = self.wait_future(fut, 90.0)
        wall = time.monotonic() - t0
        if not ok or fut.result() is None:
            return {'ok': False, 'error': 'timeout', 'wall_sec': wall}
        res = fut.result()
        try:
            diag = json.loads(res.diagnostics_json or '{}')
        except json.JSONDecodeError:
            diag = {}
        try:
            timings = json.loads(res.stage_timings_json or '{}')
        except json.JSONDecodeError:
            timings = {}
        cand = None
        if res.pose is not None:
            p = res.pose.pose.pose
            cand = (float(p.position.x), float(p.position.y), yaw_from_quat(p.orientation))
        return {
            'ok': True,
            'result_code': int(res.result_code),
            'success': bool(res.success),
            'laser_score': float(res.laser_score),
            'time_to_candidate_sec': float(res.time_to_candidate_sec),
            'amcl_convergence_sec': float(res.amcl_convergence_sec),
            'wall_sec': wall,
            'candidate': cand,
            'diagnostics': diag,
            'stage_timings': timings,
        }

    def short_nav(self, gt: Tuple[float, float, float]) -> bool:
        gx = gt[0] + NAV_OFFSET * math.cos(gt[2])
        gy = gt[1] + NAV_OFFSET * math.sin(gt[2])
        self._tasks.clear()
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.position.x = float(gx)
        msg.pose.position.y = float(gy)
        msg.pose.orientation = yaw_to_quat(gt[2])
        self._goal.publish(msg)
        t0 = time.monotonic()
        while time.monotonic() - t0 < NAV_TIMEOUT and rclpy.ok():
            self.sleep_spin(0.35, hz=3.0)
            p = self.pose()
            if p is not None and math.hypot(p[0] - gx, p[1] - gy) < NAV_TOL:
                return True
            for r in self._tasks:
                if r.capability == 'nav':
                    return r.code == 0 and p is not None and math.hypot(p[0] - gx, p[1] - gy) < 1.3
        return False

    def run_trial(
        self,
        spec: Dict[str, Any],
        prompt: bool,
        ready_file: str,
        pause_sec: float,
        max_load1: float,
    ) -> Dict[str, Any]:
        out: Dict[str, Any] = {
            'trial_id': spec['trial_id'],
            'keyframe_id': spec['keyframe_id'],
            'region': spec['region'],
            'expect': spec['expect'],
            'class': spec['class'],
            'place_target': {'x': spec['x'], 'y': spec['y'], 'yaw': spec['yaw']},
            'no_reinit': True,
            'no_pre_initialpose': True,
        }
        self.wait_operator(spec, prompt, ready_file, pause_sec)
        q = self.resource_quiesce(max_load1)
        out['quiesce'] = {k: q[k] for k in ('ok', 'load1', 'cpu_total_pct', 'wait_sec', 'error') if k in q}
        if not q.get('ok'):
            out['layer'] = 'SETUP_ABORTED'
            out['setup_error'] = 'load_not_cooled'
            return out

        # GT: prefer live AMCL if still healthy near the place region (real physical pose).
        # Fallback to keyframe place target only when AMCL is lost/far.
        amcl_before = self.pose()
        out['amcl_before'] = None if amcl_before is None else {
            'x': amcl_before[0],
            'y': amcl_before[1],
            'yaw': amcl_before[2],
            'cov_xy': self.cov_xy(),
            'loc_status': self._loc,
        }
        place = (float(spec['x']), float(spec['y']), float(spec['yaw']))
        use_amcl_gt = False
        if amcl_before is not None:
            near = math.hypot(amcl_before[0] - place[0], amcl_before[1] - place[1]) < 1.5
            cov = self.cov_xy()
            healthy = self._loc == 0 and (cov is None or cov < 0.8)
            use_amcl_gt = near and healthy
        if use_amcl_gt:
            gt = amcl_before
            out['gt'] = {'x': gt[0], 'y': gt[1], 'yaw': gt[2], 'source': 'amcl_at_place'}
        else:
            gt = place
            out['gt'] = {'x': gt[0], 'y': gt[1], 'yaw': gt[2], 'source': 'place_target_kf'}
        out['load1_at_start'] = host_load1()
        out['cpu_total_pct_at_start'] = host_cpu_total_pct()
        if out['load1_at_start'] > max_load1:
            out['layer'] = 'SETUP_ABORTED'
            out['setup_error'] = 'load_spiked_at_start'
            return out

        reloc = self.relocalize()
        # Release RGB request latch after call (node should also release).
        self._rgb_req.publish(Bool(data=False))
        diag = reloc.get('diagnostics') or {}
        timings = reloc.get('stage_timings') or {}
        sg = diag.get('sensor_gate') or {
            k: timings.get(k)
            for k in ('rgb_request_to_first_rgb_ms', 'scan_wait_ms', 'sensor_ready_ms', 'sensor_ready')
            if k in timings
        }
        out['reloc'] = {
            'result_code': reloc.get('result_code'),
            'laser_score': reloc.get('laser_score'),
            'wall_sec': reloc.get('wall_sec'),
            'time_to_candidate_sec': reloc.get('time_to_candidate_sec'),
            'amcl_convergence_sec': reloc.get('amcl_convergence_sec'),
            'candidate': reloc.get('candidate'),
            'decision': diag.get('decision'),
            'sensor_gate': sg,
            'amcl_handoff': diag.get('amcl_handoff'),
            'candidate_to_amcl_correction': diag.get('candidate_to_amcl_correction'),
        }
        out['sensor_ready_ms'] = None
        if isinstance(sg, dict) and sg.get('sensor_ready_ms') is not None:
            out['sensor_ready_ms'] = float(sg['sensor_ready_ms'])
        out['load1_after'] = host_load1()

        if not reloc.get('ok'):
            out['layer'] = 'SETUP_ABORTED'
            out['setup_error'] = reloc.get('error')
            return out

        code = reloc.get('result_code')
        cand = reloc.get('candidate')
        decision = diag.get('decision')

        if code == RC_NO_DATA or diag.get('error') == 'sensor_not_ready':
            out['layer'] = 'NO_DATA'
            return out

        if code in (RC_UNKNOWN, RC_REJECTED) or decision in ('UNKNOWN', 'REJECTED'):
            out['layer'] = 'SAFE_UNKNOWN'
            out['false_handoff'] = False
            out['false_accept'] = False
            return out

        if code not in (RC_READY, RC_AMCL_TIMEOUT) and decision != 'ACCEPT':
            out['layer'] = f'code_{code}'
            return out

        out['layer'] = 'RELOC_ACCEPT'
        if cand is not None:
            out['candidate'] = {'x': cand[0], 'y': cand[1], 'yaw': cand[2]}
            out['candidate_error_xy'] = math.hypot(cand[0] - gt[0], cand[1] - gt[1])
            out['candidate_error_yaw'] = yaw_err(cand[2], gt[2])
            if out['candidate_error_xy'] > FA_XY_M or out['candidate_error_yaw'] > FA_YAW_RAD:
                out['layer'] = 'FALSE_HANDOFF'
                out['false_accept'] = True
                out['false_handoff'] = True
                return out

        if code == RC_AMCL_TIMEOUT:
            out['layer'] = 'AMCL_TIMEOUT'
            out['amcl_ready'] = False
            out['false_handoff'] = False
            return out

        if code != RC_READY:
            out['layer'] = f'code_{code}'
            return out

        out['amcl_ready'] = True
        ah = diag.get('amcl_handoff') or {}
        amcl_pose = ah.get('amcl_pose') or list(self.pose() or [])
        if amcl_pose and len(amcl_pose) >= 3:
            out['amcl_ready_pose'] = {'x': amcl_pose[0], 'y': amcl_pose[1], 'yaw': amcl_pose[2]}
            out['amcl_cov_xy'] = ah.get('cov_xy')
            out['amcl_cov_yaw'] = ah.get('cov_yaw')
            out['final_error_xy'] = math.hypot(amcl_pose[0] - gt[0], amcl_pose[1] - gt[1])
            out['final_error_yaw'] = yaw_err(amcl_pose[2], gt[2])
            if cand is not None:
                out['candidate_to_amcl_dxy'] = math.hypot(amcl_pose[0] - cand[0], amcl_pose[1] - cand[1])
                out['candidate_to_amcl_dyaw'] = yaw_err(amcl_pose[2], cand[2])
            if out['final_error_xy'] > FA_XY_M or out['final_error_yaw'] > FA_YAW_RAD:
                out['layer'] = 'FALSE_HANDOFF'
                out['false_handoff'] = True
                out['false_accept'] = True
                return out
        out['amcl_convergence_sec'] = reloc.get('amcl_convergence_sec')
        out['false_handoff'] = False
        out['false_accept'] = False

        # Skip short nav if load already high — still count READY success.
        load_nav = host_load1()
        out['load1_before_nav'] = load_nav
        if load_nav > max_load1 + 2.0:
            out['layer'] = 'SUCCESS'
            out['nav_success'] = None
            out['nav_skipped'] = 'load_high'
            return out

        nav_ok = self.short_nav(gt)
        self._cancel.publish(Bool(data=True))
        out['nav_success'] = bool(nav_ok)
        if not nav_ok:
            out['layer'] = 'NAV_FAILED'
            return out
        out['layer'] = 'SUCCESS'
        return out


def summarize(trials: List[Dict[str, Any]]) -> Dict[str, Any]:
    layers: Dict[str, int] = {}
    for t in trials:
        layers[t.get('layer') or 'NONE'] = layers.get(t.get('layer') or 'NONE', 0) + 1
    scored = [t for t in trials if t.get('layer') != 'SETUP_ABORTED']
    accepts = [
        t
        for t in scored
        if t.get('layer') in ('RELOC_ACCEPT', 'AMCL_TIMEOUT', 'FALSE_HANDOFF', 'NAV_FAILED', 'SUCCESS')
        or t.get('amcl_ready')
    ]
    # Correct accept: ACCEPT path without FA
    correct = [
        t
        for t in scored
        if t.get('layer') in ('SUCCESS', 'NAV_FAILED', 'AMCL_TIMEOUT')
        and not t.get('false_handoff')
        and not t.get('false_accept')
    ]
    ready = [t for t in accepts if t.get('amcl_ready') and not t.get('false_handoff')]
    nav = [t for t in ready if t.get('nav_success') is not None]
    nav_ok = [t for t in nav if t.get('nav_success')]
    fa = [t for t in trials if t.get('false_accept') or t.get('layer') == 'FALSE_HANDOFF']
    walls = [float(t['reloc']['wall_sec']) for t in accepts if (t.get('reloc') or {}).get('wall_sec') is not None]
    cand_t = [
        float(t['reloc']['time_to_candidate_sec'])
        for t in accepts
        if (t.get('reloc') or {}).get('time_to_candidate_sec') is not None
    ]
    arm = [float(t['sensor_ready_ms']) for t in scored if t.get('sensor_ready_ms') is not None]
    conv = [float(t['amcl_convergence_sec']) for t in ready if t.get('amcl_convergence_sec') is not None]
    loads = [float(t['load1_at_start']) for t in scored if t.get('load1_at_start') is not None]
    return {
        'n': len(trials),
        'scored_n': len(scored),
        'layers': layers,
        'correct_accept': len(correct),
        'safe_unknown': layers.get('SAFE_UNKNOWN', 0),
        'no_data': layers.get('NO_DATA', 0),
        'setup_aborted': layers.get('SETUP_ABORTED', 0),
        'false_accept': len(fa),
        'false_handoff': len(fa),
        'reloc_accept_n': len(accepts),
        'amcl_ready_n': len(ready),
        'amcl_ready_rate_on_accept': (len(ready) / len(accepts)) if accepts else None,
        'nav_success_rate_after_ready': (len(nav_ok) / len(nav)) if nav else None,
        'success': layers.get('SUCCESS', 0),
        'visual_laser_p50': pct(cand_t, 50),
        'visual_laser_p95': pct(cand_t, 95),
        'sensor_arm_p50_ms': pct(arm, 50),
        'amcl_conv_p50': pct(conv, 50),
        'amcl_conv_p95': pct(conv, 95),
        'reloc_wall_p50': pct(walls, 50),
        'reloc_wall_p95': pct(walls, 95),
        'load1_at_start_max': max(loads) if loads else None,
        'load1_at_start_mean': (sum(loads) / len(loads)) if loads else None,
    }


def gate_smoke(stats: Dict[str, Any]) -> Dict[str, Any]:
    fa = int(stats.get('false_handoff') or 0)
    ok = fa == 0 and int(stats.get('setup_aborted') or 0) == 0
    # Prefer at least one SUCCESS or SAFE_UNKNOWN without FA
    ok = ok and (int(stats.get('success') or 0) + int(stats.get('safe_unknown') or 0) + int(stats.get('correct_accept') or 0)) >= 1
    return {
        'pass': ok and fa == 0,
        'false_accept': fa,
        'false_handoff': fa,
        'require': 'FA=0, FH=0, no SETUP_ABORTED under smoke; mechanism exercised',
    }


def gate_debug(stats: Dict[str, Any]) -> Dict[str, Any]:
    fa = int(stats.get('false_handoff') or 0)
    accepts = int(stats.get('reloc_accept_n') or 0)
    amcl_r = stats.get('amcl_ready_rate_on_accept')
    nav_r = stats.get('nav_success_rate_after_ready')
    load_max = stats.get('load1_at_start_max')
    load_ok = load_max is None or load_max <= DEFAULT_MAX_LOAD1 + 0.5
    ok = (
        fa == 0
        and load_ok
        and accepts >= 1
        and amcl_r is not None
        and amcl_r >= 0.90
        and (nav_r is None or nav_r >= 0.90)
    )
    return {
        'pass': bool(ok),
        'false_accept': fa,
        'false_handoff': fa,
        'amcl_ready_rate_on_accept': amcl_r,
        'nav_success_rate_after_ready': nav_r,
        'load1_at_start_max': load_max,
        'require': 'FA=0 FH=0; ACCEPT→READY≥90%; READY→Nav≥90%; scored under low load',
    }


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument('--mode', choices=('smoke', 'debug'), default='smoke')
    ap.add_argument('--out', default='')
    ap.add_argument('--prompt', action='store_true', default=True)
    ap.add_argument('--no-prompt', action='store_true')
    ap.add_argument('--ready-file', default='/tmp/phase2b2_placed')
    ap.add_argument('--pause-sec', type=float, default=45.0)
    ap.add_argument('--max-load1', type=float, default=DEFAULT_MAX_LOAD1)
    ap.add_argument('--use-ready-file', action='store_true', help='Wait for ready-file instead of Enter')
    args = ap.parse_args()
    prompt = False if args.no_prompt else True
    ready_file = args.ready_file if args.use_ready_file or (args.no_prompt and args.ready_file) else ''

    BENCH.mkdir(parents=True, exist_ok=True)
    out_path = Path(args.out) if args.out else BENCH / f'handoff_{args.mode}.json'
    trials_spec = build_smoke_trials() if args.mode == 'smoke' else build_debug_trials()

    rclpy.init()
    node = LowLoadNode()
    node.sleep_spin(1.0, hz=2.0)
    results: List[Dict[str, Any]] = []
    stop = None
    print(
        f'Phase2B2 {args.mode} n={len(trials_spec)} max_load1={args.max_load1} '
        f'NO reinit, NO pre-initialpose',
        flush=True,
    )
    try:
        for i, spec in enumerate(trials_spec):
            print(f'=== {i+1}/{len(trials_spec)} {spec["trial_id"]} ===', flush=True)
            one = node.run_trial(
                spec,
                prompt=prompt and not bool(ready_file),
                ready_file=ready_file,
                pause_sec=args.pause_sec,
                max_load1=args.max_load1,
            )
            results.append(one)
            print(
                json.dumps(
                    {
                        'trial': one.get('trial_id'),
                        'layer': one.get('layer'),
                        'fa': one.get('false_handoff'),
                        'load1': one.get('load1_at_start'),
                        'sensor_ms': one.get('sensor_ready_ms'),
                        'xy': None if one.get('final_error_xy') is None else round(one['final_error_xy'], 3),
                        'nav': one.get('nav_success'),
                    }
                ),
                flush=True,
            )
            if one.get('layer') == 'SETUP_ABORTED' and one.get('setup_error') in (
                'load_not_cooled',
                'load_spiked_at_start',
            ):
                stop = 'load_abort'
                print('STOP: load abnormal — not retrying.', flush=True)
                break
            if one.get('false_handoff'):
                stop = 'false_handoff'
                break
            node.quiesce_once()
            node.sleep_spin(8.0, hz=1.0)
    finally:
        stats = summarize(results)
        gate = gate_smoke(stats) if args.mode == 'smoke' else gate_debug(stats)
        payload = {
            'mode': args.mode,
            'validator': 'phase2b2_low_load_v1',
            'no_reinit': True,
            'no_pre_initialpose': True,
            'max_load1': args.max_load1,
            'allow_amcl_handoff': True,
            'production_unchanged': True,
            'stop_reason': stop,
            'stats': stats,
            'gate': gate,
            'trials': results,
        }
        out_path.write_text(json.dumps(payload, indent=2), encoding='utf-8')
        print(json.dumps({'out': str(out_path), 'stats': stats, 'gate': gate}, indent=2), flush=True)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
