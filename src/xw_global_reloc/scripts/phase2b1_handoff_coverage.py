#!/usr/bin/env python3
"""Phase2B1 Handoff Coverage Closure validator.

Pre-nav is OUT OF SCOPE: operator places robot (push/teleop/carry).
Trial = GT snapshot → induce unknown → /xw/relocalize(apply) → READY → short Nav.

Layer codes:
  SETUP_FAILED | NO_DATA | RELOC_UNKNOWN | RELOC_ACCEPT
  AMCL_TIMEOUT | AMCL_FALSE_CONVERGENCE | NAV_AFTER_READY_FAILED | SUCCESS
"""

from __future__ import annotations

import argparse
import json
import math
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

BENCH = Path('/ros2_ws/bench/phase2b1_handoff_coverage_2026-09-07')
DB = Path('/ros2_ws/maps/vp/visual/keyframes')

RC_READY, RC_UNKNOWN, RC_REJECTED, RC_AMCL_TIMEOUT = 0, 1, 2, 3
RC_NO_DATA, RC_MAP_HASH = 4, 5

NAV_TOL, NAV_YAW_TOL, NAV_TIMEOUT, NAV_OFFSET = 0.65, 0.35, 120.0, 0.80
SETTLE, COOL = 2.5, 8.0
SETUP_XY_TOL, SETUP_YAW_TOL = 0.80, 0.40
FA_XY_M, FA_YAW_RAD = 1.0, 0.52
OK_XY_M, OK_YAW_RAD = 0.50, 0.35

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


def load_kf_pose(kid: str) -> Tuple[float, float, float, str]:
    meta = yaml.safe_load((DB / kid / 'meta.yaml').read_text(encoding='utf-8'))
    mp = meta['map_pose']
    return float(mp['x']), float(mp['y']), float(mp['yaw']), str(meta.get('region_id') or '')


def build_debug_trials() -> List[Dict[str, Any]]:
    """8 Formal30 Correct-ACCEPT sites (distinct location groups) + 2 hard-negatives.

    Avoid back-to-back near-identical XY with opposite yaw (fake GT yaw under Nav2).
    """
    # One Formal Correct-ACCEPT query per diverse location (5 regions; avoid twin yaw).
    easy_ids = [
        'kf_000007',  # doorway_wp9_A
        'kf_000012',  # doorway_wp9_B (offset)
        'kf_000014',  # corridor_wp2_A
        'kf_000020',  # similar_corridor Formal ACCEPT (easy)
        'kf_000023',  # similar_corridor Formal ACCEPT (easy)
        'kf_000026',  # room_wp5_A
        'kf_000033',  # open_wp6_A
        'kf_000037',  # open_wp6_B
    ]
    # Hard-negatives / Safe UNKNOWN expected (Formal UNKNOWN)
    hard_ids = [
        'kf_000013',  # corridor Formal UNKNOWN
        'kf_000019',  # similar_corridor Formal UNKNOWN
    ]
    out = []
    for kid in easy_ids:
        x, y, yaw, region = load_kf_pose(kid)
        out.append(
            {
                'trial_id': f'easy_{kid}',
                'keyframe_id': kid,
                'region': region,
                'expect': 'ACCEPT',
                'class': 'easy_correct_accept',
                'x': x,
                'y': y,
                'yaw': yaw,
            }
        )
    for kid in hard_ids:
        x, y, yaw, region = load_kf_pose(kid)
        out.append(
            {
                'trial_id': f'hard_{kid}',
                'keyframe_id': kid,
                'region': region,
                'expect': 'UNKNOWN',
                'class': 'hard_negative',
                'x': x,
                'y': y,
                'yaw': yaw,
            }
        )
    return out


def build_formal_trials() -> List[Dict[str, Any]]:
    """≥30: rotate through Formal30-style easy poses + sparse hard-negatives."""
    easy = [
        'kf_000007', 'kf_000008', 'kf_000009', 'kf_000010',
        'kf_000014', 'kf_000015', 'kf_000016', 'kf_000017',
        'kf_000020', 'kf_000021', 'kf_000023', 'kf_000024',
        'kf_000026', 'kf_000027', 'kf_000028', 'kf_000029', 'kf_000030',
        'kf_000032', 'kf_000033', 'kf_000034', 'kf_000035', 'kf_000036', 'kf_000037',
        'kf_000011', 'kf_000012', 'kf_000018', 'kf_000025', 'kf_000031',
    ]
    hard = ['kf_000013', 'kf_000019']
    ids = (easy + hard)[:30]
    out = []
    for i, kid in enumerate(ids):
        x, y, yaw, region = load_kf_pose(kid)
        hard_set = set(hard)
        out.append(
            {
                'trial_id': f'f{i:02d}_{kid}',
                'keyframe_id': kid,
                'region': region,
                'expect': 'UNKNOWN' if kid in hard_set else 'ACCEPT',
                'class': 'hard_negative' if kid in hard_set else 'easy_correct_accept',
                'x': x,
                'y': y,
                'yaw': yaw,
            }
        )
    return out


class CoverageNode(Node):
    def __init__(self) -> None:
        super().__init__('phase2b1_handoff_coverage')
        self._amcl: Optional[PoseWithCovarianceStamped] = None
        self._loc = 1
        self._tasks: List[TaskResult] = []
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self._on_amcl, _LATCH)
        self.create_subscription(Int8, '/xw/localization_status', self._on_loc, _LATCH)
        self.create_subscription(TaskResult, '/xw/task/result', self._on_task, 10)
        self._goal = self.create_publisher(PoseStamped, '/xw/goal_pose', 10)
        self._cancel = self.create_publisher(Bool, '/xw/nav/cancel', 10)
        self._initp = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self._recovery = self.create_publisher(Bool, '/xw/localization/recovery_enable', _LATCH)
        self._nomotion = self.create_client(Empty, '/request_nomotion_update')
        self._reinit = self.create_client(Empty, '/reinitialize_global_localization')
        self._reloc = self.create_client(Relocalize, '/xw/relocalize')
        self._recovery.publish(Bool(data=False))

    def _on_amcl(self, m: PoseWithCovarianceStamped) -> None:
        self._amcl = m

    def _on_loc(self, m: Int8) -> None:
        self._loc = int(m.data)

    def _on_task(self, m: TaskResult) -> None:
        self._tasks.append(m)

    def sleep_spin(self, sec: float, hz: float = 4.0) -> None:
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
        self.sleep_spin(0.25, hz=5.0)
        if self._amcl is None:
            return None
        p = self._amcl.pose.pose
        return (float(p.position.x), float(p.position.y), yaw_from_quat(p.orientation))

    def cov_xy(self) -> Optional[float]:
        if self._amcl is None:
            return None
        c = self._amcl.pose.covariance
        return max(float(c[0]), float(c[7]))

    def seed(self, x: float, y: float, yaw: float, cov_xy: float = 0.20, cov_yaw: float = 0.10) -> None:
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.pose.position.x = float(x)
        msg.pose.pose.position.y = float(y)
        msg.pose.pose.orientation = yaw_to_quat(yaw)
        msg.pose.covariance[0] = cov_xy
        msg.pose.covariance[7] = cov_xy
        msg.pose.covariance[35] = cov_yaw
        for _ in range(2):
            self._initp.publish(msg)
            self.sleep_spin(0.15, hz=5.0)
        if self._nomotion.wait_for_service(timeout_sec=1.0):
            for _ in range(3):
                self._nomotion.call_async(Empty.Request())
                self.sleep_spin(0.35, hz=4.0)

    def wait_operator(self, spec: Dict[str, Any], prompt: bool, pause_sec: float) -> None:
        msg = (
            f"\n=== PLACE robot at {spec['trial_id']} ({spec['region']}) "
            f"expect={spec['expect']} ===\n"
            f"target pose ≈ ({spec['x']:.2f}, {spec['y']:.2f}, yaw={spec['yaw']:.2f})\n"
            f"Use push/teleop/carry — NOT part of Handoff score.\n"
        )
        print(msg, flush=True)
        self.get_logger().info(msg.strip())
        if prompt and sys.stdin.isatty():
            try:
                input('Press Enter when robot is physically placed and safe... ')
            except EOFError:
                self.sleep_spin(max(pause_sec, 5.0), hz=1.0)
        else:
            self.sleep_spin(max(pause_sec, 1.0), hz=1.0)

    def establish_gt(self, spec: Dict[str, Any], setup_seed: bool) -> Optional[Tuple[float, float, float]]:
        """Setup-only: optional seed at place pose, then snapshot GT. Not handoff."""
        if setup_seed:
            self.seed(spec['x'], spec['y'], spec['yaw'])
        t0 = time.monotonic()
        while time.monotonic() - t0 < 12.0:
            self.sleep_spin(0.3, hz=4.0)
            p = self.pose()
            c = self.cov_xy()
            if p is None or self._loc != 0 or (c is not None and c >= 1.0):
                continue
            if math.hypot(p[0] - spec['x'], p[1] - spec['y']) > SETUP_XY_TOL:
                continue
            if yaw_err(p[2], spec['yaw']) > SETUP_YAW_TOL:
                continue
            return p
        return None

    def pose_near_target(self, spec: Dict[str, Any], xy_tol: float, yaw_tol: float) -> bool:
        p = self.pose()
        if p is None:
            return False
        return math.hypot(p[0] - spec['x'], p[1] - spec['y']) < xy_tol and yaw_err(p[2], spec['yaw']) < yaw_tol

    def setup_nav_to(self, spec: Dict[str, Any]) -> bool:
        """Optional SETUP placement via Nav2 — failures are SETUP_FAILED, not handoff.

        Critical: never treat a fresh /initialpose seed as physical arrival.
        """
        p0 = self.pose()
        already = (
            p0 is not None
            and math.hypot(p0[0] - spec['x'], p0[1] - spec['y']) < NAV_TOL
            and yaw_err(p0[2], spec['yaw']) < NAV_YAW_TOL
        )
        if already:
            self.sleep_spin(SETTLE, hz=2.0)
            return True

        # Seed only to help Nav2 plan from a sane hypothesis — then MUST navigate.
        self.seed(spec['x'], spec['y'], spec['yaw'], cov_xy=0.35, cov_yaw=0.15)
        self.sleep_spin(0.8, hz=4.0)
        self._tasks.clear()
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.position.x = float(spec['x'])
        msg.pose.position.y = float(spec['y'])
        msg.pose.orientation = yaw_to_quat(spec['yaw'])
        self._goal.publish(msg)
        self.get_logger().info(f"SETUP nav → {spec['trial_id']}")
        t0 = time.monotonic()
        saw_task = False
        while time.monotonic() - t0 < NAV_TIMEOUT and rclpy.ok():
            self.sleep_spin(0.4, hz=3.0)
            for r in list(self._tasks):
                if r.capability == 'nav':
                    saw_task = True
                    if r.code != 0:
                        return False
                    # Nav reported success — still require live pose near target.
                    self.sleep_spin(SETTLE, hz=2.0)
                    return self.pose_near_target(spec, 1.0, 0.55)
            # Pose-near alone is insufficient right after seed; require elapsed travel
            # or a nav task. Allow early success only after robot had time to move.
            elapsed = time.monotonic() - t0
            if elapsed >= 8.0 and self.pose_near_target(spec, NAV_TOL, NAV_YAW_TOL):
                # Prefer evidence of motion from original pre-seed pose when available.
                if p0 is None or math.hypot(p0[0] - spec['x'], p0[1] - spec['y']) < 1.2:
                    self.sleep_spin(SETTLE, hz=2.0)
                    return True
                # Came from far away: require longer settle / task
                if elapsed >= 20.0:
                    self.sleep_spin(SETTLE, hz=2.0)
                    return True
        if saw_task:
            return self.pose_near_target(spec, 1.0, 0.55)
        return False

    def induce_unknown(self) -> Dict[str, Any]:
        if not self._reinit.wait_for_service(timeout_sec=2.0):
            return {'ok': False, 'error': 'reinit_unavailable'}
        self._reinit.call_async(Empty.Request())
        t0 = time.monotonic()
        while time.monotonic() - t0 < 6.0:
            self.sleep_spin(0.3, hz=3.0)
            c = self.cov_xy()
            if self._loc != 0 or (c is not None and c > 2.0):
                return {'ok': True, 'loc': self._loc, 'cov_xy': c, 'dt': time.monotonic() - t0}
        return {'ok': True, 'loc': self._loc, 'cov_xy': self.cov_xy(), 'soft': True}

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

    def short_nav(self, gt: Tuple[float, float, float], label: str) -> bool:
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
        pause_sec: float,
        setup_seed: bool,
        setup_nav: bool = False,
    ) -> Dict[str, Any]:
        out: Dict[str, Any] = {
            'trial_id': spec['trial_id'],
            'keyframe_id': spec['keyframe_id'],
            'region': spec['region'],
            'expect': spec['expect'],
            'class': spec['class'],
            'target': {'x': spec['x'], 'y': spec['y'], 'yaw': spec['yaw']},
        }
        if setup_nav:
            print(
                f"SETUP placement nav → {spec['trial_id']} ({spec['region']}) "
                f"(not scored as handoff)",
                flush=True,
            )
            if not self.setup_nav_to(spec):
                out['layer'] = 'SETUP_FAILED'
                out['setup_error'] = 'setup_nav_failed'
                return out
            # After Nav2 placement: snapshot live AMCL — do NOT re-seed (avoids fake yaw GT).
            gt = self.establish_gt(spec, setup_seed=False)
        else:
            self.wait_operator(spec, prompt, pause_sec)
            gt = self.establish_gt(spec, setup_seed=setup_seed)
        if gt is None or math.hypot(gt[0] - spec['x'], gt[1] - spec['y']) > SETUP_XY_TOL:
            out['layer'] = 'SETUP_FAILED'
            out['setup_error'] = 'gt_unavailable_or_far'
            out['gt_seen'] = None if gt is None else list(gt)
            return out
        if yaw_err(gt[2], spec['yaw']) > SETUP_YAW_TOL:
            out['layer'] = 'SETUP_FAILED'
            out['setup_error'] = 'gt_yaw_mismatch'
            out['gt_seen'] = list(gt)
            return out
        out['gt'] = {'x': gt[0], 'y': gt[1], 'yaw': gt[2]}
        out['gt_cov_xy'] = self.cov_xy()

        out['unknown_induction'] = self.induce_unknown()
        self.sleep_spin(0.4, hz=3.0)
        reloc = self.relocalize()
        diag = reloc.get('diagnostics') or {}
        timings = reloc.get('stage_timings') or {}
        out['reloc'] = {
            'result_code': reloc.get('result_code'),
            'laser_score': reloc.get('laser_score'),
            'wall_sec': reloc.get('wall_sec'),
            'time_to_candidate_sec': reloc.get('time_to_candidate_sec'),
            'amcl_convergence_sec': reloc.get('amcl_convergence_sec'),
            'candidate': reloc.get('candidate'),
            'decision': diag.get('decision'),
            'sensor_gate': diag.get('sensor_gate') or {
                k: timings.get(k)
                for k in (
                    'rgb_request_to_first_rgb_ms',
                    'scan_wait_ms',
                    'sensor_ready_ms',
                    'sensor_ready',
                )
                if k in timings
            },
            'amcl_handoff': diag.get('amcl_handoff'),
            'candidate_to_amcl_correction': diag.get('candidate_to_amcl_correction'),
        }
        code = reloc.get('result_code')
        cand = reloc.get('candidate')
        decision = diag.get('decision')

        if not reloc.get('ok'):
            out['layer'] = 'SETUP_FAILED'
            out['setup_error'] = reloc.get('error')
            self.seed(gt[0], gt[1], gt[2])
            return out

        if code == RC_NO_DATA or (diag.get('error') == 'sensor_not_ready'):
            out['layer'] = 'NO_DATA'
            out['safe_unknown'] = False
            self.seed(gt[0], gt[1], gt[2])
            return out

        if code in (RC_UNKNOWN, RC_REJECTED) or decision in ('UNKNOWN', 'REJECTED'):
            out['layer'] = 'RELOC_UNKNOWN'
            out['safe_unknown'] = True
            out['false_handoff'] = False
            self.seed(gt[0], gt[1], gt[2])
            return out

        if code not in (RC_READY, RC_AMCL_TIMEOUT) and decision != 'ACCEPT':
            out['layer'] = f'code_{code}'
            self.seed(gt[0], gt[1], gt[2])
            return out

        out['layer'] = 'RELOC_ACCEPT'
        out['handoff_attempted'] = True
        if cand is not None:
            out['candidate'] = {'x': cand[0], 'y': cand[1], 'yaw': cand[2]}
            out['candidate_error_xy'] = math.hypot(cand[0] - gt[0], cand[1] - gt[1])
            out['candidate_error_yaw'] = yaw_err(cand[2], gt[2])
            out['laser_score'] = reloc.get('laser_score')
            if out['candidate_error_xy'] > FA_XY_M or out['candidate_error_yaw'] > FA_YAW_RAD:
                out['layer'] = 'AMCL_FALSE_CONVERGENCE'
                out['false_handoff'] = True
                out['false_accept'] = True
                self.seed(gt[0], gt[1], gt[2])
                return out

        if code == RC_AMCL_TIMEOUT:
            out['layer'] = 'AMCL_TIMEOUT'
            out['amcl_ready'] = False
            out['false_handoff'] = False
            self.seed(gt[0], gt[1], gt[2])
            return out

        if code != RC_READY:
            out['layer'] = f'code_{code}'
            self.seed(gt[0], gt[1], gt[2])
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
                out['layer'] = 'AMCL_FALSE_CONVERGENCE'
                out['false_handoff'] = True
                return out
            out['quality_ok'] = out['final_error_xy'] <= OK_XY_M and out['final_error_yaw'] <= OK_YAW_RAD
        out['amcl_convergence_sec'] = reloc.get('amcl_convergence_sec')
        out['false_handoff'] = False

        nav_ok = self.short_nav(gt, spec['trial_id'])
        out['nav_success'] = bool(nav_ok)
        if not nav_ok:
            out['layer'] = 'NAV_AFTER_READY_FAILED'
            return out
        out['layer'] = 'SUCCESS'
        # Soft return near GT (setup convenience; not scored).
        back = PoseStamped()
        back.header.stamp = self.get_clock().now().to_msg()
        back.header.frame_id = 'map'
        back.pose.position.x = gt[0]
        back.pose.position.y = gt[1]
        back.pose.orientation = yaw_to_quat(gt[2])
        self._goal.publish(back)
        self.sleep_spin(3.0, hz=2.0)
        return out


def summarize(trials: List[Dict[str, Any]]) -> Dict[str, Any]:
    layers: Dict[str, int] = {}
    for t in trials:
        layers[t.get('layer') or 'NONE'] = layers.get(t.get('layer') or 'NONE', 0) + 1
    easy = [t for t in trials if t.get('class') == 'easy_correct_accept']
    hard = [t for t in trials if t.get('class') == 'hard_negative']
    reloc_accept = [
        t
        for t in trials
        if t.get('layer')
        in ('RELOC_ACCEPT', 'AMCL_TIMEOUT', 'AMCL_FALSE_CONVERGENCE', 'NAV_AFTER_READY_FAILED', 'SUCCESS')
    ]
    ready_success = [
        t
        for t in reloc_accept
        if t.get('amcl_ready') and not t.get('false_handoff') and t.get('layer') != 'AMCL_FALSE_CONVERGENCE'
    ]
    nav = [t for t in ready_success if 'nav_success' in t]
    nav_ok = [t for t in nav if t.get('nav_success')]
    false_h = [t for t in trials if t.get('false_handoff')]
    safe_unk = [t for t in trials if t.get('layer') == 'RELOC_UNKNOWN']
    # Easy Correct ACCEPT: easy trial produced ACCEPT path without false handoff
    # (SUCCESS / NAV_AFTER_READY_FAILED / AMCL_TIMEOUT all count as Correct ACCEPT issued)
    easy_correct_ids = {
        t['trial_id']
        for t in easy
        if t.get('layer') in ('SUCCESS', 'NAV_AFTER_READY_FAILED', 'AMCL_TIMEOUT')
        and not t.get('false_handoff')
    }
    walls = [float(t['reloc']['wall_sec']) for t in reloc_accept if t.get('reloc', {}).get('wall_sec') is not None]
    cand_t = [
        float(t['reloc']['time_to_candidate_sec'])
        for t in reloc_accept
        if t.get('reloc', {}).get('time_to_candidate_sec') is not None
    ]
    conv = [float(t['amcl_convergence_sec']) for t in ready_success if t.get('amcl_convergence_sec') is not None]
    return {
        'n': len(trials),
        'layers': layers,
        'easy_n': len(easy),
        'hard_n': len(hard),
        'easy_correct_accept': len(easy_correct_ids),
        'safe_unknown': len(safe_unk),
        'no_data': layers.get('NO_DATA', 0),
        'setup_failed': layers.get('SETUP_FAILED', 0),
        'reloc_accept_n': len(reloc_accept),
        'amcl_ready_n': len(ready_success),
        'amcl_ready_rate_on_accept': (len(ready_success) / len(reloc_accept)) if reloc_accept else None,
        'false_handoff': len(false_h),
        'nav_success_rate_after_ready': (len(nav_ok) / len(nav)) if nav else None,
        'success': layers.get('SUCCESS', 0),
        'reloc_wall_p50': pct(walls, 50),
        'reloc_wall_p95': pct(walls, 95),
        'visual_laser_p50': pct(cand_t, 50),
        'visual_laser_p95': pct(cand_t, 95),
        'amcl_conv_p50': pct(conv, 50),
        'amcl_conv_p95': pct(conv, 95),
    }


def gate_debug(stats: Dict[str, Any], trials: List[Dict[str, Any]]) -> Dict[str, Any]:
    fa = int(stats.get('false_handoff') or 0)
    easy_ca = int(stats.get('easy_correct_accept') or 0)
    reloc_n = int(stats.get('reloc_accept_n') or 0)
    amcl_rate = stats.get('amcl_ready_rate_on_accept')
    nav_rate = stats.get('nav_success_rate_after_ready')
    ok = (
        fa == 0
        and easy_ca >= 8
        and reloc_n >= 1
        and amcl_rate is not None
        and amcl_rate >= 0.90
        and nav_rate is not None
        and nav_rate >= 0.90
    )
    return {
        'pass': ok,
        'false_handoff': fa,
        'easy_correct_accept': easy_ca,
        'amcl_ready_rate_on_accept': amcl_rate,
        'nav_success_rate_after_ready': nav_rate,
        'require': 'FA=0, easy Correct ACCEPT≥8, AMCL≥90% on ACCEPT, Nav≥90% after READY',
    }


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument('--mode', choices=('debug', 'formal'), default='debug')
    ap.add_argument('--out', default='')
    ap.add_argument('--prompt', action='store_true', default=True)
    ap.add_argument('--no-prompt', action='store_true')
    ap.add_argument('--pause-sec', type=float, default=20.0)
    ap.add_argument('--setup-seed', action='store_true', default=True)
    ap.add_argument('--no-setup-seed', action='store_true')
    ap.add_argument(
        '--setup-nav',
        action='store_true',
        help='Use Nav2 only for SETUP placement (failures=SETUP_FAILED, not handoff)',
    )
    args = ap.parse_args()
    prompt = False if args.no_prompt else True
    setup_seed = False if args.no_setup_seed else True
    if args.setup_nav:
        prompt = False

    BENCH.mkdir(parents=True, exist_ok=True)
    out_path = Path(args.out) if args.out else BENCH / f'coverage_{args.mode}.json'
    trials_spec = build_debug_trials() if args.mode == 'debug' else build_formal_trials()

    rclpy.init()
    node = CoverageNode()
    node.sleep_spin(1.0, hz=3.0)
    results: List[Dict[str, Any]] = []
    stop = None
    node.get_logger().info(
        f'Phase2B1 coverage {args.mode} n={len(trials_spec)} '
        f'prompt={prompt} setup_nav={args.setup_nav}'
    )
    try:
        for i, spec in enumerate(trials_spec):
            node.get_logger().info(f'=== {i+1}/{len(trials_spec)} {spec["trial_id"]} ===')
            one = node.run_trial(
                spec,
                prompt=prompt,
                pause_sec=args.pause_sec,
                setup_seed=setup_seed,
                setup_nav=bool(args.setup_nav),
            )
            results.append(one)
            print(
                json.dumps(
                    {
                        'trial': one.get('trial_id'),
                        'layer': one.get('layer'),
                        'expect': one.get('expect'),
                        'fa': one.get('false_handoff'),
                        'xy': None if one.get('final_error_xy') is None else round(one['final_error_xy'], 3),
                        'nav': one.get('nav_success'),
                        'sensor_ms': (
                            (one.get('reloc') or {}).get('sensor_gate') or {}
                        ).get('sensor_ready_ms')
                        if isinstance((one.get('reloc') or {}).get('sensor_gate'), dict)
                        else None,
                    }
                ),
                flush=True,
            )
            if one.get('false_handoff'):
                stop = 'false_handoff'
                # Continue remaining trials for coverage diagnostics (gate still FA=0).
            node.sleep_spin(COOL, hz=1.0)
    finally:
        stats = summarize(results)
        payload = {
            'mode': args.mode,
            'validator': 'phase2b1_coverage_v1',
            'pre_nav_isolated': True,
            'allow_amcl_handoff': True,
            'production_unchanged': True,
            'stop_reason': stop,
            'stats': stats,
            'debug_gate': gate_debug(stats, results) if args.mode == 'debug' else None,
            'trials': results,
        }
        out_path.write_text(json.dumps(payload, indent=2), encoding='utf-8')
        print(json.dumps({'out': str(out_path), 'stats': stats, 'gate': payload.get('debug_gate')}, indent=2))
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
