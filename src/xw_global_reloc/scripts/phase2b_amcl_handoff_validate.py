#!/usr/bin/env python3
"""Phase2B AMCL Handoff live validation (Debug10 / Formal30).

Protocol per trial:
  1) Navigate under healthy AMCL → record GT
  2) Induce unknown via reinitialize_global_localization (test setup ONLY)
  3) /xw/relocalize apply_initial_pose=true (no human initialpose)
  4) On READY → short safe NavigateToPose
  5) On UNKNOWN → record, not counted as handoff error
  6) False Handoff → stop (no Formal)

Does NOT enable Supervisor BOOT / production bringup.
Light metrics only — no topic hz / recorders.
"""

from __future__ import annotations

import argparse
import json
import math
import time
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Int8
from std_srvs.srv import Empty
from tf2_ros import Buffer, TransformException, TransformListener

from xw_interfaces.msg import TaskResult
from xw_interfaces.srv import GetState, Relocalize

BENCH = Path('/ros2_ws/bench/phase2b_amcl_handoff_2026-09-07')

# Result codes from Relocalize.srv
RC_READY = 0
RC_UNKNOWN = 1
RC_REJECTED = 2
RC_AMCL_TIMEOUT = 3
RC_NO_DATA = 4
RC_MAP_HASH = 5
RC_DRY_RUN = 6

# False handoff / wrong convergence vs pre-corruption GT
FA_XY_M = 1.0
FA_YAW_RAD = 0.52
# Engineering quality after READY
OK_XY_M = 0.50
OK_YAW_RAD = 0.35

NAV_XY_TOL = 0.55
NAV_TIMEOUT = 120.0
SETTLE_SEC = 1.0
NAV_GOAL_OFFSET_M = 0.90
MIN_BATTERY = 18.0

_LATCH = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)
_AMCL_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)

# ≥5 regions × 2 = Debug10. Formal expands A/B offsets.
REGIONS = [
    {'id': 'doorway_wp9', 'cls': 'doorway', 'x': 0.015, 'y': -0.580, 'yaw': 3.253},
    {'id': 'corridor_wp2', 'cls': 'corridor', 'x': -4.078, 'y': -0.939, 'yaw': 3.302},
    {'id': 'similar_corridor_wp3', 'cls': 'similar', 'x': -9.209, 'y': 1.191, 'yaw': 3.386},
    {'id': 'room_wp5', 'cls': 'room', 'x': -7.872, 'y': 3.250, 'yaw': 0.812},
    {'id': 'open_wp6', 'cls': 'open', 'x': -0.754, 'y': 9.388, 'yaw': 5.480},
]


def yaw_norm(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


def yaw_err(a: float, b: float) -> float:
    return abs(yaw_norm(a - b))


def yaw_to_quat(yaw: float):
    from geometry_msgs.msg import Quaternion

    q = Quaternion()
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q


def yaw_from_quat(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def pct(xs: List[float], p: float) -> Optional[float]:
    if not xs:
        return None
    import numpy as np

    return float(np.percentile(np.asarray(xs, dtype=float), p))


def build_trials(mode: str) -> List[Dict[str, Any]]:
    trials = []
    if mode == 'debug':
        for r in REGIONS:
            for i, dyaw in enumerate((-0.15, 0.15)):
                trials.append(
                    {
                        'trial_id': f"{r['id']}_{'A' if i == 0 else 'B'}",
                        'region': r['id'],
                        'cls': r['cls'],
                        'x': r['x'],
                        'y': r['y'],
                        'yaw': yaw_norm(r['yaw'] + dyaw),
                    }
                )
    else:
        # Formal ≥30: 5 regions × 3 yaws × 2 slight XY offsets
        for r in REGIONS:
            for j, (ox, oy) in enumerate(((0.0, 0.0), (0.25, 0.0), (0.0, 0.25))):
                for i, dyaw in enumerate((-0.20, 0.0, 0.20)):
                    trials.append(
                        {
                            'trial_id': f"{r['id']}_f{j}{i}",
                            'region': r['id'],
                            'cls': r['cls'],
                            'x': r['x'] + ox,
                            'y': r['y'] + oy,
                            'yaw': yaw_norm(r['yaw'] + dyaw),
                        }
                    )
        trials = trials[:30]
    return trials


class HandoffValidator(Node):
    def __init__(self) -> None:
        super().__init__('phase2b_amcl_handoff_validate')
        self.amcl: Optional[PoseWithCovarianceStamped] = None
        self.amcl_mono: Optional[float] = None
        self.odom: Optional[Odometry] = None
        self.loc = 1
        self.task_results: List[TaskResult] = []
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self._on_amcl, _AMCL_QOS)
        self.create_subscription(Odometry, '/odom', self._on_odom, 10)
        self.create_subscription(Int8, '/xw/localization_status', self._on_loc, _LATCH)
        self.create_subscription(TaskResult, '/xw/task/result', self._on_task, 10)
        self.goal_pub = self.create_publisher(PoseStamped, '/xw/goal_pose', 10)
        self.nav_cancel = self.create_publisher(Bool, '/xw/nav/cancel', 10)
        self.initialpose = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self.recovery_en = self.create_publisher(Bool, '/xw/localization/recovery_enable', _LATCH)
        self.nomotion = self.create_client(Empty, '/request_nomotion_update')
        self.reinit = self.create_client(Empty, '/reinitialize_global_localization')
        self.reloc = self.create_client(Relocalize, '/xw/relocalize')
        self.get_state = self.create_client(GetState, '/xw/supervisor/get_state')
        self._tf = Buffer()
        self._tfl = TransformListener(self._tf, self)
        # Disarm health self-heal so it does not fight our unknown→handoff sequence.
        self.recovery_en.publish(Bool(data=False))

    def _on_amcl(self, msg: PoseWithCovarianceStamped) -> None:
        self.amcl = msg
        self.amcl_mono = time.monotonic()

    def _on_odom(self, msg: Odometry) -> None:
        self.odom = msg

    def _on_loc(self, msg: Int8) -> None:
        self.loc = int(msg.data)

    def _on_task(self, msg: TaskResult) -> None:
        self.task_results.append(msg)

    def spin_wait(self, sec: float) -> None:
        t0 = time.monotonic()
        while time.monotonic() - t0 < sec and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)

    def battery(self) -> Optional[float]:
        if not self.get_state.wait_for_service(timeout_sec=1.0):
            return None
        fut = self.get_state.call_async(GetState.Request())
        t0 = time.monotonic()
        while time.monotonic() - t0 < 3.0 and not fut.done():
            rclpy.spin_once(self, timeout_sec=0.05)
        if not fut.done() or fut.result() is None:
            return None
        try:
            return float(fut.result().state.power.battery_percent)
        except Exception:  # noqa: BLE001
            return None

    def pose_amcl(self) -> Optional[Tuple[float, float, float]]:
        if self.amcl is None:
            return None
        p = self.amcl.pose.pose
        return (float(p.position.x), float(p.position.y), yaw_from_quat(p.orientation))

    def pose_tf(self) -> Optional[Tuple[float, float, float]]:
        try:
            tf = self._tf.lookup_transform(
                'map', 'base_link', rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=0.2)
            )
        except TransformException:
            return None
        t = tf.transform.translation
        return (float(t.x), float(t.y), yaw_from_quat(tf.transform.rotation))

    def cov(self) -> Tuple[Optional[float], Optional[float]]:
        if self.amcl is None:
            return None, None
        c = self.amcl.pose.covariance
        return max(float(c[0]), float(c[7])), float(c[35])

    def speed(self) -> float:
        if self.odom is None:
            return 0.0
        v = self.odom.twist.twist.linear
        w = self.odom.twist.twist.angular
        return math.hypot(v.x, v.y) + abs(w.z)

    def seed_pose(self, x: float, y: float, yaw: float, cov_xy: float = 0.25, cov_yaw: float = 0.15) -> None:
        """Test-setup seed only (navigate under known pose). Not used as recovery path."""
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.pose.position.x = float(x)
        msg.pose.pose.position.y = float(y)
        msg.pose.pose.orientation = yaw_to_quat(yaw)
        msg.pose.covariance[0] = cov_xy
        msg.pose.covariance[7] = cov_xy
        msg.pose.covariance[35] = cov_yaw
        self.initialpose.publish(msg)
        if self.nomotion.wait_for_service(timeout_sec=1.0):
            self.nomotion.call_async(Empty.Request())
        self.spin_wait(1.5)

    def ensure_localized_for_setup(
        self, x: float, y: float, yaw: float, last_physical: Optional[Tuple[float, float, float]]
    ) -> bool:
        """Recover localization for SETUP navigation only.

        Never seeds the *destination* (that teleports AMCL without moving the robot).
        If lost, re-seed at last known physical pose, then real NavigateToPose.
        """
        self.spin_wait(0.5)
        p = self.pose_tf() or self.pose_amcl()
        cxy, _ = self.cov()
        healthy = self.loc == 0 and p is not None and (cxy is None or cxy < 1.0)
        if healthy:
            return True
        seed = last_physical or p
        if seed is None:
            # Last resort: seed near map origin of known open area (doorway) — still not target.
            seed = (0.0, -0.6, 3.14)
            self.get_logger().warn('no physical prior; weak doorway seed for setup only')
        self.get_logger().info(
            f'setup re-seed at physical prior ({seed[0]:.2f},{seed[1]:.2f}) — not destination'
        )
        self.seed_pose(seed[0], seed[1], seed[2], cov_xy=0.35, cov_yaw=0.20)
        t0 = time.monotonic()
        while time.monotonic() - t0 < 12.0:
            rclpy.spin_once(self, timeout_sec=0.05)
            cxy, _ = self.cov()
            if self.loc == 0 and self.pose_amcl() is not None and (cxy is None or cxy < 1.0):
                return True
        return self.pose_amcl() is not None

    def goto(self, x: float, y: float, yaw: float, label: str, require_motion: bool = False) -> bool:
        cur = self.pose_tf() or self.pose_amcl()
        if (
            not require_motion
            and cur is not None
            and math.hypot(cur[0] - x, cur[1] - y) < NAV_XY_TOL
        ):
            self.get_logger().info(f'{label}: already near')
            return True
        before = time.monotonic()
        self.task_results.clear()
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.orientation = yaw_to_quat(yaw)
        self.goal_pub.publish(msg)
        self.get_logger().info(f'nav → {label} ({x:.2f},{y:.2f})')
        t0 = time.monotonic()
        settled = 0.0
        while time.monotonic() - t0 < NAV_TIMEOUT and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)
            p = self.pose_tf() or self.pose_amcl()
            if p is None:
                continue
            d = math.hypot(p[0] - x, p[1] - y)
            if d < NAV_XY_TOL and self.speed() < 0.08:
                settled += 0.05
                if settled >= SETTLE_SEC:
                    return True
            else:
                settled = 0.0
            for r in list(self.task_results):
                stamp = r.stamp.sec + r.stamp.nanosec * 1e-9
                if r.capability == 'nav' and stamp > before - 1.0:
                    if r.code == 0:
                        p2 = self.pose_tf() or self.pose_amcl()
                        if p2 is not None and math.hypot(p2[0] - x, p2[1] - y) < 1.2:
                            return True
                    if r.code == 1:
                        self.get_logger().warn(f'{label}: nav fail {r.message}')
                        return False
        self.nav_cancel.publish(Bool(data=True))
        self.spin_wait(0.3)
        return False

    def induce_unknown(self) -> Dict[str, Any]:
        """Spread particles for cold-start simulation. Allowed ONLY before reloc candidate."""
        info: Dict[str, Any] = {'method': 'reinitialize_global_localization'}
        if not self.reinit.wait_for_service(timeout_sec=2.0):
            info['ok'] = False
            info['error'] = 'reinit_unavailable'
            return info
        before = self.amcl_mono
        self.reinit.call_async(Empty.Request())
        t0 = time.monotonic()
        while time.monotonic() - t0 < 8.0:
            rclpy.spin_once(self, timeout_sec=0.05)
            cxy, cyaw = self.cov()
            if self.loc != 0:
                info['ok'] = True
                info['loc'] = self.loc
                info['cov_xy'] = cxy
                info['wait_sec'] = time.monotonic() - t0
                return info
            if cxy is not None and cxy > 2.0:
                info['ok'] = True
                info['loc'] = self.loc
                info['cov_xy'] = cxy
                info['wait_sec'] = time.monotonic() - t0
                return info
        info['ok'] = True
        info['loc'] = self.loc
        info['cov_xy'], info['cov_yaw'] = self.cov()
        info['wait_sec'] = time.monotonic() - t0
        info['note'] = 'unknown_induction_soft'
        return info

    def call_relocalize(self) -> Dict[str, Any]:
        if not self.reloc.wait_for_service(timeout_sec=5.0):
            return {'ok': False, 'error': 'relocalize_service_missing'}
        req = Relocalize.Request()
        req.map_name = 'vp'
        req.force_visual = True
        req.max_candidates = 5
        req.apply_initial_pose = True
        req.allow_motion = False
        t0 = time.monotonic()
        fut = self.reloc.call_async(req)
        while time.monotonic() - t0 < 90.0 and not fut.done():
            rclpy.spin_once(self, timeout_sec=0.05)
        wall = time.monotonic() - t0
        if not fut.done() or fut.result() is None:
            return {'ok': False, 'error': 'relocalize_timeout', 'wall_sec': wall}
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
            'success': bool(res.success),
            'result_code': int(res.result_code),
            'laser_score': float(res.laser_score),
            'visual_score': float(res.visual_score),
            'time_to_candidate_sec': float(res.time_to_candidate_sec),
            'amcl_convergence_sec': float(res.amcl_convergence_sec),
            'wall_sec': wall,
            'candidate': cand,
            'diagnostics': diag,
            'stage_timings': timings,
        }

    def run_trial(
        self, spec: Dict[str, Any], last_physical: Optional[Tuple[float, float, float]]
    ) -> Tuple[Dict[str, Any], Optional[Tuple[float, float, float]]]:
        out: Dict[str, Any] = {
            'trial_id': spec['trial_id'],
            'region': spec['region'],
            'cls': spec['cls'],
            'target': {'x': spec['x'], 'y': spec['y'], 'yaw': spec['yaw']},
            'battery_start': self.battery(),
        }
        if out['battery_start'] is not None and out['battery_start'] < MIN_BATTERY:
            out['aborted'] = 'low_battery'
            return out, last_physical

        if not self.ensure_localized_for_setup(spec['x'], spec['y'], spec['yaw'], last_physical):
            out['aborted'] = 'setup_localize_failed'
            return out, last_physical
        # Always command nav to region (physical motion). Skip only if TF already near.
        if not self.goto(spec['x'], spec['y'], spec['yaw'], spec['trial_id']):
            out['aborted'] = 'goto_failed'
            return out, last_physical
        self.spin_wait(0.8)
        gt = self.pose_tf() or self.pose_amcl()
        if gt is None:
            out['aborted'] = 'no_gt'
            return out, last_physical
        # Sanity: GT must be near commanded target (catches teleport-seed bugs).
        if math.hypot(gt[0] - spec['x'], gt[1] - spec['y']) > 1.5:
            out['aborted'] = 'gt_far_from_target'
            out['gt_bad'] = {'x': gt[0], 'y': gt[1], 'yaw': gt[2]}
            return out, last_physical
        physical = gt
        out['gt'] = {'x': gt[0], 'y': gt[1], 'yaw': gt[2]}
        out['gt_cov_xy'], out['gt_cov_yaw'] = self.cov()
        out['gt_loc'] = self.loc

        out['unknown_induction'] = self.induce_unknown()
        self.spin_wait(0.3)

        reloc = self.call_relocalize()
        out['reloc'] = reloc
        code = reloc.get('result_code')
        if code == RC_UNKNOWN or code == RC_REJECTED:
            out['reloc_decision'] = 'UNKNOWN' if code == RC_UNKNOWN else 'REJECTED'
            out['handoff_attempted'] = False
            out['false_handoff'] = False
            # Restore setup localization for next trial.
            self.seed_pose(physical[0], physical[1], physical[2])
            return out, physical
        if code == RC_AMCL_TIMEOUT:
            out['reloc_decision'] = 'ACCEPT'
            out['amcl_ready'] = False
            out['amcl_timeout'] = True
            out['handoff_attempted'] = True
            out['false_handoff'] = False
            cand = reloc.get('candidate')
            if cand is not None:
                out['candidate_error_xy'] = math.hypot(cand[0] - gt[0], cand[1] - gt[1])
                out['candidate_error_yaw'] = yaw_err(cand[2], gt[2])
                out['laser_score'] = reloc.get('laser_score')
            self.seed_pose(physical[0], physical[1], physical[2])
            return out, physical
        if code != RC_READY:
            out['reloc_decision'] = f'code_{code}'
            out['handoff_attempted'] = bool(reloc.get('diagnostics', {}).get('apply_initial_pose'))
            out['false_handoff'] = False
            self.seed_pose(physical[0], physical[1], physical[2])
            return out, physical

        out['reloc_decision'] = 'ACCEPT'
        out['amcl_ready'] = True
        out['amcl_timeout'] = False
        out['handoff_attempted'] = True
        cand = reloc.get('candidate')
        amcl_after = self.pose_amcl() or self.pose_tf()
        cxy, cyaw = self.cov()
        out['amcl_after'] = None if amcl_after is None else {
            'x': amcl_after[0], 'y': amcl_after[1], 'yaw': amcl_after[2]
        }
        out['amcl_cov_xy'] = cxy
        out['amcl_cov_yaw'] = cyaw
        if cand is not None:
            out['candidate_error_xy'] = math.hypot(cand[0] - gt[0], cand[1] - gt[1])
            out['candidate_error_yaw'] = yaw_err(cand[2], gt[2])
            out['laser_score'] = reloc.get('laser_score')
        if amcl_after is not None:
            out['final_error_xy'] = math.hypot(amcl_after[0] - gt[0], amcl_after[1] - gt[1])
            out['final_error_yaw'] = yaw_err(amcl_after[2], gt[2])
            if cand is not None:
                out['candidate_to_amcl_dxy'] = math.hypot(
                    amcl_after[0] - cand[0], amcl_after[1] - cand[1]
                )
                out['candidate_to_amcl_dyaw'] = yaw_err(amcl_after[2], cand[2])
            wrong = out['final_error_xy'] > FA_XY_M or out['final_error_yaw'] > FA_YAW_RAD
            out['false_handoff'] = bool(wrong)
            out['wrong_convergence'] = bool(wrong)
            out['quality_ok'] = (
                out['final_error_xy'] <= OK_XY_M and out['final_error_yaw'] <= OK_YAW_RAD
            )
        else:
            out['false_handoff'] = False
            out['wrong_convergence'] = False

        if out.get('false_handoff'):
            out['nav_success'] = False
            out['nav_skipped'] = 'false_handoff_stop'
            return out, physical

        gx = gt[0] + NAV_GOAL_OFFSET_M * math.cos(gt[2])
        gy = gt[1] + NAV_GOAL_OFFSET_M * math.sin(gt[2])
        pre_nav = self.pose_amcl() or self.pose_tf()
        nav_ok = self.goto(gx, gy, gt[2], f"{spec['trial_id']}_nav")
        post_nav = self.pose_amcl() or self.pose_tf()
        out['nav_goal'] = {'x': gx, 'y': gy, 'yaw': gt[2]}
        out['nav_success'] = bool(nav_ok)
        out['nav_loc_end'] = self.loc
        if pre_nav and post_nav:
            out['nav_pose_jump'] = math.hypot(post_nav[0] - pre_nav[0], post_nav[1] - pre_nav[1])
        out['localization_lost_during_nav'] = self.loc == 3
        self.goto(gt[0], gt[1], gt[2], f"{spec['trial_id']}_return")
        out['battery_end'] = self.battery()
        # Update physical prior from post-handoff pose if quality OK.
        if amcl_after is not None and out.get('quality_ok'):
            physical = amcl_after
        return out, physical


def summarize(trials: List[Dict[str, Any]]) -> Dict[str, Any]:
    n = len(trials)
    aborted = [t for t in trials if t.get('aborted')]
    active = [t for t in trials if not t.get('aborted')]
    accepts = [t for t in active if t.get('reloc_decision') == 'ACCEPT']
    unknowns = [t for t in active if t.get('reloc_decision') == 'UNKNOWN']
    ready = [t for t in accepts if t.get('amcl_ready')]
    timeouts = [t for t in accepts if t.get('amcl_timeout')]
    false_h = [t for t in accepts if t.get('false_handoff')]
    wrong = [t for t in accepts if t.get('wrong_convergence')]
    nav = [t for t in ready if 'nav_success' in t]
    nav_ok = [t for t in nav if t.get('nav_success')]
    reloc_wall = [float(t['reloc']['wall_sec']) for t in accepts if t.get('reloc', {}).get('wall_sec') is not None]
    amcl_conv = [
        float(t['reloc']['amcl_convergence_sec'])
        for t in ready
        if t.get('reloc', {}).get('amcl_convergence_sec') is not None
    ]
    corr_xy = [float(t['candidate_to_amcl_dxy']) for t in ready if t.get('candidate_to_amcl_dxy') is not None]
    final_xy = [float(t['final_error_xy']) for t in ready if t.get('final_error_xy') is not None]
    return {
        'n_trials': n,
        'aborted': len(aborted),
        'active': len(active),
        'reloc_ACCEPT': len(accepts),
        'reloc_UNKNOWN': len(unknowns),
        'amcl_ready': len(ready),
        'amcl_timeout': len(timeouts),
        'false_handoff': len(false_h),
        'wrong_convergence': len(wrong),
        'amcl_convergence_success_rate': (len(ready) / len(accepts)) if accepts else None,
        'nav_trials': len(nav),
        'nav_success': len(nav_ok),
        'nav_success_rate': (len(nav_ok) / len(nav)) if nav else None,
        'reloc_wall_p50': pct(reloc_wall, 50),
        'reloc_wall_p95': pct(reloc_wall, 95),
        'amcl_conv_p50': pct(amcl_conv, 50),
        'amcl_conv_p95': pct(amcl_conv, 95),
        'cand_to_amcl_dxy_p50': pct(corr_xy, 50),
        'final_error_xy_p50': pct(final_xy, 50),
        'final_error_xy_p95': pct(final_xy, 95),
    }


def gate_debug(stats: Dict[str, Any]) -> Dict[str, Any]:
    fa = int(stats.get('false_handoff') or 0)
    conv = stats.get('amcl_convergence_success_rate')
    nav = stats.get('nav_success_rate')
    accepts = int(stats.get('reloc_ACCEPT') or 0)
    ok = (
        fa == 0
        and accepts >= 1
        and conv is not None
        and conv >= 0.90
        and nav is not None
        and nav >= 0.90
    )
    return {
        'pass': ok,
        'false_handoff': fa,
        'amcl_convergence_success_rate': conv,
        'nav_success_rate': nav,
        'require': 'FA=0, AMCL≥90%, Nav≥90%',
    }


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument('--mode', choices=('debug', 'formal'), default='debug')
    ap.add_argument('--out', default='')
    ap.add_argument('--start', type=int, default=0)
    args = ap.parse_args()
    BENCH.mkdir(parents=True, exist_ok=True)
    out_path = Path(args.out) if args.out else BENCH / f'handoff_{args.mode}.json'

    rclpy.init()
    node = HandoffValidator()
    trials_spec = build_trials(args.mode)[args.start :]
    results: List[Dict[str, Any]] = []
    stopped = False
    stop_reason = ''
    last_physical: Optional[Tuple[float, float, float]] = None
    node.get_logger().info(f'Phase2B handoff {args.mode} n={len(trials_spec)}')
    try:
        for i, spec in enumerate(trials_spec):
            node.get_logger().info(f'=== trial {i+1}/{len(trials_spec)} {spec["trial_id"]} ===')
            one, last_physical = node.run_trial(spec, last_physical)
            results.append(one)
            print(
                json.dumps(
                    {
                        'trial': one.get('trial_id'),
                        'decision': one.get('reloc_decision'),
                        'ready': one.get('amcl_ready'),
                        'fa': one.get('false_handoff'),
                        'nav': one.get('nav_success'),
                        'final_xy': one.get('final_error_xy'),
                        'cand_xy': one.get('candidate_error_xy'),
                        'aborted': one.get('aborted'),
                    }
                ),
                flush=True,
            )
            if one.get('false_handoff'):
                stopped = True
                stop_reason = 'false_handoff'
                break
            if one.get('aborted') == 'low_battery':
                stopped = True
                stop_reason = 'low_battery'
                break
    finally:
        stats = summarize(results)
        payload = {
            'mode': args.mode,
            'allow_amcl_handoff': True,
            'apply_initial_pose': True,
            'production_robot_launch_unchanged': True,
            'stopped_on_false_handoff': stopped and stop_reason == 'false_handoff',
            'stop_reason': stop_reason or None,
            'stats': stats,
            'debug_gate': gate_debug(stats) if args.mode == 'debug' else None,
            'trials': results,
        }
        out_path.write_text(json.dumps(payload, indent=2), encoding='utf-8')
        print(json.dumps({'out': str(out_path), 'stats': stats, 'gate': payload.get('debug_gate')}, indent=2))
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
