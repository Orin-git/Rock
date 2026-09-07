#!/usr/bin/env python3
"""Phase2B AMCL Handoff — LIGHT validator.

Design goals (anti observer-effect):
  - Reloc idle must not decode camera (node-side gate).
  - Validator: no TF listener, no odom, no topic hz, no recorders.
  - Sparse spin (≤5 Hz waits), long cool-downs between reloc.
  - One NavigateToPose per region; ≥2 handoffs at same physical pose.
  - Metrics primarily from Relocalize response diagnostics_json.

Protocol per region:
  nav once → settle → cool-down
  ×N: snapshot GT → reinit(unknown) → /xw/relocalize(apply) → optional short nav
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
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Int8
from std_srvs.srv import Empty

from xw_interfaces.msg import TaskResult
from xw_interfaces.srv import Relocalize

BENCH = Path('/ros2_ws/bench/phase2b_amcl_handoff_2026-09-07')

RC_READY, RC_UNKNOWN, RC_REJECTED, RC_AMCL_TIMEOUT = 0, 1, 2, 3
FA_XY_M, FA_YAW_RAD = 1.0, 0.52
OK_XY_M, OK_YAW_RAD = 0.50, 0.35
NAV_TOL, NAV_TIMEOUT = 0.60, 100.0
NAV_OFFSET = 0.80
COOL_AFTER_RELOC = 8.0
COOL_AFTER_REGION = 12.0
SETTLE = 2.0

_LATCH = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)

REGIONS = [
    {'id': 'doorway_wp9', 'cls': 'doorway', 'x': 0.015, 'y': -0.580, 'yaw': 3.253},
    {'id': 'corridor_wp2', 'cls': 'corridor', 'x': -4.078, 'y': -0.939, 'yaw': 3.302},
    {'id': 'similar_corridor_wp3', 'cls': 'similar', 'x': -9.209, 'y': 1.191, 'yaw': 3.386},
    {'id': 'room_wp5', 'cls': 'room', 'x': -7.872, 'y': 3.250, 'yaw': 0.812},
    {'id': 'open_wp6', 'cls': 'open', 'x': -0.754, 'y': 9.388, 'yaw': 5.480},
]


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


class LiteHandoff(Node):
    def __init__(self) -> None:
        super().__init__('phase2b_handoff_lite')
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
        """Low-rate wait — never busy-spin."""
        dt = 1.0 / max(hz, 1.0)
        t0 = time.monotonic()
        while time.monotonic() - t0 < sec and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(dt)

    def wait_future(self, fut, timeout: float) -> bool:
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout and rclpy.ok() and not fut.done():
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(0.15)
        return fut.done()

    def pose(self) -> Optional[Tuple[float, float, float]]:
        self.sleep_spin(0.3, hz=5.0)
        if self._amcl is None:
            return None
        p = self._amcl.pose.pose
        return (float(p.position.x), float(p.position.y), yaw_from_quat(p.orientation))

    def cov_xy(self) -> Optional[float]:
        if self._amcl is None:
            return None
        c = self._amcl.pose.covariance
        return max(float(c[0]), float(c[7]))

    def seed(self, x: float, y: float, yaw: float) -> None:
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.pose.position.x = float(x)
        msg.pose.pose.position.y = float(y)
        msg.pose.pose.orientation = yaw_to_quat(yaw)
        msg.pose.covariance[0] = 0.25
        msg.pose.covariance[7] = 0.25
        msg.pose.covariance[35] = 0.12
        self._initp.publish(msg)
        if self._nomotion.wait_for_service(timeout_sec=1.0):
            self._nomotion.call_async(Empty.Request())
        self.sleep_spin(2.0, hz=3.0)

    def goto(self, x: float, y: float, yaw: float, label: str) -> bool:
        p = self.pose()
        if p is not None and math.hypot(p[0] - x, p[1] - y) < NAV_TOL:
            self.get_logger().info(f'{label}: near, skip')
            return True
        self._cancel.publish(Bool(data=True))
        self.sleep_spin(0.4, hz=5.0)
        self._tasks.clear()
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.orientation = yaw_to_quat(yaw)
        self._goal.publish(msg)
        self.get_logger().info(f'nav {label} → ({x:.2f},{y:.2f})')
        t0 = time.monotonic()
        while time.monotonic() - t0 < NAV_TIMEOUT and rclpy.ok():
            self.sleep_spin(0.4, hz=3.0)
            p = self.pose()
            if p is not None and math.hypot(p[0] - x, p[1] - y) < NAV_TOL:
                self.sleep_spin(SETTLE, hz=2.0)
                return True
            for r in self._tasks:
                if r.capability == 'nav':
                    if r.code == 0:
                        p2 = self.pose()
                        return p2 is not None and math.hypot(p2[0] - x, p2[1] - y) < 1.3
                    self.get_logger().warn(f'{label}: nav {r.message}')
                    return False
        self._cancel.publish(Bool(data=True))
        return False

    def induce_unknown(self) -> Dict[str, Any]:
        if not self._reinit.wait_for_service(timeout_sec=2.0):
            return {'ok': False}
        self._reinit.call_async(Empty.Request())
        t0 = time.monotonic()
        while time.monotonic() - t0 < 5.0:
            self.sleep_spin(0.35, hz=3.0)
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
        # Service does the heavy work; we only poll sparsely.
        ok = self.wait_future(fut, 75.0)
        wall = time.monotonic() - t0
        if not ok or fut.result() is None:
            return {'ok': False, 'error': 'timeout', 'wall_sec': wall}
        res = fut.result()
        try:
            diag = json.loads(res.diagnostics_json or '{}')
        except json.JSONDecodeError:
            diag = {}
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
        }

    def one_handoff(self, trial_id: str, region: Dict[str, Any], gt: Tuple[float, float, float]) -> Dict[str, Any]:
        out: Dict[str, Any] = {
            'trial_id': trial_id,
            'region': region['id'],
            'cls': region['cls'],
            'gt': {'x': gt[0], 'y': gt[1], 'yaw': gt[2]},
        }
        out['unknown'] = self.induce_unknown()
        self.sleep_spin(0.5, hz=3.0)
        reloc = self.relocalize()
        out['reloc'] = {
            k: reloc.get(k)
            for k in (
                'ok',
                'result_code',
                'success',
                'laser_score',
                'time_to_candidate_sec',
                'amcl_convergence_sec',
                'wall_sec',
                'candidate',
            )
        }
        # Keep handoff diag only (drop huge topk to stay light on disk/CPU JSON).
        diag = reloc.get('diagnostics') or {}
        out['reloc']['amcl_handoff'] = diag.get('amcl_handoff')
        out['reloc']['candidate_to_amcl_correction'] = diag.get('candidate_to_amcl_correction')
        out['reloc']['decision'] = diag.get('decision')
        code = reloc.get('result_code')
        cand = reloc.get('candidate')

        if code in (RC_UNKNOWN, RC_REJECTED):
            out['reloc_decision'] = 'UNKNOWN' if code == RC_UNKNOWN else 'REJECTED'
            out['handoff_attempted'] = False
            out['false_handoff'] = False
            self.seed(gt[0], gt[1], gt[2])
            return out

        out['reloc_decision'] = 'ACCEPT'
        out['handoff_attempted'] = True
        if cand is not None:
            out['candidate_error_xy'] = math.hypot(cand[0] - gt[0], cand[1] - gt[1])
            out['candidate_error_yaw'] = yaw_err(cand[2], gt[2])

        if code == RC_AMCL_TIMEOUT:
            out['amcl_ready'] = False
            out['amcl_timeout'] = True
            out['false_handoff'] = False
            self.seed(gt[0], gt[1], gt[2])
            return out

        if code != RC_READY:
            out['amcl_ready'] = False
            out['false_handoff'] = False
            self.seed(gt[0], gt[1], gt[2])
            return out

        out['amcl_ready'] = True
        out['amcl_timeout'] = False
        ah = diag.get('amcl_handoff') or {}
        amcl_pose = ah.get('amcl_pose')
        if amcl_pose is None:
            amcl_pose = list(self.pose() or [])
        if amcl_pose and len(amcl_pose) >= 3:
            out['amcl_after'] = {'x': amcl_pose[0], 'y': amcl_pose[1], 'yaw': amcl_pose[2]}
            out['amcl_cov_xy'] = ah.get('cov_xy')
            out['amcl_cov_yaw'] = ah.get('cov_yaw')
            out['final_error_xy'] = math.hypot(amcl_pose[0] - gt[0], amcl_pose[1] - gt[1])
            out['final_error_yaw'] = yaw_err(amcl_pose[2], gt[2])
            if cand is not None:
                out['candidate_to_amcl_dxy'] = math.hypot(amcl_pose[0] - cand[0], amcl_pose[1] - cand[1])
                out['candidate_to_amcl_dyaw'] = yaw_err(amcl_pose[2], cand[2])
            wrong = out['final_error_xy'] > FA_XY_M or out['final_error_yaw'] > FA_YAW_RAD
            out['false_handoff'] = bool(wrong)
            out['wrong_convergence'] = bool(wrong)
            out['quality_ok'] = out['final_error_xy'] <= OK_XY_M and out['final_error_yaw'] <= OK_YAW_RAD
        else:
            out['false_handoff'] = False

        if out.get('false_handoff'):
            return out

        # Short safe nav after READY.
        gx = gt[0] + NAV_OFFSET * math.cos(gt[2])
        gy = gt[1] + NAV_OFFSET * math.sin(gt[2])
        nav_ok = self.goto(gx, gy, gt[2], f'{trial_id}_nav')
        out['nav_success'] = bool(nav_ok)
        out['localization_lost_during_nav'] = self._loc == 3
        # Return near GT for next handoff at same region (cheap if already close).
        self.goto(gt[0], gt[1], gt[2], f'{trial_id}_back')
        return out


def summarize(trials: List[Dict[str, Any]]) -> Dict[str, Any]:
    active = [t for t in trials if not t.get('aborted')]
    acc = [t for t in active if t.get('reloc_decision') == 'ACCEPT']
    unk = [t for t in active if t.get('reloc_decision') == 'UNKNOWN']
    ready = [t for t in acc if t.get('amcl_ready')]
    to = [t for t in acc if t.get('amcl_timeout')]
    fa = [t for t in acc if t.get('false_handoff')]
    nav = [t for t in ready if 'nav_success' in t]
    nav_ok = [t for t in nav if t.get('nav_success')]
    walls = [float(t['reloc']['wall_sec']) for t in acc if t.get('reloc', {}).get('wall_sec') is not None]
    conv = [float(t['reloc']['amcl_convergence_sec']) for t in ready if t.get('reloc', {}).get('amcl_convergence_sec') is not None]
    return {
        'n': len(trials),
        'aborted': sum(1 for t in trials if t.get('aborted')),
        'ACCEPT': len(acc),
        'UNKNOWN': len(unk),
        'amcl_ready': len(ready),
        'amcl_timeout': len(to),
        'false_handoff': len(fa),
        'amcl_success_rate': (len(ready) / len(acc)) if acc else None,
        'nav_success_rate': (len(nav_ok) / len(nav)) if nav else None,
        'reloc_wall_p50': pct(walls, 50),
        'reloc_wall_p95': pct(walls, 95),
        'amcl_conv_p50': pct(conv, 50),
        'amcl_conv_p95': pct(conv, 95),
    }


def gate_debug(s: Dict[str, Any]) -> Dict[str, Any]:
    fa = int(s.get('false_handoff') or 0)
    conv = s.get('amcl_success_rate')
    nav = s.get('nav_success_rate')
    # Need enough ACCEPT attempts; aborted-only runs cannot pass.
    ok = (
        fa == 0
        and int(s.get('ACCEPT') or 0) >= 8
        and conv is not None
        and conv >= 0.90
        and nav is not None
        and nav >= 0.90
    )
    return {'pass': ok, 'false_handoff': fa, 'amcl_success_rate': conv, 'nav_success_rate': nav}


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument('--mode', choices=('debug', 'formal'), default='debug')
    ap.add_argument('--out', default='')
    args = ap.parse_args()
    BENCH.mkdir(parents=True, exist_ok=True)
    out_path = Path(args.out) if args.out else BENCH / f'handoff_lite_{args.mode}.json'
    per_region = 2 if args.mode == 'debug' else 6

    rclpy.init()
    node = LiteHandoff()
    # Prefetch latch.
    node.sleep_spin(1.0, hz=3.0)
    trials: List[Dict[str, Any]] = []
    stop = None
    node.get_logger().info(f'LITE handoff {args.mode} regions={len(REGIONS)} x{per_region}')
    try:
        for region in REGIONS:
            # Setup localize only if clearly lost — seed at region ONLY after failed previous,
            # but prefer current pose if already healthy near path.
            p = node.pose()
            c = node.cov_xy()
            if node._loc != 0 or p is None or (c is not None and c > 1.5):
                # Seed near last known / region approach: use current if any, else region.
                if p is not None:
                    node.seed(p[0], p[1], p[2])
                else:
                    node.seed(region['x'], region['y'], region['yaw'])
            if not node.goto(region['x'], region['y'], region['yaw'], region['id']):
                for i in range(per_region):
                    trials.append(
                        {
                            'trial_id': f"{region['id']}_{i}",
                            'region': region['id'],
                            'aborted': 'goto_failed',
                        }
                    )
                node.sleep_spin(COOL_AFTER_REGION, hz=1.0)
                continue
            node.sleep_spin(SETTLE, hz=2.0)
            gt = node.pose()
            if gt is None or math.hypot(gt[0] - region['x'], gt[1] - region['y']) > 1.5:
                for i in range(per_region):
                    trials.append(
                        {
                            'trial_id': f"{region['id']}_{i}",
                            'region': region['id'],
                            'aborted': 'bad_gt',
                            'gt_seen': None if gt is None else list(gt),
                        }
                    )
                continue

            for i in range(per_region):
                tid = f"{region['id']}_{i}"
                node.get_logger().info(f'=== {tid} ===')
                # Refresh GT each handoff (robot should still be at region).
                gt_i = node.pose() or gt
                one = node.one_handoff(tid, region, gt_i)
                trials.append(one)
                print(
                    json.dumps(
                        {
                            'trial': tid,
                            'dec': one.get('reloc_decision'),
                            'ready': one.get('amcl_ready'),
                            'fa': one.get('false_handoff'),
                            'nav': one.get('nav_success'),
                            'xy': None if one.get('final_error_xy') is None else round(one['final_error_xy'], 3),
                            'wall': None
                            if not one.get('reloc')
                            else round(float(one['reloc'].get('wall_sec') or 0), 1),
                        }
                    ),
                    flush=True,
                )
                if one.get('false_handoff'):
                    stop = 'false_handoff'
                    break
                node.sleep_spin(COOL_AFTER_RELOC, hz=1.0)
            if stop:
                break
            node.sleep_spin(COOL_AFTER_REGION, hz=1.0)
    finally:
        stats = summarize(trials)
        payload = {
            'mode': args.mode,
            'validator': 'lite_v1',
            'allow_amcl_handoff': True,
            'per_region': per_region,
            'cool_after_reloc_sec': COOL_AFTER_RELOC,
            'cool_after_region_sec': COOL_AFTER_REGION,
            'stop_reason': stop,
            'stats': stats,
            'debug_gate': gate_debug(stats) if args.mode == 'debug' else None,
            'trials': trials,
        }
        out_path.write_text(json.dumps(payload, indent=2), encoding='utf-8')
        print(json.dumps({'out': str(out_path), 'stats': stats, 'gate': payload.get('debug_gate')}, indent=2))
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
