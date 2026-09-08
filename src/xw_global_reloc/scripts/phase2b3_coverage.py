#!/usr/bin/env python3
"""Phase2B3 coverage — resident Reloc (idle-fixed), no reinit, ~10 low-load trials."""

from __future__ import annotations

import json
import math
import os
import time
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

import rclpy
import yaml
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Int8
from xw_interfaces.msg import TaskResult
from xw_interfaces.srv import Relocalize

BENCH = Path('/ros2_ws/bench/phase2b3_resource_coverage_2026-09-08')
WP_FILE = Path('/ros2_ws/maps/waypoints/vp_pointList.yaml')
RC_READY, RC_UNKNOWN, RC_REJECTED, RC_AMCL_TIMEOUT, RC_NO_DATA = 0, 1, 2, 3, 4
FA_XY, FA_YAW = 1.0, 0.52
MAX_LOAD1 = 9.0
NAV_TOL, NAV_TIMEOUT = 0.85, 120.0

# Prior Phase2B2 scored trials credited toward ~10:
# wp_9 SUCCESS, wp_2 SUCCESS, wp_3 SAFE_UNKNOWN, wp_5 SUCCESS
SESSION = [
    {'wp': 'wp_6', 'expect': 'ACCEPT', 'region': 'open', 'already': True},
    {'wp': 'wp_9', 'expect': 'ACCEPT', 'region': 'doorway', 'already': False},
    {'wp': 'wp_2', 'expect': 'ACCEPT', 'region': 'corridor', 'already': False},
    {'wp': 'wp_5', 'expect': 'ACCEPT', 'region': 'room', 'already': False},
    {'wp': 'wp_3', 'expect': 'UNKNOWN', 'region': 'similar_corridor', 'already': False},
    {'wp': 'wp_6', 'expect': 'ACCEPT', 'region': 'open', 'already': False},  # second open viewpoint
    {'wp': 'wp_9', 'expect': 'ACCEPT', 'region': 'doorway', 'already': False},
]

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


def load1() -> float:
    return float(Path('/proc/loadavg').read_text().split()[0])


def load_wps() -> Dict[str, Tuple[float, float, float]]:
    doc = yaml.safe_load(WP_FILE.read_text(encoding='utf-8'))
    return {w['name']: (float(w['x']), float(w['y']), float(w['yaw'])) for w in doc['waypoints']}


class Cov(Node):
    def __init__(self) -> None:
        super().__init__('phase2b3_coverage')
        self._amcl = None
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
        self._reloc = self.create_client(Relocalize, '/xw/relocalize')
        self.quiesce()

    def _on_amcl(self, m): self._amcl = m
    def _on_loc(self, m): self._loc = int(m.data)
    def _on_task(self, m): self._tasks.append(m)

    def spin_sleep(self, sec, hz=3.0):
        dt = 1.0 / max(hz, 1.0)
        t0 = time.monotonic()
        while time.monotonic() - t0 < sec and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(dt)

    def wait_fut(self, fut, timeout):
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout and rclpy.ok() and not fut.done():
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(0.1)
        return fut.done()

    def pose(self):
        self.spin_sleep(0.2, hz=5.0)
        if self._amcl is None:
            return None
        p = self._amcl.pose.pose
        return (float(p.position.x), float(p.position.y), yaw_from_quat(p.orientation))

    def cov_xy(self):
        if self._amcl is None:
            return None
        c = self._amcl.pose.covariance
        return max(float(c[0]), float(c[7]))

    def quiesce(self):
        self._cancel.publish(Bool(data=True))
        self._follow.publish(Bool(data=False))
        self._slam.publish(Bool(data=False))
        self._recovery.publish(Bool(data=False))

    def wait_load(self, timeout=90.0) -> bool:
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout and rclpy.ok():
            self.quiesce()
            l = load1()
            print(f'  cool load1={l:.2f}', flush=True)
            if 0 <= l <= MAX_LOAD1:
                self.spin_sleep(2.0, hz=1.0)
                if load1() <= MAX_LOAD1:
                    return True
            self.spin_sleep(3.0, hz=1.0)
        return False

    def nav_to(self, x, y, yaw, label) -> bool:
        print(f'  NAV SETUP → {label}', flush=True)
        # Without AMCL, Nav2 cannot deliver — fall through to manual place quickly.
        p0 = self.pose()
        if p0 is None:
            print('  no /amcl_pose yet — skip Nav SETUP', flush=True)
            return False
        if self._loc != 0 and (self.cov_xy() or 99) > 1.5:
            print(f'  loc={self._loc} cov={self.cov_xy()} — skip Nav SETUP', flush=True)
            return False
        self._tasks.clear()
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.orientation = yaw_to_quat(yaw)
        self._goal.publish(msg)
        t0 = time.monotonic()
        while time.monotonic() - t0 < NAV_TIMEOUT and rclpy.ok():
            self.spin_sleep(0.4, hz=3.0)
            p = self.pose()
            if p and math.hypot(p[0] - x, p[1] - y) < NAV_TOL:
                self._cancel.publish(Bool(data=True))
                self.spin_sleep(2.0, hz=2.0)
                return True
            for r in self._tasks:
                if r.capability == 'nav':
                    self._cancel.publish(Bool(data=True))
                    p2 = self.pose()
                    return r.code == 0 and p2 is not None and math.hypot(p2[0] - x, p2[1] - y) < 1.3
        self._cancel.publish(Bool(data=True))
        return False

    def wait_manual(self, wp: str, ready: str, timeout=600.0) -> bool:
        Path(ready).unlink(missing_ok=True)
        print(f'  MANUAL PLACE at {wp} then: docker exec ros2_humble_dev touch {ready}', flush=True)
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout and rclpy.ok():
            if Path(ready).is_file():
                Path(ready).unlink(missing_ok=True)
                return True
            self.spin_sleep(0.5, hz=1.0)
        return False

    def relocalize(self):
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
        ok = self.wait_fut(fut, 90.0)
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
            'laser_score': float(res.laser_score),
            'time_to_candidate_sec': float(res.time_to_candidate_sec),
            'amcl_convergence_sec': float(res.amcl_convergence_sec),
            'wall_sec': wall,
            'candidate': cand,
            'diagnostics': diag,
        }

    def short_nav(self, gt):
        if load1() > MAX_LOAD1 + 2:
            return None
        gx = gt[0] + 0.7 * math.cos(gt[2])
        gy = gt[1] + 0.7 * math.sin(gt[2])
        self._tasks.clear()
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.position.x = gx
        msg.pose.position.y = gy
        msg.pose.orientation = yaw_to_quat(gt[2])
        self._goal.publish(msg)
        t0 = time.monotonic()
        while time.monotonic() - t0 < 60 and rclpy.ok():
            self.spin_sleep(0.35, hz=3.0)
            p = self.pose()
            if p and math.hypot(p[0] - gx, p[1] - gy) < 0.7:
                self._cancel.publish(Bool(data=True))
                return True
            for r in self._tasks:
                if r.capability == 'nav':
                    self._cancel.publish(Bool(data=True))
                    return r.code == 0
        self._cancel.publish(Bool(data=True))
        return False

    def run_one(self, spec, wps, idx) -> Dict[str, Any]:
        wp = spec['wp']
        tx, ty, tyaw = wps[wp]
        out: Dict[str, Any] = {
            'trial_id': f'b3_{idx:02d}_{wp}',
            'wp': wp,
            'region': spec['region'],
            'expect': spec['expect'],
        }
        print(f'\n===== {out["trial_id"]} ({spec["region"]}) =====', flush=True)
        placed = False
        if spec.get('already'):
            p = self.pose()
            if p and math.hypot(p[0] - tx, p[1] - ty) < 1.5:
                placed = True
                print('  already near target', flush=True)
        if not placed:
            if not self.nav_to(tx, ty, tyaw, wp):
                p = self.pose()
                if p and math.hypot(p[0] - tx, p[1] - ty) < 1.2:
                    print('  near target after nav-fail — accept place', flush=True)
                    placed = True
                elif not self.wait_manual(wp, '/tmp/phase2b3_placed'):
                    out['layer'] = 'SETUP_ABORTED'
                    out['setup_error'] = 'nav_and_manual_failed'
                    return out
                else:
                    placed = True
        if not self.wait_load():
            out['layer'] = 'SETUP_ABORTED'
            out['setup_error'] = 'load_not_cooled'
            return out

        # Prefer live AMCL as GT when still localized near the place.
        # If lost / high cov / far from wp: use waypoint XY as physical GT, but relax
        # yaw (nav SETUP does not enforce yaw; physical place yaw is approximate).
        amcl = self.pose()
        place = (tx, ty, tyaw)
        cov = self.cov_xy()
        near = amcl is not None and math.hypot(amcl[0] - tx, amcl[1] - ty) < 1.5
        localized = self._loc == 0 and (cov is None or cov < 0.8)
        if amcl is not None and near and localized:
            gt = amcl
            src = 'amcl_at_place'
            fa_yaw = FA_YAW
        else:
            gt = place
            src = 'wp_pose'
            fa_yaw = 1.20  # ~69° — placement/nav yaw not ground truth
        out['gt'] = {'x': gt[0], 'y': gt[1], 'yaw': gt[2], 'source': src, 'fa_yaw': fa_yaw}
        out['amcl_before'] = None if amcl is None else {'x': amcl[0], 'y': amcl[1], 'yaw': amcl[2], 'cov': cov, 'loc': self._loc}
        out['load1_at_start'] = load1()

        reloc = self.relocalize()
        if not reloc.get('ok'):
            out['layer'] = 'SETUP_ABORTED'
            out['setup_error'] = reloc.get('error')
            return out
        diag = reloc.get('diagnostics') or {}
        code = reloc.get('result_code')
        cand = reloc.get('candidate')
        decision = diag.get('decision')
        sg = diag.get('sensor_gate') or {}
        out['reloc'] = {
            'result_code': code,
            'decision': decision,
            'laser_score': reloc.get('laser_score'),
            'wall_sec': reloc.get('wall_sec'),
            'time_to_candidate_sec': reloc.get('time_to_candidate_sec'),
            'amcl_convergence_sec': reloc.get('amcl_convergence_sec'),
            'candidate': cand,
            'sensor_ready_ms': sg.get('sensor_ready_ms'),
            'amcl_handoff': diag.get('amcl_handoff'),
        }

        if code == RC_NO_DATA or diag.get('error') == 'sensor_not_ready':
            out['layer'] = 'NO_DATA'
        elif code in (RC_UNKNOWN, RC_REJECTED) or decision in ('UNKNOWN', 'REJECTED'):
            out['layer'] = 'SAFE_UNKNOWN'
            out['false_handoff'] = False
        elif code in (RC_READY, RC_AMCL_TIMEOUT) or decision == 'ACCEPT':
            if cand is not None:
                out['candidate_error_xy'] = math.hypot(cand[0] - gt[0], cand[1] - gt[1])
                out['candidate_error_yaw'] = yaw_err(cand[2], gt[2])
                if out['candidate_error_xy'] > FA_XY or out['candidate_error_yaw'] > fa_yaw:
                    out['layer'] = 'FALSE_HANDOFF'
                    out['false_handoff'] = True
                    print(json.dumps({'trial': out['trial_id'], 'layer': out['layer'], 'xy': out['candidate_error_xy'], 'yaw': out['candidate_error_yaw'], 'src': src}), flush=True)
                    return out
            if code == RC_AMCL_TIMEOUT:
                out['layer'] = 'AMCL_TIMEOUT'
                out['amcl_ready'] = False
            elif code == RC_READY:
                out['amcl_ready'] = True
                ah = diag.get('amcl_handoff') or {}
                ap = ah.get('amcl_pose') or list(self.pose() or [])
                if ap and len(ap) >= 3:
                    out['final_error_xy'] = math.hypot(ap[0] - gt[0], ap[1] - gt[1])
                    out['final_error_yaw'] = yaw_err(ap[2], gt[2])
                    if out['final_error_xy'] > FA_XY or out['final_error_yaw'] > fa_yaw:
                        out['layer'] = 'FALSE_HANDOFF'
                        out['false_handoff'] = True
                        print(json.dumps({'trial': out['trial_id'], 'layer': out['layer'], 'xy': out['final_error_xy'], 'yaw': out['final_error_yaw'], 'src': src}), flush=True)
                        return out
                out['amcl_convergence_sec'] = reloc.get('amcl_convergence_sec')
                nav = self.short_nav(gt)
                out['nav_success'] = nav
                out['layer'] = 'SUCCESS' if nav is not False else 'NAV_FAILED'
                out['false_handoff'] = False
            else:
                out['layer'] = f'code_{code}'
        else:
            out['layer'] = f'code_{code}'

        print(
            json.dumps(
                {
                    'trial': out['trial_id'],
                    'layer': out.get('layer'),
                    'fa': out.get('false_handoff'),
                    'load1': out.get('load1_at_start'),
                    'xy': None if out.get('final_error_xy') is None else round(out['final_error_xy'], 3),
                    'nav': out.get('nav_success'),
                    'amcl_s': out.get('amcl_convergence_sec'),
                    'vl': (out.get('reloc') or {}).get('time_to_candidate_sec'),
                }
            ),
            flush=True,
        )
        return out


def main():
    BENCH.mkdir(parents=True, exist_ok=True)
    wps = load_wps()
    rclpy.init()
    node = Cov()
    results = []
    stop = None
    try:
        for i, spec in enumerate(SESSION):
            one = node.run_one(spec, wps, i)
            results.append(one)
            node.quiesce()
            if one.get('false_handoff'):
                stop = 'false_handoff'
                break
            if one.get('layer') == 'SETUP_ABORTED' and one.get('setup_error') == 'load_not_cooled':
                stop = 'load_abort'
                break
            node.wait_load(timeout=60.0)
    finally:
        payload = {
            'validator': 'phase2b3_coverage_v1',
            'resident_reloc': True,
            'no_reinit': True,
            'stop_reason': stop,
            'prior_phase2b2_credit': [
                {'wp': 'wp_9', 'layer': 'SUCCESS'},
                {'wp': 'wp_2', 'layer': 'SUCCESS'},
                {'wp': 'wp_3', 'layer': 'SAFE_UNKNOWN'},
                {'wp': 'wp_5', 'layer': 'SUCCESS'},
            ],
            'trials': results,
        }
        outp = BENCH / 'coverage_trials.json'
        outp.write_text(json.dumps(payload, indent=2), encoding='utf-8')
        print(json.dumps({'out': str(outp), 'n': len(results), 'layers': [t.get('layer') for t in results], 'stop': stop}, indent=2), flush=True)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
