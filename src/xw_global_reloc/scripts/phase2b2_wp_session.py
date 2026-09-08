#!/usr/bin/env python3
"""Phase2B2 session: agent runs test+cleanup; optional Nav2 between waypoints.

Protocol per WP:
  (optional) Nav2 place → cool load → start reloc → /xw/relocalize(apply)
  → score → kill reloc → cancel nav
NO /reinitialize_global_localization.
"""

from __future__ import annotations

import argparse
import json
import math
import subprocess
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
WP_FILE = Path('/ros2_ws/maps/waypoints/vp_pointList.yaml')
DB = Path('/ros2_ws/maps/vp/visual/keyframes')

RC_READY, RC_UNKNOWN, RC_REJECTED, RC_AMCL_TIMEOUT, RC_NO_DATA = 0, 1, 2, 3, 4
FA_XY, FA_YAW = 1.0, 0.52
NAV_TOL, NAV_YAW, NAV_TIMEOUT = 0.80, 0.55, 150.0
MAX_LOAD1 = 9.0

# Smoke remaining after wp_9 SUCCESS: corridor→similar→room→open (user: 2 then 3…)
# Map Formal KF near each web waypoint.
SESSION = [
    {'wp': 'wp_2', 'kf': 'kf_000014', 'expect': 'ACCEPT', 'region': 'corridor'},
    {'wp': 'wp_3', 'kf': 'kf_000020', 'expect': 'ACCEPT', 'region': 'similar_corridor'},
    {'wp': 'wp_5', 'kf': 'kf_000026', 'expect': 'ACCEPT', 'region': 'room'},
    {'wp': 'wp_6', 'kf': 'kf_000033', 'expect': 'ACCEPT', 'region': 'open'},
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


def load_waypoints() -> Dict[str, Tuple[float, float, float]]:
    doc = yaml.safe_load(WP_FILE.read_text(encoding='utf-8'))
    out = {}
    for w in doc.get('waypoints') or []:
        out[str(w['name'])] = (float(w['x']), float(w['y']), float(w['yaw']))
    return out


def load_kf(kid: str) -> Tuple[float, float, float]:
    meta = yaml.safe_load((DB / kid / 'meta.yaml').read_text(encoding='utf-8'))
    mp = meta['map_pose']
    return float(mp['x']), float(mp['y']), float(mp['yaw'])


def sh(cmd: str) -> None:
    subprocess.run(['bash', '-lc', cmd], check=False)


def start_reloc() -> None:
    stop_reloc()
    time.sleep(1)
    subprocess.Popen(
        [
            'bash', '-lc',
            'source /ros2_ws/scripts/ros_env.sh; source /ros2_ws/install/setup.bash; '
            'ros2 launch xw_global_reloc reloc_poc.launch.py allow_amcl_handoff:=true '
            '>/ros2_ws/bench/phase2b2_low_load_handoff_2026-09-08/reloc_launch.log 2>&1',
        ],
        start_new_session=True,
    )
    # wait until service up
    for _ in range(30):
        time.sleep(1)
        r = subprocess.run(
            ['bash', '-lc', 'source /ros2_ws/scripts/ros_env.sh; ros2 node list 2>/dev/null | grep -q xw_global_reloc_poc'],
            check=False,
        )
        if r.returncode == 0:
            break
    time.sleep(1)


def stop_reloc() -> None:
    sh('pkill -f global_reloc_poc || true; pkill -f reloc_poc.launch || true')
    time.sleep(1)


class Session(Node):
    def __init__(self) -> None:
        super().__init__('phase2b2_wp_session')
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
        self._rgb = self.create_publisher(Bool, '/xw/reloc/rgb_request', _LATCH)
        self._reloc = self.create_client(Relocalize, '/xw/relocalize')
        self.quiesce()

    def _on_amcl(self, m): self._amcl = m
    def _on_loc(self, m): self._loc = int(m.data)
    def _on_task(self, m): self._tasks.append(m)

    def spin_sleep(self, sec: float, hz: float = 3.0) -> None:
        dt = 1.0 / max(hz, 1.0)
        t0 = time.monotonic()
        while time.monotonic() - t0 < sec and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(dt)

    def wait_fut(self, fut, timeout: float) -> bool:
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout and rclpy.ok() and not fut.done():
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(0.1)
        return fut.done()

    def pose(self) -> Optional[Tuple[float, float, float]]:
        self.spin_sleep(0.2, hz=5.0)
        if self._amcl is None:
            return None
        p = self._amcl.pose.pose
        return (float(p.position.x), float(p.position.y), yaw_from_quat(p.orientation))

    def cov_xy(self) -> Optional[float]:
        if self._amcl is None:
            return None
        c = self._amcl.pose.covariance
        return max(float(c[0]), float(c[7]))

    def quiesce(self) -> None:
        self._cancel.publish(Bool(data=True))
        self._follow.publish(Bool(data=False))
        self._slam.publish(Bool(data=False))
        self._recovery.publish(Bool(data=False))
        self._rgb.publish(Bool(data=False))

    def wait_load(self, max_load: float, timeout: float = 120.0) -> bool:
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout and rclpy.ok():
            self.quiesce()
            l = load1()
            print(f'  cool load1={l:.2f} (need ≤{max_load})', flush=True)
            if 0 <= l <= max_load:
                self.spin_sleep(3.0, hz=1.0)
                if load1() <= max_load:
                    return True
            self.spin_sleep(4.0, hz=1.0)
        return False

    def nav_to(self, x: float, y: float, yaw: float, label: str) -> bool:
        print(f'  NAV SETUP → {label} ({x:.2f},{y:.2f})', flush=True)
        self.quiesce()
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
            if p and math.hypot(p[0] - x, p[1] - y) < NAV_TOL and yaw_err(p[2], yaw) < NAV_YAW:
                self.spin_sleep(2.0, hz=2.0)
                self._cancel.publish(Bool(data=True))
                return True
            for r in self._tasks:
                if r.capability == 'nav':
                    self._cancel.publish(Bool(data=True))
                    p2 = self.pose()
                    return (
                        r.code == 0
                        and p2 is not None
                        and math.hypot(p2[0] - x, p2[1] - y) < 1.2
                    )
        self._cancel.publish(Bool(data=True))
        return False

    def relocalize(self) -> Dict[str, Any]:
        if not self._reloc.wait_for_service(timeout_sec=8.0):
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
            'laser_score': float(res.laser_score),
            'time_to_candidate_sec': float(res.time_to_candidate_sec),
            'amcl_convergence_sec': float(res.amcl_convergence_sec),
            'wall_sec': wall,
            'candidate': cand,
            'diagnostics': diag,
            'stage_timings': timings,
        }

    def short_nav(self, gt: Tuple[float, float, float]) -> Optional[bool]:
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
        while time.monotonic() - t0 < 60.0 and rclpy.ok():
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

    def run_one(self, spec: Dict[str, Any], wps: Dict[str, Tuple[float, float, float]], need_nav: bool) -> Dict[str, Any]:
        wp = spec['wp']
        tx, ty, tyaw = wps[wp]
        kf_pose = load_kf(spec['kf'])
        out: Dict[str, Any] = {
            'wp': wp,
            'region': spec['region'],
            'keyframe_id': spec['kf'],
            'expect': spec['expect'],
            'wp_pose': {'x': tx, 'y': ty, 'yaw': tyaw},
        }
        print(f'\n===== TEST {wp} ({spec["region"]}) =====', flush=True)

        if need_nav:
            if not self.nav_to(tx, ty, tyaw, wp):
                out['layer'] = 'SETUP_ABORTED'
                out['setup_error'] = 'nav_to_wp_failed'
                print(json.dumps({'wp': wp, 'layer': out['layer']}), flush=True)
                return out
        else:
            print(f'  assume already at {wp}', flush=True)
            self.spin_sleep(2.0, hz=2.0)

        if not self.wait_load(MAX_LOAD1, timeout=100.0):
            out['layer'] = 'SETUP_ABORTED'
            out['setup_error'] = 'load_not_cooled'
            print(json.dumps({'wp': wp, 'layer': out['layer'], 'load1': load1()}), flush=True)
            return out

        # GT: live AMCL if healthy near WP, else WP pose
        amcl = self.pose()
        place = (tx, ty, tyaw)
        if (
            amcl
            and math.hypot(amcl[0] - tx, amcl[1] - ty) < 1.5
            and self._loc == 0
            and (self.cov_xy() is None or self.cov_xy() < 0.8)
        ):
            gt = amcl
            src = 'amcl_at_place'
        else:
            gt = place
            src = 'wp_pose'
        out['gt'] = {'x': gt[0], 'y': gt[1], 'yaw': gt[2], 'source': src}
        out['load1_at_start'] = load1()

        print(f'  start reloc load1={out["load1_at_start"]:.2f}', flush=True)
        start_reloc()
        self.spin_sleep(2.0, hz=2.0)
        reloc = self.relocalize()
        self._rgb.publish(Bool(data=False))
        # Always kill reloc after call — prevents idle CPU leak
        stop_reloc()
        self.quiesce()

        if not reloc.get('ok'):
            out['layer'] = 'SETUP_ABORTED'
            out['setup_error'] = reloc.get('error')
            print(json.dumps({'wp': wp, 'layer': out['layer']}), flush=True)
            return out

        diag = reloc.get('diagnostics') or {}
        timings = reloc.get('stage_timings') or {}
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
            'sensor_ready_ms': sg.get('sensor_ready_ms') or timings.get('sensor_ready_ms'),
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
                if out['candidate_error_xy'] > FA_XY or out['candidate_error_yaw'] > FA_YAW:
                    out['layer'] = 'FALSE_HANDOFF'
                    out['false_handoff'] = True
                    print(json.dumps({'wp': wp, 'layer': out['layer'], 'err_xy': out['candidate_error_xy'], 'err_yaw': out['candidate_error_yaw']}), flush=True)
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
                    if out['final_error_xy'] > FA_XY or out['final_error_yaw'] > FA_YAW:
                        out['layer'] = 'FALSE_HANDOFF'
                        out['false_handoff'] = True
                        print(json.dumps({'wp': wp, 'layer': out['layer']}), flush=True)
                        return out
                out['amcl_convergence_sec'] = reloc.get('amcl_convergence_sec')
                nav = self.short_nav(gt)
                out['nav_success'] = nav
                if nav is False:
                    out['layer'] = 'NAV_FAILED'
                else:
                    out['layer'] = 'SUCCESS'
                    if nav is None:
                        out['nav_skipped'] = 'load_high'
                out['false_handoff'] = False
            else:
                out['layer'] = f'code_{code}'
        else:
            out['layer'] = f'code_{code}'

        print(
            json.dumps(
                {
                    'wp': wp,
                    'layer': out.get('layer'),
                    'fa': out.get('false_handoff'),
                    'load1': out.get('load1_at_start'),
                    'sensor_ms': (out.get('reloc') or {}).get('sensor_ready_ms'),
                    'xy': None if out.get('final_error_xy') is None else round(out['final_error_xy'], 3),
                    'nav': out.get('nav_success'),
                    'amcl_s': out.get('amcl_convergence_sec'),
                }
            ),
            flush=True,
        )
        return out


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument('--start-wp', default='wp_2')
    ap.add_argument('--already-at-first', action='store_true', default=True)
    ap.add_argument('--out', default=str(BENCH / 'handoff_session_wp2plus.json'))
    args = ap.parse_args()

    BENCH.mkdir(parents=True, exist_ok=True)
    wps = load_waypoints()
    # start from requested wp
    seq = []
    seen = False
    for s in SESSION:
        if s['wp'] == args.start_wp:
            seen = True
        if seen:
            seq.append(s)
    if not seq:
        raise SystemExit(f'unknown start {args.start_wp}')

    stop_reloc()
    rclpy.init()
    node = Session()
    results: List[Dict[str, Any]] = []
    stop = None
    try:
        for i, spec in enumerate(seq):
            need_nav = not (i == 0 and args.already_at_first)
            one = node.run_one(spec, wps, need_nav=need_nav)
            results.append(one)
            node.quiesce()
            stop_reloc()
            if one.get('layer') == 'SETUP_ABORTED' and one.get('setup_error') == 'load_not_cooled':
                stop = 'load_abort'
                print('STOP load — not retrying', flush=True)
                break
            if one.get('false_handoff'):
                stop = 'false_handoff'
                break
            # cool between sites
            print('  inter-trial cool + kill reloc…', flush=True)
            node.wait_load(MAX_LOAD1, timeout=90.0)
    finally:
        stop_reloc()
        node.quiesce()
        payload = {
            'validator': 'phase2b2_wp_session_v1',
            'start_wp': args.start_wp,
            'stop_reason': stop,
            'trials': results,
            'prior_wp9': 'SUCCESS (earlier smoke)',
        }
        Path(args.out).write_text(json.dumps(payload, indent=2), encoding='utf-8')
        print(json.dumps({'out': args.out, 'n': len(results), 'stop': stop, 'layers': [t.get('layer') for t in results]}, indent=2), flush=True)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
