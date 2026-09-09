#!/usr/bin/env python3
"""Phase2C-C4B3.3 missing evidence: manual carry + dedicated hard-negative.

Does not retune laser/AMCL/ORB/NPU/Depth. Does not publish a recovery
/initialpose. Operator pose uses the official Web API only after UNKNOWN.
Physical placement is confirmed by ops markers. No Phase2D / C5.
"""

from __future__ import annotations

import importlib.util
import json
import math
import os
import time
from pathlib import Path
from typing import Any, Dict, Optional

import rclpy

OUT = Path(os.environ.get('C4B33_OUT', '/ros2_ws/bench/phase2c_c4b33_2026-09-09'))
OPS = OUT / 'ops'
HARD = {'label': 'similar_corridor_wp3', 'x': -9.209, 'y': 1.191, 'yaw': 3.386}
ALIGN_M = 0.45


def _load_cascade():
    path = Path('/ros2_ws/src/xw_phase2c/scripts/phase2c_c4b32_cascade.py')
    spec = importlib.util.spec_from_file_location('c4b32_cascade', path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


C = _load_cascade()


def write_text(path: Path, text: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text, encoding='utf-8')
    print(text, flush=True)


def write_json(name: str, payload: Dict[str, Any]) -> None:
    OUT.mkdir(parents=True, exist_ok=True)
    (OUT / name).write_text(json.dumps(payload, indent=2, default=str), encoding='utf-8')
    print(f'WROTE {name}', flush=True)


def marker(name: str) -> bool:
    return (OPS / name).is_file()


def clear_marker(name: str) -> None:
    p = OPS / name
    if p.is_file():
        p.unlink()


def read_pose_json(name: str) -> Optional[Dict[str, Any]]:
    p = OPS / name
    if not p.is_file():
        return None
    try:
        data = json.loads(p.read_text(encoding='utf-8'))
    except json.JSONDecodeError:
        return None
    if not isinstance(data, dict):
        return None
    if 'x' not in data or 'y' not in data:
        return None
    return data


def xy_dist(a: Optional[Dict[str, Any]], b: Optional[Dict[str, Any]]) -> Optional[float]:
    if not a or not b or 'x' not in a or 'y' not in b:
        return None
    return float(math.hypot(float(a['x']) - float(b['x']), float(a['y']) - float(b['y'])))


def aligned(amcl: Optional[Dict[str, Any]], last_good: Dict[str, Any]) -> bool:
    if not amcl or not last_good or last_good.get('missing') or last_good.get('error'):
        return False
    if not bool(last_good.get('laser_verified')):
        return False
    d = xy_dist(amcl, last_good)
    return d is not None and d <= ALIGN_M


def stage_laser(lr: Dict[str, Any], stage: str) -> Optional[float]:
    for s in lr.get('stages') or []:
        if s.get('stage') == stage and isinstance(s.get('laser_score'), (int, float)):
            return float(s['laser_score'])
    return None


def stage_row(lr: Dict[str, Any], stage: str) -> Dict[str, Any]:
    return next((s for s in (lr.get('stages') or []) if s.get('stage') == stage), {})


def motion_after(samples, t0: float) -> list:
    return [s for s in samples if s.get('t', 0) >= t0]


def wait_pred(n, pred, timeout: float, label: str) -> bool:
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout and rclpy.ok():
        n.spin(0.2)
        if pred():
            print(f'OK {label}', flush=True)
            return True
    print(f'TIMEOUT {label}', flush=True)
    return False


def wait_marker_or_pred(n, name: str, pred, timeout: float) -> Dict[str, Any]:
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout and rclpy.ok():
        n.spin(0.25)
        if marker(name):
            return {'ok': True, 'how': name, 'wait_sec': round(time.monotonic() - t0, 2)}
        if pred():
            return {'ok': True, 'how': 'pred', 'wait_sec': round(time.monotonic() - t0, 2)}
    return {'ok': False, 'how': 'timeout', 'wait_sec': round(time.monotonic() - t0, 2)}


def ensure_nav(n) -> Dict[str, Any]:
    started = n.set_mode(2, json.dumps({'map_name': C.MAP}))
    n.spin(1.0)
    return started


def induce_lost(n, kind: str) -> Dict[str, Any]:
    report: Dict[str, Any] = {
        'kind': kind,
        'induction': 'localization_status=3 while NAV mode on; no /initialpose; no nav goal',
        'pose_A': dict(n.amcl) if n.amcl else None,
        'last_good': C.read_pose_file(),
    }
    n.lost_results.clear()
    n.initialposes.clear()
    n.cmd_samples.clear()
    n.goals.clear()
    n.owners.clear()
    n.loc_states.clear()
    nav = ensure_nav(n)
    report['nav'] = nav
    # No goal. A goal would drive the old pose before STOP.
    t_induce = time.time()
    n.inject_status(3, 2.4)
    n.wait_until(lambda: n.loc in ('LOST', 'RECOVERING', 'NEED_OPERATOR') or bool(n.lost_results), 25.0)
    n.wait_until(lambda: bool(n.lost_results) and n.lost_results[-1].get('final') in ('READY', 'UNKNOWN'), 180.0)
    n.spin(1.2)
    lr = n.lost_results[-1] if n.lost_results else {}
    casc = C._cascade(lr)
    resume = lr.get('resume_policy') or {}
    r1 = stage_row(lr, 'R1')
    r2 = stage_row(lr, 'R2')
    r3 = stage_row(lr, 'R3')
    motion = motion_after(n.cmd_samples, t_induce)
    report.update({
        **casc,
        'lost_result': lr,
        'snapshot': lr.get('snapshot'),
        'loc': n.loc,
        'blocked': n.blocked,
        'saw_lost': any(s.get('loc') == 'LOST' for s in n.loc_states),
        'saw_need_operator': any(s.get('loc') == 'NEED_OPERATOR' for s in n.loc_states),
        'initialpose_count': len(n.initialposes),
        'initialposes': n.initialposes[-8:],
        'owners': C._owners_ok(n.owners),
        'cmd_motion_samples': motion[-8:],
        'motion_count': len(motion),
        'replan_count': len([g for g in n.goals if g['t'] >= t_induce]),
        'amcl_end': n.amcl,
        'r1_laser': stage_laser(lr, 'R1'),
        'r2_laser': stage_laser(lr, 'R2'),
        'r1_seeded': bool(r1.get('seeded')),
        'r2_seeded': bool(r2.get('seeded')),
        'r3_seeded': bool(r3.get('seeded')),
        'resume_policy': resume,
        'nav_en': n.nav_en,
        'follow_en': n.follow_en,
        'recharge_en': n.recharge_en,
    })
    return report


def score_carry(report: Dict[str, Any]) -> bool:
    lr_final = report.get('final')
    r1 = report.get('r1_code')
    r2 = report.get('r2_code')
    r3 = report.get('r3_code')
    r1_laser = report.get('r1_laser')
    r2_laser = report.get('r2_laser')
    old_rejected = (
        r1 == 'R1_CURRENT_REJECT'
        and r2 == 'R2_LAST_GOOD_REJECT'
        and not report.get('r1_seeded')
        and not report.get('r2_seeded')
        and isinstance(r1_laser, float)
        and r1_laser < 0.38
        and isinstance(r2_laser, float)
        and r2_laser < 0.38
        and report.get('selected_recovery_path') not in ('R1', 'R2')
    )
    if not old_rejected:
        return False
    if report.get('owners', {}).get('legacy_seen'):
        return False
    if int(report.get('sequential_fallback_seed_count') or 0) > 1:
        return False
    seeds = int(report.get('seed_count') or 0)
    if lr_final == 'READY':
        return bool(
            r3 == 'R3_VISUAL_ACCEPT'
            and report.get('selected_recovery_path') == 'R3'
            and seeds == 1
            and report.get('loc') == 'READY'
            and report.get('blocked') is False
        )
    if lr_final == 'UNKNOWN':
        resume = report.get('resume_policy') or {}
        return bool(
            r3 == 'R3_VISUAL_UNKNOWN'
            and seeds == 0
            and report.get('loc') == 'NEED_OPERATOR'
            and report.get('blocked') is True
            and resume.get('nav') == 'forbidden'
            and resume.get('follow') == 'forbidden'
            and resume.get('recharge') == 'forbidden'
            and report.get('motion_count') == 0
            and report.get('replan_count') == 0
        )
    return False


def score_hard(report: Dict[str, Any]) -> bool:
    resume = report.get('resume_policy') or {}
    r1 = report.get('r1_code')
    r2 = report.get('r2_code')
    return bool(
        report.get('final') == 'UNKNOWN'
        and report.get('r3_code') == 'R3_VISUAL_UNKNOWN'
        and r1 in ('R1_CURRENT_REJECT', 'R1_CURRENT_SKIP')
        and r2 in ('R2_LAST_GOOD_REJECT', 'R2_LAST_GOOD_SKIP')
        and not report.get('r1_seeded')
        and not report.get('r2_seeded')
        and not report.get('r3_seeded')
        and int(report.get('seed_count') or 0) == 0
        and report.get('loc') == 'NEED_OPERATOR'
        and report.get('blocked') is True
        and resume.get('nav') == 'forbidden'
        and resume.get('follow') == 'forbidden'
        and resume.get('recharge') == 'forbidden'
        and report.get('motion_count') == 0
        and report.get('replan_count') == 0
        and report.get('follow_en') is not True
        and report.get('recharge_en') is not True
        and not report.get('owners', {}).get('legacy_seen')
        and report.get('selected_recovery_path') not in ('R1', 'R2')
    )


def run_operator(n, pose: Dict[str, Any]) -> Dict[str, Any]:
    report: Dict[str, Any] = {'pose': pose, 'snap0': n.snap()}
    if n.loc != 'NEED_OPERATOR' or n.blocked is not True:
        report['pass'] = False
        report['reason'] = 'not_in_need_operator'
        report['loc'] = n.loc
        report['blocked'] = n.blocked
        return report
    n.results.clear()
    n.initialposes.clear()
    web = C.web_initialpose(float(pose['x']), float(pose['y']), float(pose.get('yaw') or 0.0))
    saw_verifying = False
    t0 = time.monotonic()
    while time.monotonic() - t0 < 28.0:
        n.spin(0.2)
        if n.boot.get('state') == 'VERIFYING_OPERATOR_POSE' or n.loc == 'VERIFYING_OPERATOR_POSE':
            saw_verifying = True
        if n.loc == 'READY' and n.blocked is False:
            break
    n.spin(1.0)
    owners = [o for o in n.owners if o.get('owner') == 'operator']
    report.update({
        'web': web,
        'saw_verifying': saw_verifying,
        'saw_owner_operator': bool(owners),
        'loc': n.loc,
        'blocked': n.blocked,
        'initialpose_count': len(n.initialposes),
        'canonical_tail': n.canonical[-8:],
        'boot_state': n.boot.get('state'),
    })
    report['pass'] = bool(
        saw_verifying
        and bool(owners)
        and n.loc == 'READY'
        and n.blocked is False
    )
    return report


def wait_ready_aligned(n, timeout: float) -> Dict[str, Any]:
    write_text(
        OPS / 'WAIT_READY_A.txt',
        'C4B3.3 Test 1 setup.\n'
        'Need READY at physical A, current AMCL = A, laser_verified last_good = A.\n'
        'Do not send /initialpose unless the robot is already NEED_OPERATOR and the Web pose is the true A.\n'
        'Stop the robot. Leave it still until this file disappears or PLACE_A_CONFIRMED is created.\n'
        f'Alignment gate: XY <= {ALIGN_M} m and last_good.laser_verified=true.\n',
    )
    ok = wait_pred(
        n,
        lambda: n.loc == 'READY' and n.blocked is False and aligned(n.amcl, C.read_pose_file()),
        timeout,
        'ready_aligned_A',
    )
    snap = {
        'ok': ok,
        'loc': n.loc,
        'blocked': n.blocked,
        'status': n.status,
        'amcl': n.amcl,
        'last_good': C.read_pose_file(),
        'align_m': xy_dist(n.amcl, C.read_pose_file()),
    }
    write_json('ready_A.json', snap)
    return snap


def main() -> None:
    OPS.mkdir(parents=True, exist_ok=True)
    for name in ('PLACE_CARRY_DONE', 'PLACE_HARD_DONE'):
        clear_marker(name)
    rclpy.init()
    n = C.Watch()
    summary: Dict[str, Any] = {'started': time.time()}
    try:
        n.spin(1.2)
        write_json('baseline.json', {'snap': n.snap(), 'last_good': C.read_pose_file()})
        ready = wait_ready_aligned(n, 600.0)
        summary['ready_A'] = ready
        if not ready['ok']:
            summary['manual_carry'] = {'pass': False, 'reason': 'not_ready_aligned_A'}
            summary['hard_negative'] = {'pass': False, 'reason': 'blocked_by_setup'}
            summary['gate'] = 'NO'
            write_json('summary.json', summary)
            return

        pose_a = dict(n.amcl)
        n.set_mode(0)
        n.spin(0.8)
        write_text(
            OPS / 'WAIT_CARRY.txt',
            'C4B3.3 Test 1 — Manual Carry.\n'
            f'A recorded: x={pose_a["x"]:.3f} y={pose_a["y"]:.3f} yaw={pose_a["yaw"]:.3f}\n'
            'Robot is stopped. Physically carry it to a clearly different place B.\n'
            'Do NOT send /initialpose. Do NOT call reinitialize_global_localization.\n'
            'Set it down. Then: touch ops/PLACE_CARRY_DONE\n'
            'Optional: write ops/PLACE_B.json {"x":..,"y":..,"yaw":..,"label":".."}\n',
        )
        carry_wait = wait_marker_or_pred(n, 'PLACE_CARRY_DONE', lambda: False, 900.0)
        if not carry_wait['ok']:
            summary['manual_carry'] = {'pass': False, 'reason': 'carry_not_confirmed', 'pose_A': pose_a}
            summary['hard_negative'] = {'pass': False, 'reason': 'blocked_by_carry'}
            summary['gate'] = 'NO'
            write_json('summary.json', summary)
            return

        b_file = read_pose_json('PLACE_B.json')
        carry = induce_lost(n, 'manual_carry')
        carry['pose_A'] = pose_a
        carry['pose_B_file'] = b_file
        carry['amcl_still_near_A'] = xy_dist(pose_a, n.amcl if carry.get('final') != 'READY' else pose_a)
        if carry.get('final') == 'READY' and carry.get('amcl_end'):
            carry['pose_B_recovered'] = carry['amcl_end']
            carry['A_to_B_m'] = xy_dist(pose_a, carry['amcl_end'])
        elif b_file:
            carry['A_to_B_m'] = xy_dist(pose_a, b_file)
        carry['pass'] = score_carry(carry)
        write_json('manual_carry.json', carry)
        summary['manual_carry'] = {
            'pass': carry['pass'],
            'r1_code': carry.get('r1_code'),
            'r2_code': carry.get('r2_code'),
            'r3_code': carry.get('r3_code'),
            'r1_laser': carry.get('r1_laser'),
            'r2_laser': carry.get('r2_laser'),
            'final': carry.get('final'),
            'selected_recovery_path': carry.get('selected_recovery_path'),
            'seed_count': carry.get('seed_count'),
            'A_to_B_m': carry.get('A_to_B_m'),
            'loc': carry.get('loc'),
            'blocked': carry.get('blocked'),
        }

        if n.loc == 'NEED_OPERATOR':
            write_text(
                OPS / 'WAIT_MID_OPERATOR.txt',
                'Carry ended NEED_OPERATOR. Before the hard-negative, restore READY at true B\n'
                'with the official Web initialpose, then leave the robot still until last_good matches AMCL.\n'
                'Do not start the hard-scene carry until WAIT_HARD.txt appears.\n',
            )
            mid_ok = wait_pred(
                n,
                lambda: n.loc == 'READY' and n.blocked is False and aligned(n.amcl, C.read_pose_file()),
                240.0,
                'mid_ready_after_carry_unknown',
            )
            summary['mid_operator'] = {'ok': mid_ok, 'loc': n.loc, 'amcl': n.amcl, 'last_good': C.read_pose_file()}
            if not mid_ok:
                summary['hard_negative'] = {'pass': False, 'reason': 'not_ready_before_hard'}
                summary['gate'] = 'NO'
                write_json('summary.json', summary)
                return

        pose_before_hard = dict(n.amcl) if n.amcl else None
        n.set_mode(0)
        n.spin(0.6)
        write_text(
            OPS / 'WAIT_HARD.txt',
            'C4B3.3 Test 2 — Dedicated hard-negative.\n'
            'Place the robot at known hard scene similar_corridor_wp3\n'
            f'  x={HARD["x"]} y={HARD["y"]} yaw={HARD["yaw"]}\n'
            'Do not reuse the false-good belief injection. Do NOT send /initialpose.\n'
            'Set it down. Then: touch ops/PLACE_HARD_DONE\n',
        )
        hard_wait = wait_marker_or_pred(n, 'PLACE_HARD_DONE', lambda: False, 900.0)
        if not hard_wait['ok']:
            summary['hard_negative'] = {'pass': False, 'reason': 'hard_not_confirmed'}
            summary['gate'] = 'NO'
            write_json('summary.json', summary)
            return

        hard = induce_lost(n, 'hard_negative')
        hard['scene'] = HARD
        hard['pose_before'] = pose_before_hard
        r3 = stage_row(hard.get('lost_result') or {}, 'R3')
        hard['r3_reason'] = r3.get('reason')
        hard['pass'] = score_hard(hard)
        write_json('hard_negative.json', hard)
        summary['hard_negative'] = {
            'pass': hard['pass'],
            'r1_code': hard.get('r1_code'),
            'r2_code': hard.get('r2_code'),
            'r3_code': hard.get('r3_code'),
            'r3_reason': hard.get('r3_reason'),
            'final': hard.get('final'),
            'loc': hard.get('loc'),
            'blocked': hard.get('blocked'),
            'motion_count': hard.get('motion_count'),
            'seed_count': hard.get('seed_count'),
        }

        if hard.get('final') == 'UNKNOWN' and n.loc == 'NEED_OPERATOR':
            op_pose = read_pose_json('PLACE_HARD.json') or HARD
            op = run_operator(n, op_pose)
            write_json('operator_recovery.json', op)
            summary['operator_recovery'] = {
                'pass': op.get('pass'),
                'saw_verifying': op.get('saw_verifying'),
                'loc': op.get('loc'),
                'blocked': op.get('blocked'),
            }
        else:
            summary['operator_recovery'] = {'pass': False, 'reason': 'hard_not_unknown'}

        n.spin(2.0)
        summary['gate'] = 'YES' if (
            summary.get('manual_carry', {}).get('pass')
            and summary.get('hard_negative', {}).get('pass')
            and summary.get('operator_recovery', {}).get('pass')
        ) else 'NO'
        write_json('summary.json', summary)
    finally:
        n.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
