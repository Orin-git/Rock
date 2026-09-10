#!/usr/bin/env python3
"""Phase2C-C4B3 final smoke observer. No production retune. No recovery /initialpose."""

from __future__ import annotations

import argparse
import importlib.util
import json
import math
import time
from pathlib import Path
from typing import Any, Dict, List

import rclpy

OUT = Path("/ros2_ws/bench/phase2c_c4b3_final_2026-09-09")
MAP = "vp"


def _cascade():
    path = Path("/ros2_ws/src/xw_phase2c/scripts/phase2c_c4b32_cascade.py")
    spec = importlib.util.spec_from_file_location("c4b32_cascade", path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


C = _cascade()


def dump(name: str, payload: Dict[str, Any]) -> None:
    OUT.mkdir(parents=True, exist_ok=True)
    (OUT / name).write_text(json.dumps(payload, indent=2, default=str), encoding="utf-8")
    keys = (
        "pass", "outcome", "selected_path", "final", "reason", "p2_result", "p2_score",
        "p3_ran", "r1_code", "r2_code", "r3_code", "loc", "blocked",
    )
    print(json.dumps({k: payload[k] for k in keys if k in payload}, indent=2, default=str), flush=True)


def owners(rows: List[Dict[str, Any]]) -> Dict[str, Any]:
    names = [o.get("owner") for o in rows if o.get("owner")]
    return {
        "owners": names[-20:],
        "legacy_seen": any(n == "legacy" for n in names),
    }


def stage(result: Dict[str, Any], name: str) -> Dict[str, Any]:
    for s in result.get("stages") or []:
        if s.get("stage") == name:
            return s
    return {}


def wait_boot(n, timeout: float) -> Dict[str, Any]:
    n.results.clear()
    n.initialposes.clear()
    t0 = time.monotonic()
    n.wait_until(
        lambda: bool(n.boot.get("busy")) or n.boot.get("state") in (
            "PRE_LOCALIZATION_READY", "TRY_CHARGER", "TRY_LAST_GOOD",
            "TRY_VISUAL_LASER", "WAIT_AMCL_SETTLE", "POST_SEED_AMCL_READY",
        ),
        min(30.0, timeout),
    )
    ok = n.wait_until(
        lambda: (not n.boot.get("busy")) and n.boot.get("state") in (
            "READY", "UNKNOWN", "SENSOR_TIMEOUT",
        ),
        timeout,
    )
    n.spin(1.2)
    return {"settled": ok, "elapsed_sec": round(time.monotonic() - t0, 2), "result": n.results[-1] if n.results else {}}


def start_nav(n) -> Dict[str, Any]:
    n.initialposes.clear()
    n.results.clear()
    n.owners.clear()
    idle = n.set_mode(0)
    n.spin(1.2)
    started = n.set_mode(2, json.dumps({"map_name": MAP}))
    return {"idle": idle, "nav": started, "last_good": C.read_pose_file()}


def pack_boot(n, label: str, started: Dict[str, Any], watched: Dict[str, Any]) -> Dict[str, Any]:
    result = watched.get("result") or {}
    p1, p2 = stage(result, "P1"), stage(result, "P2")
    p3s = [s for s in (result.get("stages") or []) if s.get("stage") == "P3"]
    ms = result.get("milestones") or {}
    ip_m, tf_m = ms.get("initialpose_publish_mono"), ms.get("first_map_odom_mono")
    # Cold break is per-session seed vs first map->odom of this seed, not the process-lifetime TF.
    cold = result.get("cold_break") or {}
    order_ok = (
        cold.get("map_odom_required_in_pre") is False
        and ip_m is not None
        and (tf_m is None or float(ip_m) < float(tf_m) or ms.get("map_odom_before_seed") is False)
    )
    if ms.get("map_odom_before_seed") is False and ip_m is not None:
        order_ok = True
    own = owners(n.owners)
    return {
        "label": label,
        "started": started,
        "selected_path": result.get("selected_path") or n.boot.get("selected_path"),
        "final": result.get("final") or n.boot.get("state"),
        "p1": {"result": p1.get("result"), "reason": p1.get("reason"), "score": p1.get("score", p1.get("laser_score"))},
        "p2_result": p2.get("result"),
        "p2_reason": p2.get("reason"),
        "p2_score": p2.get("score", p2.get("laser_score")),
        "p2_amcl": p2.get("amcl") if isinstance(p2.get("amcl"), dict) else {},
        "p3_ran": bool(p3s),
        "p3": [{"result": s.get("result"), "reason": s.get("reason"), "score": s.get("score", s.get("laser_score"))} for s in p3s],
        "sequential_fallback_seed_count": result.get("sequential_fallback_seed_count"),
        "initialpose_count": len(n.initialposes),
        "initialposes": n.initialposes[-8:],
        "initialpose_before_map_odom": order_ok,
        "map_odom_before_seed": ms.get("map_odom_before_seed"),
        "milestones": ms,
        "cold_break": cold,
        "owners": own,
        "snap_end": n.snap(),
        "power": n.power,
        "elapsed_sec": watched.get("elapsed_sec"),
        "legacy_blind_seed": own["legacy_seen"],
        "boot_result": result,
        "loc": n.loc,
        "blocked": n.blocked,
    }


def run_boot(n, label: str, timeout: float) -> Dict[str, Any]:
    return pack_boot(n, label, start_nav(n), wait_boot(n, timeout))


def induce_lost(n, with_goal: bool) -> Dict[str, Any]:
    n.lost_results.clear()
    n.initialposes.clear()
    n.cmd_samples.clear()
    n.owners.clear()
    n.loc_states.clear()
    t0 = time.time()
    goal = None
    if with_goal and n.amcl:
        pose = n.amcl
        goal = {"x": float(pose["x"]) + 0.6, "y": float(pose["y"]), "yaw": float(pose["yaw"])}
        n.publish_goal(goal["x"], goal["y"], goal["yaw"])
        n.spin(1.5)
    n.inject_status(3, 6.0)
    n.wait_until(lambda: n.loc in ("LOST", "RECOVERING", "NEED_OPERATOR") or bool(n.lost_results), 25.0)
    n.wait_until(lambda: bool(n.lost_results) and n.lost_results[-1].get("final") in ("READY", "UNKNOWN"), 180.0)
    n.spin(1.2)
    lr = n.lost_results[-1] if n.lost_results else {}
    r1, r2, r3 = stage(lr, "R1"), stage(lr, "R2"), stage(lr, "R3")
    report = {
        "goal": goal,
        "lost_result": lr,
        "saw_lost": any(s.get("loc") == "LOST" for s in n.loc_states),
        "loc_states": n.loc_states[-24:],
        "initialpose_count": len(n.initialposes),
        "initialposes": n.initialposes[-8:],
        "owners": owners(n.owners),
        "motion_count": len([s for s in n.cmd_samples if s.get("t", 0) >= t0]),
        "amcl_end": n.amcl,
        "loc": n.loc,
        "blocked": n.blocked,
        "nav_en": n.nav_en,
        "follow_en": n.follow_en,
        "recharge_en": n.recharge_en,
        "snap_end": n.snap(),
        "r1_laser": r1.get("laser_score"),
        "r2_laser": r2.get("laser_score"),
        "r3_laser": r3.get("laser_score"),
        "r1_seeded": bool(r1.get("seeded")),
        "r2_seeded": bool(r2.get("seeded")),
        "r3_seeded": bool(r3.get("seeded")),
        "r3_reason": r3.get("reason"),
    }
    report.update(C._cascade(lr))
    return report


def wait_verified(n, timeout: float) -> Dict[str, Any]:
    t0 = time.monotonic()
    cur = C.read_pose_file()
    while time.monotonic() - t0 < timeout:
        n.spin(1.0)
        cur = C.read_pose_file()
        if cur.get("laser_verified") and n.amcl:
            if math.hypot(float(cur.get("x", 0)) - n.amcl["x"], float(cur.get("y", 0)) - n.amcl["y"]) <= 0.6:
                return cur
    return cur


def run_t1(n) -> None:
    n.spin(0.8)
    report = run_boot(n, "t1_cold_nav", 220.0)
    report["pass"] = bool(
        report["final"] == "READY"
        and n.blocked is False
        and n.loc == "READY"
        and report["initialpose_count"] >= 1
        and not report["legacy_blind_seed"]
        and report["initialpose_before_map_odom"]
        and report["selected_path"] in ("P1", "P2", "P3")
    )
    dump("t1_cold_nav.json", report)


def run_t2(n) -> None:
    file0 = wait_verified(n, 50.0)
    report: Dict[str, Any] = {"file_before": file0, "snap0": n.snap()}
    if not file0.get("laser_verified"):
        report["pass"] = False
        report["reason"] = "no_laser_verified_last_good"
        dump("t2_unmoved_p2.json", report)
        return
    report.update(run_boot(n, "t2_unmoved_p2", 180.0))
    report["pass"] = bool(
        file0.get("laser_verified")
        and report.get("selected_path") == "P2"
        and report.get("final") == "READY"
        and n.blocked is False
        and not report.get("p3_ran")
        and report.get("initialpose_count") == 1
        and not report.get("legacy_blind_seed")
    )
    dump("t2_unmoved_p2.json", report)


def run_t3(n) -> None:
    old = C.read_pose_file()
    report = run_boot(n, "t3_relocated_p3", 220.0)
    report["old_last_good"] = {k: old.get(k) for k in ("x", "y", "yaw", "laser_verified", "laser_score_at_write")}
    ready = report.get("final") == "READY" and n.loc == "READY" and n.blocked is False
    unknown = n.loc == "NEED_OPERATOR" and n.blocked is True
    p2_ready = report.get("selected_path") == "P2" and report.get("final") == "READY"
    report["p2_false_accept"] = bool(p2_ready)
    report["outcome"] = "READY" if ready else ("SAFE_UNKNOWN" if unknown else "FAIL")
    report["pass"] = bool(
        (not p2_ready)
        and report.get("p3_ran")
        and (ready or unknown)
        and not report.get("legacy_blind_seed")
    )
    dump("t3_relocated_p3.json", report)


def run_t4(n) -> None:
    n.spin(1.2)
    report: Dict[str, Any] = {"power": n.power, "snap0": n.snap()}
    if not (n.power.get("charging") or n.power.get("docked")):
        report["pass"] = False
        report["reason"] = "not_charging_no_dock_evidence"
        report["note"] = "robot not on dock; charger blind seed not used; path stopped"
        dump("t4_charger_p1.json", report)
        return
    report.update(run_boot(n, "t4_charger_p1", 180.0))
    report["pass"] = bool(
        report.get("selected_path") == "P1"
        and report.get("final") == "READY"
        and n.blocked is False
        and not report.get("legacy_blind_seed")
    )
    dump("t4_charger_p1.json", report)


def run_t5(n) -> None:
    report: Dict[str, Any] = {"snap0": n.snap()}
    n.wait_until(lambda: n.loc == "READY" and n.blocked is False and n.amcl is not None, 15.0)
    if not (n.loc == "READY" and n.blocked is False and n.amcl):
        report["pass"] = False
        report["reason"] = "not_ready_before_nav"
        dump("t5_lost_nav.json", report)
        return
    report.update(induce_lost(n, True))
    resume = report.get("resume_policy") or {}
    snap = (report.get("lost_result") or {}).get("snapshot") or {}
    report["pass"] = bool(
        report.get("final") == "READY"
        and report.get("selected_recovery_path") in ("R1", "R2")
        and report.get("loc") == "READY"
        and report.get("blocked") is False
        and not report.get("owners", {}).get("legacy_seen")
        and int(report.get("seed_count") or 0) == 1
        and resume.get("nav") in ("replan_published", "no_goal_saved")
        and report.get("r3_code") in ("", None)
    )
    if snap.get("nav_goal") and resume.get("nav") != "replan_published":
        report["pass"] = False
    dump("t5_lost_nav.json", report)


def run_t6(n) -> None:
    report = induce_lost(n, False)
    old_rejected = (
        report.get("r1_code") == "R1_CURRENT_REJECT"
        and report.get("r2_code") == "R2_LAST_GOOD_REJECT"
        and not report.get("r1_seeded")
        and not report.get("r2_seeded")
    )
    ready = report.get("final") == "READY" and report.get("selected_recovery_path") == "R3"
    unknown = report.get("final") == "UNKNOWN" and report.get("loc") == "NEED_OPERATOR"
    report["old_rejected"] = old_rejected
    report["outcome"] = "READY" if ready else ("SAFE_UNKNOWN" if unknown else "FAIL")
    report["pass"] = bool(old_rejected and (ready or unknown) and not report.get("owners", {}).get("legacy_seen"))
    if ready:
        report["pass"] = bool(report["pass"] and report.get("r3_code") == "R3_VISUAL_ACCEPT" and int(report.get("seed_count") or 0) == 1)
    if unknown:
        resume = report.get("resume_policy") or {}
        report["pass"] = bool(report["pass"] and report.get("blocked") is True and resume.get("nav") == "forbidden" and report.get("motion_count") == 0)
    dump("t6_manual_carry.json", report)


def run_t7(n) -> None:
    report = induce_lost(n, False)
    resume = report.get("resume_policy") or {}
    report["pass"] = bool(
        report.get("final") == "UNKNOWN"
        and report.get("r3_code") == "R3_VISUAL_UNKNOWN"
        and report.get("r1_code") in ("R1_CURRENT_REJECT", "R1_CURRENT_SKIP")
        and report.get("r2_code") in ("R2_LAST_GOOD_REJECT", "R2_LAST_GOOD_SKIP")
        and not report.get("r1_seeded")
        and not report.get("r2_seeded")
        and not report.get("r3_seeded")
        and int(report.get("seed_count") or 0) == 0
        and report.get("loc") == "NEED_OPERATOR"
        and report.get("blocked") is True
        and resume.get("nav") == "forbidden"
        and resume.get("follow") == "forbidden"
        and resume.get("recharge") == "forbidden"
        and report.get("motion_count") == 0
        and report.get("follow_en") is not True
        and report.get("recharge_en") is not True
        and not report.get("owners", {}).get("legacy_seen")
    )
    dump("t7_hard_negative.json", report)


def run_operator(n, x: float, y: float, yaw: float, name: str, expect_ready: bool) -> None:
    report: Dict[str, Any] = {"pose": {"x": x, "y": y, "yaw": yaw}, "expect_ready": expect_ready, "snap0": n.snap()}
    n.wait_until(lambda: n.loc == "NEED_OPERATOR", 8.0)
    if n.loc != "NEED_OPERATOR":
        report.update({"pass": False, "reason": "not_in_need_operator", "loc": n.loc})
        dump(name, report)
        return
    web = C.web_initialpose(x, y, yaw)
    saw = False
    t0 = time.monotonic()
    while time.monotonic() - t0 < 35.0:
        n.spin(0.25)
        if n.boot.get("state") == "VERIFYING_OPERATOR_POSE" or n.loc == "VERIFYING_OPERATOR_POSE":
            saw = True
        if expect_ready and n.loc == "READY" and n.blocked is False:
            break
        if (not expect_ready) and time.monotonic() - t0 > 18.0 and n.loc != "READY":
            break
    report.update({
        "web": web,
        "saw_verifying": saw,
        "loc": n.loc,
        "blocked": n.blocked,
        "owners": owners(n.owners),
        "snap_end": n.snap(),
    })
    if expect_ready:
        report["pass"] = bool(n.loc == "READY" and n.blocked is False and saw and any(o == "operator" for o in report["owners"]["owners"]))
    else:
        report["pass"] = bool(n.loc != "READY")
    dump(name, report)


def run_cpu(n, window: float) -> None:
    spec = importlib.util.spec_from_file_location("c4b2_cpu", Path("/ros2_ws/src/xw_phase2c/scripts/phase2c_c4b2_proc_cpu.py"))
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    sample = mod.sample(window)
    sample["snap"] = n.snap()
    sample["pass"] = all(v is None or v < 20.0 for v in (sample.get("by_key") or {}).values())
    dump("cpu_idle.json", sample)


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--phase", required=True, choices=[
        "snapshot", "t1", "t2", "t3", "t4", "t5", "t6", "t7", "operator", "operator_bad", "cpu",
    ])
    ap.add_argument("--x", type=float, default=0.0)
    ap.add_argument("--y", type=float, default=0.0)
    ap.add_argument("--yaw", type=float, default=0.0)
    ap.add_argument("--window", type=float, default=45.0)
    args = ap.parse_args()
    OUT.mkdir(parents=True, exist_ok=True)
    rclpy.init()
    n = C.Watch()
    try:
        n.spin(1.2)
        if args.phase == "snapshot":
            dump("snapshot.json", {"snap": n.snap(), "last_good": C.read_pose_file()})
        elif args.phase == "t1":
            run_t1(n)
        elif args.phase == "t2":
            run_t2(n)
        elif args.phase == "t3":
            run_t3(n)
        elif args.phase == "t4":
            run_t4(n)
        elif args.phase == "t5":
            run_t5(n)
        elif args.phase == "t6":
            run_t6(n)
        elif args.phase == "t7":
            run_t7(n)
        elif args.phase == "operator":
            run_operator(n, args.x, args.y, args.yaw, "operator_good.json", True)
        elif args.phase == "operator_bad":
            run_operator(n, args.x, args.y, args.yaw, "operator_bad.json", False)
        elif args.phase == "cpu":
            run_cpu(n, args.window)
    finally:
        n.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
