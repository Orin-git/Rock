#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
p3a_l3_pause_watch.py —— P3A L3 收尾「闭环触发闸」

【只做一件事】订阅 /odom，按三段状态机判出「机器人已经穿过起点」，然后调用
rtabmap 自带的 pause 服务，把新节点创建掐断。

   IDLE ──(距起点 > ARM m)──▶ ARMED        ← 必须先离开，否则起跑时就误触
   ARMED ─(距起点 ≤ NEAR m)─▶ NEAR         ← 机器人正在穿过起点，rtabmap 在提帧
   NEAR ──(距起点 > DEPART m)─▶ FIRED      ← 已经穿过并走远 ⇒ 此刻 pause 才对
   NEAR ──(停留超 HOLD s)────▶ FIRED        ← 兜底：机器人停在起点附近了

── 为什么是「穿过后再 pause」，而不是「到 1.5 m 就 pause」 ──────────
   pause 的语义是【停止处理新帧】。若在机器人距起点 1.5 m 时就 pause，
   它穿过起点那几秒 rtabmap 根本没看见 ⇒ 回环照样提不出来。
   必须把「接近 → 穿过 → 走远」整段帧都喂给它，pause 才落在正确的时刻。

── 为什么需要它（v6 实测，2026-09-29） ──────────────────────────────
   pause 生效             = 02:49:32  （最后一个节点 id144 造于 02:49:31）
   那一刻机器人距起点     = 3.2 m
   机器人走到距起点 0.30 m = 02:49:47  ← 比 pause 晚 15 秒
   ⇒ rtabmap 活着时机器人【从没接近过起点】⇒ 回环根本没机会被提出。
   ⇒ 收尾不能靠「人喊 + 固定延迟」，必须【看着位姿】触发。

── 安全边界（刻意收得很紧） ────────────────────────────────────────
  * 只调【一个】服务：/rtabmap/rtabmap/pause —— 名字写死，不做任何通配匹配
  * 触发时用 `ros2 service type`【现场核对类型】，不是 std_srvs/srv/Empty 就拒绝调用
  * 调用命令与 stop_l3.sh 里已实测成功（exit=0）的那条【逐字相同】
  * 不 kill、不 backup、不 reset、不碰任何其它话题/服务/进程
  * 状态机三段，缺一段都不触发；--dry-run 可全程演练
"""
import argparse
import math
import os
import signal
import subprocess
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry


class Watch(Node):
    def __init__(self, a, logf):
        super().__init__("p3a_l3_pause_watch")
        self.a = a
        self.logf = logf
        self.ref = None
        self.state = "IDLE"
        self.max_dist = 0.0
        self.min_near = None      # 穿过起点期间最近距离（诊断）
        self.t_near = None
        self.fired = False
        self._last_log = 0.0
        qos = QoSProfile(depth=10,
                         reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(Odometry, a.odom_topic, self.cb, qos)
        self.log(f"订阅 {a.odom_topic}（BEST_EFFORT）；"
                 f"arm>{a.arm_dist}  near<={a.near_dist}  depart>{a.depart_dist}  hold={a.hold_timeout}s"
                 f"{'  [DRY-RUN]' if a.dry_run else ''}")

    def log(self, s):
        line = f"[{time.strftime('%H:%M:%S')}] {s}"
        print(line, flush=True)
        try:
            with open(self.logf, "a") as f:
                f.write(line + "\n")
        except Exception:
            pass

    def cb(self, msg):
        if self.fired:
            return
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        if self.ref is None:
            self.ref = (x, y)
            self.log(f"★ 起点参考位姿已锁定  ref=({x:+.3f}, {y:+.3f})"
                     f"   —— 必须【先离开 {self.a.arm_dist} m】才会武装")

        d = math.hypot(x - self.ref[0], y - self.ref[1])
        self.max_dist = max(self.max_dist, d)

        # ── ① IDLE → ARMED
        if self.state == "IDLE" and d > self.a.arm_dist:
            self.state = "ARMED"
            self.log(f"▲ ARMED（距起点 {d:.2f} m > {self.a.arm_dist} m）")

        # ── ② ARMED → NEAR
        elif self.state == "ARMED" and d <= self.a.near_dist:
            self.state = "NEAR"
            self.t_near = time.time()
            self.min_near = d
            self.log(f"◆ NEAR（距起点 {d:.2f} m ≤ {self.a.near_dist} m）"
                     f" —— 机器人正在穿过起点，rtabmap 正在提帧")

        # ── ③ NEAR → FIRED
        elif self.state == "NEAR":
            self.min_near = min(self.min_near, d)
            held = time.time() - self.t_near
            if d > self.a.depart_dist:
                self.fire(f"已穿过并走远（距起点 {d:.2f} m > {self.a.depart_dist} m，"
                          f"穿过期间最近 {self.min_near:.2f} m）")
            elif held > self.a.hold_timeout:
                self.fire(f"在起点附近停留 {held:.0f} s 仍未走远（当前 {d:.2f} m，"
                          f"最近 {self.min_near:.2f} m）—— 兜底触发")

        # ── 节流打印
        now = time.time()
        if not self.fired and now - self._last_log >= 5.0:
            self._last_log = now
            self.log(f"  距起点 {d:6.2f} m  最远 {self.max_dist:6.2f} m  "
                     f"[{self.state}]  ({x:+.2f},{y:+.2f})")

        if self.fired:
            raise SystemExit(0)

    def fire(self, why):
        self.fired = True
        self.log(f"★★★ 触发 → pause  |  {why}")
        a = self.a
        if a.dry_run:
            self.log("[DRY-RUN] 本应调用 pause —— 不执行任何动作")
            return
        try:
            t = subprocess.run(["ros2", "service", "type", a.service],
                               capture_output=True, text=True, timeout=30)
            got = (t.stdout or "").strip().splitlines()[0].strip() if t.stdout.strip() else ""
        except Exception as e:
            self.log(f"✗ 取服务类型失败（{e}）—— 拒绝调用")
            return
        if got != a.service_type:
            self.log(f"✗ 类型不符：期望 {a.service_type}，实得「{got}」—— 拒绝调用")
            return
        self.log(f"   类型核对通过（{got}），调用 pause …")
        try:
            r = subprocess.run(["ros2", "service", "call", a.service, a.service_type, "{}"],
                               capture_output=True, text=True, timeout=60)
            tail = " ".join((r.stdout or "").strip().splitlines()[-2:])
            self.log(f"   pause exit={r.returncode}  输出: {tail[:160]}")
        except Exception as e:
            self.log(f"✗ pause 调用异常（{e}）—— 未生效，stop_l3.sh 仍会兜底")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--arm-dist",     type=float, default=5.0)
    ap.add_argument("--near-dist",    type=float, default=1.5)
    ap.add_argument("--depart-dist",  type=float, default=3.0)
    ap.add_argument("--hold-timeout", type=float, default=30.0)
    ap.add_argument("--timeout",      type=float, default=1800.0)
    ap.add_argument("--dry-run",      action="store_true")
    ap.add_argument("--log",          default="/tmp/p3a_l3_pause_watch.log")
    ap.add_argument("--odom-topic",   default="/odom")
    ap.add_argument("--service",      default="/rtabmap/rtabmap/pause")
    ap.add_argument("--service-type", default="std_srvs/srv/Empty")
    a = ap.parse_args()

    logf = open(a.log, "a")
    def L(s):
        line = f"[{time.strftime('%H:%M:%S')}] {s}"
        print(line, flush=True); logf.write(line + "\n"); logf.flush()
    L(f"=== 闭环触发闸启动  pid={os.getpid()}  日志={a.log} ===")

    rclpy.init()
    n = Watch(a, a.log)

    def on_term(signum, frame):
        n.log(f"收到信号 {signum} —— 退出；state={n.state} 最远 {n.max_dist:.2f} m"
              + (f"  穿过期间最近 {n.min_near:.2f} m" if n.min_near is not None else ""))
        raise SystemExit(0)
    signal.signal(signal.SIGTERM, on_term)
    signal.signal(signal.SIGINT, on_term)

    t0 = time.time()
    while rclpy.ok() and time.time() - t0 < a.timeout and not n.fired:
        try:
            rclpy.spin_once(n, timeout_sec=0.2)
        except SystemExit:
            break

    if not n.fired:
        n.log(f"⏱ 超时/退出，未触发。state={n.state} 最远 {n.max_dist:.2f} m"
              + (f"  穿过期间最近 {n.min_near:.2f} m" if n.min_near is not None else "  (从未到过起点附近)"))
    logf.close()


if __name__ == "__main__":
    main()
