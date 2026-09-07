#!/usr/bin/env python3
"""Record amcl pose at 0/1/2/5/10s after Follow exit; compute jump vs t0."""
from __future__ import annotations

import math
import sys
import time

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import Int8


def yaw_of(msg: PoseWithCovarianceStamped) -> float:
    q = msg.pose.pose.orientation
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


def ang_diff(a: float, b: float) -> float:
    d = a - b
    while d > math.pi:
        d -= 2 * math.pi
    while d < -math.pi:
        d += 2 * math.pi
    return d


class Cap(Node):
    def __init__(self) -> None:
        super().__init__("exit_jump_cap")
        self.pose = None  # type: PoseWithCovarianceStamped | None
        self.loc = -1
        qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(PoseWithCovarianceStamped, "/amcl_pose", self._p, qos)
        self.create_subscription(Int8, "/xw/localization_status", self._l, 10)

    def _p(self, msg: PoseWithCovarianceStamped) -> None:
        self.pose = msg

    def _l(self, msg: Int8) -> None:
        self.loc = int(msg.data)

    def wait_pose(self, timeout: float = 2.0):
        t0 = time.time()
        while time.time() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.pose is not None:
                return self.pose
        return self.pose


def main() -> None:
    out = sys.argv[1] if len(sys.argv) > 1 else "/tmp/exit_jump.txt"
    times = [0.0, 1.0, 2.0, 5.0, 10.0]
    rclpy.init()
    n = Cap()
    # prime
    for _ in range(50):
        rclpy.spin_once(n, timeout_sec=0.05)
    t_start = time.time()
    samples = []
    for t in times:
        while time.time() - t_start < t:
            rclpy.spin_once(n, timeout_sec=0.05)
        p = n.wait_pose(1.5)
        if p is None:
            samples.append((t, None))
            continue
        samples.append((t, p, n.loc))
        print(
            f"t={t:.0f}s loc={n.loc} x={p.pose.pose.position.x:.4f} "
            f"y={p.pose.pose.position.y:.4f} yaw={yaw_of(p):.4f}"
        )

    lines = ["t_s,loc,x,y,yaw,dpos_m,dyaw_deg"]
    if samples and samples[0][1] is not None:
        p0 = samples[0][1]
        x0, y0, yaw0 = p0.pose.pose.position.x, p0.pose.pose.position.y, yaw_of(p0)
        for item in samples:
            t = item[0]
            if item[1] is None:
                lines.append(f"{t},,,,,,,")
                continue
            p, loc = item[1], item[2]
            x, y, yaw = p.pose.pose.position.x, p.pose.pose.position.y, yaw_of(p)
            dpos = math.hypot(x - x0, y - y0)
            dyaw = abs(ang_diff(yaw, yaw0)) * 180.0 / math.pi
            lines.append(f"{t},{loc},{x:.4f},{y:.4f},{yaw:.4f},{dpos:.4f},{dyaw:.3f}")
        # summary from last sample vs t0
        last = samples[-1]
        if last[1] is not None:
            p = last[1]
            dpos = math.hypot(p.pose.pose.position.x - x0, p.pose.pose.position.y - y0)
            dyaw = abs(ang_diff(yaw_of(p), yaw0)) * 180.0 / math.pi
            lines.append(f"# ExitPoseJump_vs_t0_at_10s position_m={dpos:.4f} yaw_deg={dyaw:.3f}")
            print(f"ExitPoseJump position={dpos:.4f}m yaw={dyaw:.3f}deg")
    open(out, "w").write("\n".join(lines) + "\n")
    n.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
