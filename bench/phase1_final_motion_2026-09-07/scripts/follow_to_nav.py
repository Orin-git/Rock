#!/usr/bin/env python3
"""Send /xw/goal_pose to a named vp waypoint and wait for arrival."""
from __future__ import annotations

import math
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

WPS = {
    "charger": (1.8663955491712294, -0.05958837147746455, -3.1286646850836126),
    "wp_9": (0.014664885102192216, -0.5797687003570173, 1.5707963267948966),
    "wp_2": (-4.0779483147098965, -0.9390276581991088, 3.3018138790728147),
}


class NavOnce(Node):
    def __init__(self, name: str) -> None:
        super().__init__("t2_follow_to_nav")
        x, y, yaw = WPS[name]
        self.goal = (x, y, yaw)
        self.pose = None
        self.cmds = []
        self.pub = self.create_publisher(PoseStamped, "/xw/goal_pose", 10)
        qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(PoseWithCovarianceStamped, "/amcl_pose", self._on_pose, qos)
        self.create_subscription(Twist, "/cmd_vel", self._on_cmd, 10)

    def _on_pose(self, msg: PoseWithCovarianceStamped) -> None:
        self.pose = msg

    def _on_cmd(self, msg: Twist) -> None:
        self.cmds.append((msg.linear.x, msg.angular.z))


def main() -> None:
    name = sys.argv[1] if len(sys.argv) > 1 else "charger"
    out = sys.argv[2] if len(sys.argv) > 2 else "/tmp/follow_to_nav.txt"
    rclpy.init()
    n = NavOnce(name)
    x, y, yaw = n.goal
    msg = PoseStamped()
    msg.header.frame_id = "map"
    msg.pose.position.x = x
    msg.pose.position.y = y
    msg.pose.orientation.z = math.sin(yaw / 2.0)
    msg.pose.orientation.w = math.cos(yaw / 2.0)
    for _ in range(8):
        msg.header.stamp = n.get_clock().now().to_msg()
        n.pub.publish(msg)
        rclpy.spin_once(n, timeout_sec=0.05)
        time.sleep(0.15)
    print("goal_published", name, x, y, yaw)
    t0 = time.time()
    moved = False
    while time.time() - t0 < 20:
        rclpy.spin_once(n, timeout_sec=0.1)
        if n.cmds and (abs(n.cmds[-1][0]) > 0.05 or abs(n.cmds[-1][1]) > 0.05):
            moved = True
            break
    print("nav_motion_seen", moved, "cmd_samples", len(n.cmds))
    ok = False
    final = (None, None)
    while time.time() - t0 < 150:
        rclpy.spin_once(n, timeout_sec=0.1)
        if n.pose is None:
            continue
        px = n.pose.pose.pose.position.x
        py = n.pose.pose.pose.position.y
        final = (px, py)
        d = math.hypot(px - x, py - y)
        if d < 0.5:
            ok = True
            print("ARRIVED d=%.3f t=%.1f" % (d, time.time() - t0))
            break
    line = "goal=%s nav_motion_seen=%s nav_success=%s final_xy=%s\n" % (
        name,
        moved,
        ok,
        final,
    )
    open(out, "w").write(line)
    print(line.strip())
    n.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
