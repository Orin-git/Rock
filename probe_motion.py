#!/usr/bin/env python3
"""Is the robot actually moving? Sample odom twist + cmd_vel for 6 s."""
import math, sys, threading, time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

SAMPLES = []

class P(Node):
    def __init__(self):
        super().__init__('probe_motion')
        q = QoSProfile(depth=10)
        q.reliability = ReliabilityPolicy.BEST_EFFORT
        q.durability = DurabilityPolicy.VOLATILE
        self.create_subscription(Odometry, '/odom', self.on_odom, q)
        self.create_subscription(Twist, '/cmd_vel', self.on_cmd, 10)
        self.cmd = None
        self.cmd_t = 0.0

    def on_odom(self, m):
        t = m.twist.twist
        SAMPLES.append((time.time(), math.hypot(t.linear.x, t.linear.y), t.angular.z))

    def on_cmd(self, m):
        self.cmd = m
        self.cmd_t = time.time()

def main():
    rclpy.init()
    n = P()
    t0 = time.time()
    while time.time() - t0 < 6.0:
        rclpy.spin_once(n, timeout_sec=0.2)
    print(f"odom samples = {len(SAMPLES)} over 6.0 s")
    if SAMPLES:
        v = [s[1] for s in SAMPLES]
        w = [abs(s[2]) for s in SAMPLES]
        moving = [x for x in v if x > 0.02]
        print(f"  linear:  max={max(v):.4f}  mean={sum(v)/len(v):.4f}  samples>0.02 = {len(moving)}/{len(v)}")
        print(f"  angular: max={max(w):.4f}  mean={sum(w)/len(w):.4f}")
        print(f"  LAST linear={v[-1]:.4f} angular={w[-1]:.4f}  (age {time.time()-SAMPLES[-1][0]:.2f}s)")
    if n.cmd is not None:
        c = n.cmd
        print(f"cmd_vel: linear.x={c.linear.x:.4f} angular.z={c.angular.z:.4f} (age {time.time()-n.cmd_t:.2f}s)")
    else:
        print("cmd_vel: NO MESSAGE in 6 s")
    n.destroy_node(); rclpy.shutdown()

main()
