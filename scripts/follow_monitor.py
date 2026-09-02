#!/usr/bin/env python3
"""Follow-session live monitor: enable latch, tracks, motion cmds, nav state."""
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from xw_interfaces.msg import PersonTracks
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

class Mon(Node):
    def __init__(self):
        super().__init__('follow_monitor')
        self.en = None
        self.last_track = 0.0
        self.last_cmd = 0.0
        self.t0 = time.time()
        self.create_subscription(Bool, '/xw/follow/enable', self.on_en, 10)
        self.create_subscription(
            PersonTracks, '/xw/perception/tracks', self.on_tracks, 10)
        self.create_subscription(Twist, '/xw/cmd/follow', self.on_follow_cmd, 10)
        self.create_subscription(Twist, '/cmd_vel_nav', self.on_nav_cmd, 10)
        self.get_logger().info('=== follow monitor started (t0 + seconds) ===')

    def on_en(self, m):
        self.en = bool(m.data)
        self.log(f'FOLLOW_ENABLE -> {self.en}')

    def on_tracks(self, m):
        now = time.time()
        if now - self.last_track < 0.8:
            return
        self.last_track = now
        txt = []
        for t in m.tracks:
            txt.append(
                f'id={t.track_id} xyz=({t.x:.2f},{t.y:.2f},{t.z:.2f}) d={t.distance:.2f} c={t.confidence:.2f} '
                f'primary={t.is_primary} target={t.is_target}')
        self.log(f'TRACKS n={len(m.tracks)} [{" | ".join(txt) or "empty"}]')

    def on_follow_cmd(self, m):
        sp = (m.linear.x ** 2 + m.linear.y ** 2 + m.angular.z ** 2) ** 0.5
        if sp > 0.02 and time.time() - self.last_cmd > 0.8:
            self.last_cmd = time.time()
            self.log(f'FOLLOW_CMD lin.x={m.linear.x:.3f} ang.z={m.angular.z:.3f}')

    def on_nav_cmd(self, m):
        sp = (m.linear.x ** 2 + m.linear.y ** 2 + m.angular.z ** 2) ** 0.5
        if sp > 0.02 and time.time() - self.last_cmd > 0.8:
            self.last_cmd = time.time()
            self.log(f'NAV_CMD lin.x={m.linear.x:.3f} ang.z={m.angular.z:.3f}')

    def log(self, s):
        self.get_logger().info(f'[{time.time() - self.t0:7.1f}s] {s}')

def main():
    rclpy.init()
    node = Mon()
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.2)

main()
