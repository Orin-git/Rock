#!/usr/bin/env python3
"""Avoidance analysis monitor: sensor usage, CM interventions, speed profile."""
import math
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan, PointCloud2
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

try:
    from nav2_msgs.msg import CollisionMonitorState
    _HAS_CM = True
except Exception:  # noqa: BLE001
    _HAS_CM = False


class Mon(Node):
    def __init__(self):
        _BE = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        super().__init__('avoid_monitor')
        self.t0 = time.time()
        self.scan = {'n': 0, 'last': 0.0}
        self.d1 = {'n': 0, 'width': 0, 'densed': 0, 'last': 0.0}
        self.d2 = {'n': 0, 'width': 0, 'densed': 0, 'last': 0.0}
        self.vel = Twist()
        self.odom = None
        self.cm = None
        self.create_subscription(LaserScan, '/scan', self.cb_scan, 10)
        self.create_subscription(
            PointCloud2, '/camera/front_up/depth/points_nav', self.cb_d1, 10)
        self.create_subscription(
            PointCloud2, '/camera/front_down/depth/points_nav', self.cb_d2, 10)
        self.create_subscription(Twist, '/xw/cmd/nav', self.cb_nav, 10)
        self.create_subscription(Twist, '/cmd_vel_nav', self.cb_cvel, 10)
        self.create_subscription(Odometry, '/odom', self.cb_odom, 10)
        if _HAS_CM:
            self.create_subscription(
                CollisionMonitorState, '/collision_monitor_state', self.cb_cm, 10)
        self.create_timer(1.0, self.tick)
        self.get_logger().info('=== avoid_monitor started ===')

    def cb_scan(self, m):
        self.scan['n'] += 1
        self.scan['last'] = time.time()

    def cb_d1(self, m):
        self.d1['n'] += 1
        self.d1['width'] = m.width * m.height

    def cb_d2(self, m):
        self.d2['n'] += 1
        self.d2['width'] = m.width * m.height

    def cb_nav(self, m):
        self.vel = m

    def cb_cvel(self, m):
        self.vel = m

    def cb_cm(self, m):
        self.cm = m

    def cb_odom(self, m):
        self.odom = (m.pose.pose.position.x, m.pose.pose.position.y)

    def tick(self):
        now = time.time()
        rt = now - self.t0
        sp = math.sqrt(
            self.vel.linear.x ** 2 + self.vel.linear.y ** 2 + self.vel.angular.z ** 2)
        cmtxt = ''
        if self.cm is not None and _HAS_CM:
            cmtxt = f' cm={list(self.cm.actions)}'
        self.get_logger().info(
            f'[{rt:5.0f}s] scan={self.scan["n"]:2d} | d1={self.d1["n"]:2d}@{self.d1["width"]:4d}pts'
            f' | d2={self.d2["n"]:2d}@{self.d2["width"]:4d}pts'
            f' | vel=({self.vel.linear.x:+.3f},{self.vel.angular.z:+.3f}) sp={sp:.2f}'
            f' | odom={self.odom}{cmtxt}')
        self.scan['n'] = 0
        self.d1['n'] = 0
        self.d2['n'] = 0


def main():
    rclpy.init()
    node = Mon()
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.1)


main()
