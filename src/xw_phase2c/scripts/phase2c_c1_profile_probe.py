#!/usr/bin/env python3
"""Phase2C-C1 live-ish ROS checks (optional). Uses transient nodes; no Reloc; no bringup edit.

Requires sourced ROS2 + built packages. Safe with phase2c_lost_cancel default OFF —
this script publishes phase2c_recovery directly to validate profile switching.
"""

from __future__ import annotations

import json
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String


_LATCH = QoSProfile(
    depth=1,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    reliability=ReliabilityPolicy.RELIABLE,
)


class Probe(Node):
    def __init__(self) -> None:
        super().__init__('phase2c_c1_probe')
        self.profile = ''
        self.cfg = {}
        self.create_subscription(String, '/xw/perception/profile', self._on_p, _LATCH)
        self.create_subscription(String, '/xw/perception/profile_config', self._on_c, _LATCH)
        self.rec_pub = self.create_publisher(Bool, '/xw/localization/phase2c_recovery', _LATCH)
        self.block_pub = self.create_publisher(Bool, '/xw/nav/goals_blocked', _LATCH)

    def _on_p(self, msg: String) -> None:
        self.profile = msg.data

    def _on_c(self, msg: String) -> None:
        try:
            self.cfg = json.loads(msg.data or '{}')
        except json.JSONDecodeError:
            self.cfg = {}


def main() -> int:
    rclpy.init()
    n = Probe()
    # Enter recovery profile
    n.rec_pub.publish(Bool(data=True))
    t0 = time.time()
    ok_enter = False
    while time.time() - t0 < 5.0:
        rclpy.spin_once(n, timeout_sec=0.1)
        if n.profile == 'LOCALIZATION_RECOVERY':
            cfg = n.cfg
            if (
                cfg.get('rgb_up') is True
                and cfg.get('rgb_down') is False
                and cfg.get('depth_up') is False
                and cfg.get('points_nav') is False
                and float(cfg.get('fall_infer_fps') or 0) == 0.0
            ):
                ok_enter = True
                break
    print(f'profile_enter={ok_enter} profile={n.profile} cfg={n.cfg}', flush=True)

    n.rec_pub.publish(Bool(data=False))
    t0 = time.time()
    ok_exit = False
    while time.time() - t0 < 5.0:
        rclpy.spin_once(n, timeout_sec=0.1)
        if n.profile and n.profile != 'LOCALIZATION_RECOVERY':
            ok_exit = True
            break
    print(f'profile_exit={ok_exit} profile={n.profile}', flush=True)

    # goals_blocked publish smoke (nav_session may or may not be up)
    n.block_pub.publish(Bool(data=True))
    time.sleep(0.2)
    n.block_pub.publish(Bool(data=False))
    print('goals_blocked_pub=ok', flush=True)

    n.destroy_node()
    rclpy.shutdown()
    return 0 if ok_enter and ok_exit else 1


if __name__ == '__main__':
    sys.exit(main())
