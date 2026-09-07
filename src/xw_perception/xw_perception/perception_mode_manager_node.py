#!/usr/bin/env python3
"""Phase1 Perception Mode Manager — profiles for camera/NPU budget.

Profiles: IDLE | NAVIGATION | FOLLOW | FALL_DETECTION | RECHARGE

Publishes:
  /xw/perception/profile          (latched String)
  /xw/perception/profile_config   (latched String JSON)

Does NOT permanently force fall_enable_default=False. Fall latch remains
orthogonal; NAVIGATION defers RGB so fall-default cannot keep dual RGB hot.
"""

from __future__ import annotations

import json
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String

from xw_interfaces.msg import RobotState


_PROFILES = {
    'IDLE': {
        'rgb_up': False,
        'rgb_down': False,
        'depth_up': False,
        'depth_down': False,
        'preview': False,
        'fall_infer_fps': 0.0,
        'follow_infer_fps': 0.0,
        'points_nav': False,
    },
    'NAVIGATION': {
        'rgb_up': False,
        'rgb_down': False,
        'depth_up': True,
        'depth_down': True,
        'preview': False,
        'fall_infer_fps': 0.0,
        'follow_infer_fps': 0.0,
        'points_nav': True,
    },
    'FOLLOW': {
        'rgb_up': True,
        'rgb_down': False,
        'depth_up': True,
        'depth_down': False,
        'preview': False,
        'fall_infer_fps': 0.0,
        'follow_infer_fps': 10.0,
        'points_nav': True,
    },
    'FALL_DETECTION': {
        'rgb_up': True,
        'rgb_down': True,
        'depth_up': True,
        'depth_down': True,
        'preview': False,
        'fall_infer_fps': 3.5,
        'follow_infer_fps': 0.0,
        'points_nav': False,
    },
    'RECHARGE': {
        'rgb_up': False,
        'rgb_down': False,
        'depth_up': False,
        'depth_down': False,
        'preview': False,
        'fall_infer_fps': 0.0,
        'follow_infer_fps': 0.0,
        'points_nav': False,
    },
}


class PerceptionModeManager(Node):
    def __init__(self) -> None:
        super().__init__('xw_perception_mode_manager')
        self.declare_parameter('allow_fall_rgb_during_nav', False)

        latch = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self._profile_pub = self.create_publisher(String, '/xw/perception/profile', latch)
        self._cfg_pub = self.create_publisher(String, '/xw/perception/profile_config', latch)

        self._follow = False
        self._fall = False
        self._recharge = False
        self._nav = False
        self._mode = 0
        self._profile = ''

        self.create_subscription(Bool, '/xw/follow/enable', self._on_follow, latch)
        self.create_subscription(Bool, '/xw/fall/enable', self._on_fall, latch)
        self.create_subscription(Bool, '/xw/recharge/enable', self._on_recharge, latch)
        self.create_subscription(Bool, '/xw/nav/enable', self._on_nav, latch)
        self.create_subscription(RobotState, '/xw/robot_state', self._on_state, 10)

        self._last_profile_log = 0.0
        self.create_timer(0.5, self._publish_if_needed)
        self._recompute(force=True)
        self.get_logger().info('perception mode manager ready')

    def _on_follow(self, msg: Bool) -> None:
        self._follow = bool(msg.data)
        self._recompute()

    def _on_fall(self, msg: Bool) -> None:
        self._fall = bool(msg.data)
        self._recompute()

    def _on_recharge(self, msg: Bool) -> None:
        self._recharge = bool(msg.data)
        self._recompute()

    def _on_nav(self, msg: Bool) -> None:
        self._nav = bool(msg.data)
        self._recompute()

    def _on_state(self, msg: RobotState) -> None:
        self._mode = int(msg.mode)
        self._recompute()

    def _choose(self) -> str:
        # Priority: follow > recharge > fall-as-primary > navigation > idle
        if self._follow:
            return 'FOLLOW'
        if self._recharge:
            return 'RECHARGE'
        allow_fall_nav = bool(self.get_parameter('allow_fall_rgb_during_nav').value)
        if self._fall:
            if self._mode == 4:
                return 'FALL_DETECTION'
            if self._nav and not allow_fall_nav:
                # Fall latch may stay on for product, but NAV profile defers dual RGB.
                return 'NAVIGATION'
            if not self._nav:
                return 'FALL_DETECTION'
            return 'FALL_DETECTION' if allow_fall_nav else 'NAVIGATION'
        if self._nav or self._mode in (2, 3):
            return 'NAVIGATION'
        return 'IDLE'

    def _recompute(self, force: bool = False) -> None:
        name = self._choose()
        if not force and name == self._profile:
            return
        self._profile = name
        self._publish()

    def _publish_if_needed(self) -> None:
        # Re-latch periodically for late subscribers.
        self._publish()

    def _publish(self) -> None:
        import time

        cfg = dict(_PROFILES.get(self._profile, _PROFILES['IDLE']))
        # FOLLOW may still want fall geometry on same pose infer (fps 0 = use shared).
        if self._profile == 'FOLLOW' and self._fall:
            cfg['fall_infer_fps'] = 0.0  # shared with follow infer on same frame
        msg = String()
        msg.data = self._profile
        self._profile_pub.publish(msg)
        cmsg = String()
        cmsg.data = json.dumps({'profile': self._profile, **cfg}, separators=(',', ':'))
        self._cfg_pub.publish(cmsg)
        now = time.monotonic()
        if now - self._last_profile_log > 5.0:
            self._last_profile_log = now
            self.get_logger().info(
                f'profile → {self._profile} follow={self._follow} fall={self._fall} '
                f'recharge={self._recharge} nav={self._nav} mode={self._mode}'
            )


def main(args: Optional[list] = None) -> None:
    rclpy.init(args=args)
    node = PerceptionModeManager()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
