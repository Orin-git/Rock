#!/usr/bin/env python3
"""Task1 — verify /xw/reloc/rgb_request gate without heavy topic hz."""

from __future__ import annotations

import json
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Bool


_SENSOR = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=2,
)
_LATCH = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)


class GateProbe(Node):
    def __init__(self) -> None:
        super().__init__('rgb_gate_probe')
        self.pub_count = {'up_pub': 0, 'down_pub': 0, 'vendor_up': 0}
        self.create_subscription(
            Image, '/camera/front_up/color/image_raw',
            lambda m: self._bump('up_pub'), _SENSOR,
        )
        self.create_subscription(
            Image, '/camera/front_down/color/image_raw',
            lambda m: self._bump('down_pub'), _SENSOR,
        )
        self.create_subscription(
            Image, '/ascamera_hp60c/camera_publisher/rgb0/image',
            lambda m: self._bump('vendor_up'), _SENSOR,
        )
        self.req = self.create_publisher(Bool, '/xw/reloc/rgb_request', _LATCH)

    def _bump(self, k: str) -> None:
        self.pub_count[k] += 1

    def sample(self, sec: float) -> dict:
        before = dict(self.pub_count)
        t0 = time.monotonic()
        while time.monotonic() - t0 < sec:
            rclpy.spin_once(self, timeout_sec=0.05)
        after = dict(self.pub_count)
        return {k: after[k] - before[k] for k in before}


def main() -> None:
    rclpy.init()
    n = GateProbe()
    # ensure OFF
    n.req.publish(Bool(data=False))
    time.sleep(1.0)
    off = n.sample(3.0)
    n.req.publish(Bool(data=True))
    time.sleep(1.5)
    on = n.sample(3.0)
    n.req.publish(Bool(data=False))
    time.sleep(1.5)
    off2 = n.sample(3.0)
    report = {
        'nav_like_off': off,
        'reloc_active_on': on,
        'reloc_end_off': off2,
        'pass_criteria': {
            'off_public_up_near_zero': off['up_pub'] == 0,
            'on_public_up_positive': on['up_pub'] > 0,
            'off2_public_up_near_zero': off2['up_pub'] == 0,
            'down_never_on': off['down_pub'] == 0 and on['down_pub'] == 0 and off2['down_pub'] == 0,
        },
    }
    report['gate_pass'] = all(report['pass_criteria'].values())
    out = Path('/ros2_ws/bench/phase2a_poc_v1_2026-09-07/task1_rgb_gate.json')
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(report, indent=2), encoding='utf-8')
    print(json.dumps(report, indent=2))
    n.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
