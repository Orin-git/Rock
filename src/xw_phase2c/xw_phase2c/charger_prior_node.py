#!/usr/bin/env python3
"""Publish charger soft-prior evaluation. Never publishes /initialpose."""

from __future__ import annotations

import json
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Bool, String

from xw_interfaces.msg import PowerState
from xw_phase2c.charger_prior import (
    evaluate_charger_soft_prior,
    verify_charger_with_laser,
)


_LATCH = QoSProfile(
    depth=1,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    reliability=ReliabilityPolicy.RELIABLE,
)


class ChargerPriorNode(Node):
    def __init__(self) -> None:
        super().__init__('xw_charger_prior')
        self.declare_parameter('maps_dir', '/ros2_ws/maps')
        self.declare_parameter('publish_hz', 1.0)

        self._power = PowerState()
        self._battery_charging = False
        self._map_name = ''

        self._avail_pub = self.create_publisher(Bool, '/xw/localization/charger_prior_available', _LATCH)
        self._diag_pub = self.create_publisher(String, '/xw/localization/charger_prior_diag', _LATCH)

        self.create_subscription(PowerState, '/xw/power', self._on_power, 10)
        self.create_subscription(BatteryState, '/battery_state', self._on_battery, 10)
        self.create_subscription(String, '/xw/nav/map_name', self._on_map, _LATCH)

        hz = max(0.2, float(self.get_parameter('publish_hz').value))
        self.create_timer(1.0 / hz, self._tick)
        self.get_logger().info(
            'charger prior helper ready (SOFT PRIOR only; no /initialpose; '
            'verify_charger_with_laser stub reserved)'
        )
        # Touch reserved API so import/link is exercised in C1.
        _ = verify_charger_with_laser((0.0, 0.0, 0.0))

    def _on_power(self, msg: PowerState) -> None:
        self._power = msg

    def _on_battery(self, msg: BatteryState) -> None:
        self._battery_charging = (
            msg.power_supply_status == BatteryState.POWER_SUPPLY_STATUS_CHARGING
        )

    def _on_map(self, msg: String) -> None:
        self._map_name = (msg.data or '').strip()

    def _tick(self) -> None:
        name = self._map_name or 'vp'
        res = evaluate_charger_soft_prior(
            charging=bool(self._power.charging),
            docked=bool(self._power.docked),
            battery_charging=bool(self._battery_charging),
            maps_dir=str(self.get_parameter('maps_dir').value),
            map_name=name,
        )
        self._avail_pub.publish(Bool(data=bool(res.charger_prior_available)))
        self._diag_pub.publish(String(data=json.dumps(res.as_dict(), separators=(',', ':'))))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ChargerPriorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
