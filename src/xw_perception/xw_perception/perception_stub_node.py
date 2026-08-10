#!/usr/bin/env python3
"""Perception stub: empty tracks + no-fall by default."""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

from xw_interfaces.msg import FallStatus, PersonTrack, PersonTracks


class PerceptionStubNode(Node):
    def __init__(self) -> None:
        super().__init__('xw_perception_stub')
        self.declare_parameter('simulate_person', False)
        self.declare_parameter('simulate_fall', False)
        self._follow_en = False
        self._fall_en = False
        self._tracks_pub = self.create_publisher(PersonTracks, '/xw/perception/tracks', 10)
        self._fall_pub = self.create_publisher(FallStatus, '/xw/perception/fall', 10)
        self.create_subscription(Bool, '/xw/follow/enable', self._on_follow_en, 10)
        self.create_subscription(Bool, '/xw/fall/enable', self._on_fall_en, 10)
        self.create_timer(0.2, self._tick)
        self.get_logger().info('perception stub ready')

    def _on_follow_en(self, msg: Bool) -> None:
        self._follow_en = bool(msg.data)

    def _on_fall_en(self, msg: Bool) -> None:
        self._fall_en = bool(msg.data)

    def _tick(self) -> None:
        if self._follow_en:
            t = PersonTracks()
            t.stamp = self.get_clock().now().to_msg()
            t.frame_id = 'camera_front_link'
            if self.get_parameter('simulate_person').value:
                p = PersonTrack()
                p.track_id = 1
                p.x = 0.0
                p.y = 0.0
                p.z = 1.5
                p.distance = 1.5
                p.confidence = 0.9
                p.is_primary = True
                t.tracks = [p]
            self._tracks_pub.publish(t)

        if self._fall_en:
            f = FallStatus()
            f.stamp = self.get_clock().now().to_msg()
            f.is_fallen = bool(self.get_parameter('simulate_fall').value)
            f.confidence = 0.95 if f.is_fallen else 0.1
            f.source = 'stub'
            f.detail = 'perception stub'
            self._fall_pub.publish(f)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PerceptionStubNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
