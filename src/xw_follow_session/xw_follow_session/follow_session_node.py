#!/usr/bin/env python3
"""Depth body-follow session skeleton."""

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool

from xw_interfaces.msg import PersonTracks, TaskResult
from xw_interfaces.srv import SessionControl


class FollowSessionNode(Node):
    def __init__(self) -> None:
        super().__init__('xw_follow_session')
        self.declare_parameter('follow_speed', 0.15)
        self.declare_parameter('stop_distance', 0.8)
        self._active = False
        self._command_id = ''
        latch = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self._cmd_pub = self.create_publisher(Twist, '/xw/cmd/follow', 10)
        self._result_pub = self.create_publisher(TaskResult, '/xw/task/result', 10)
        self.create_subscription(Bool, '/xw/follow/enable', self._on_enable, latch)
        self.create_subscription(PersonTracks, '/xw/perception/tracks', self._on_tracks, 10)
        self.create_service(SessionControl, '/xw/session/follow/control', self._on_control)
        self.create_timer(0.1, self._idle_zero)
        self.get_logger().info('follow session (stub) ready')

    def _on_enable(self, msg: Bool) -> None:
        self._active = bool(msg.data)
        if not self._active:
            self._cmd_pub.publish(Twist())

    def _on_control(self, req: SessionControl.Request, res: SessionControl.Response):
        self._command_id = req.command_id or 'follow-svc'
        self._active = bool(req.start)
        if not self._active:
            self._cmd_pub.publish(Twist())
            r = TaskResult()
            r.stamp = self.get_clock().now().to_msg()
            r.command_id = self._command_id
            r.capability = 'follow'
            r.code = 0
            r.message = 'stopped'
            self._result_pub.publish(r)
        res.success = True
        res.message = 'follow started' if self._active else 'follow stopped'
        res.state = 'active' if self._active else 'idle'
        return res

    def _on_tracks(self, msg: PersonTracks) -> None:
        if not self._active:
            return
        primary = None
        for t in msg.tracks:
            if t.is_primary:
                primary = t
                break
        if primary is None and msg.tracks:
            primary = msg.tracks[0]
        out = Twist()
        if primary is not None:
            stop_d = float(self.get_parameter('stop_distance').value)
            if primary.distance > stop_d:
                out.linear.x = float(self.get_parameter('follow_speed').value)
            out.angular.z = max(-0.4, min(0.4, -0.5 * primary.x))
        self._cmd_pub.publish(out)

    def _idle_zero(self) -> None:
        if not self._active:
            self._cmd_pub.publish(Twist())


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FollowSessionNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
