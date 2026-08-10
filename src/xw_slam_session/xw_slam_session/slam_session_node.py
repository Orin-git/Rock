#!/usr/bin/env python3
"""SLAM session skeleton — reacts to /xw/slam/enable + optional service."""

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool

from xw_interfaces.msg import TaskProgress, TaskResult
from xw_interfaces.srv import SessionControl


class SlamSessionNode(Node):
    def __init__(self) -> None:
        super().__init__('xw_slam_session')
        self._active = False
        self._command_id = ''
        latch = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self._progress_pub = self.create_publisher(TaskProgress, '/xw/task/progress', 10)
        self._result_pub = self.create_publisher(TaskResult, '/xw/task/result', 10)
        self.create_subscription(Bool, '/xw/slam/enable', self._on_enable, latch)
        self.create_service(SessionControl, '/xw/session/slam/control', self._on_control)
        self.get_logger().info('slam session (stub) ready')

    def _on_enable(self, msg: Bool) -> None:
        self._set_active(bool(msg.data), 'enable-topic')

    def _on_control(self, req: SessionControl.Request, res: SessionControl.Response):
        self._command_id = req.command_id or 'slam-svc'
        self._set_active(bool(req.start), 'service')
        res.success = True
        res.message = 'slam started' if self._active else 'slam stopped'
        res.state = 'active' if self._active else 'idle'
        return res

    def _set_active(self, active: bool, source: str) -> None:
        self._active = active
        if active:
            self.get_logger().info(f'slam active ({source})')
            p = TaskProgress()
            p.stamp = self.get_clock().now().to_msg()
            p.command_id = self._command_id or source
            p.capability = 'slam'
            p.phase = 'active'
            self._progress_pub.publish(p)
        else:
            r = TaskResult()
            r.stamp = self.get_clock().now().to_msg()
            r.command_id = self._command_id or source
            r.capability = 'slam'
            r.code = 0
            r.message = 'stopped'
            self._result_pub.publish(r)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SlamSessionNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
