#!/usr/bin/env python3
"""Fall detection session skeleton."""

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool

from xw_interfaces.msg import FallStatus, TaskResult
from xw_interfaces.srv import SessionControl


class FallSessionNode(Node):
    def __init__(self) -> None:
        super().__init__('xw_fall_session')
        self._active = False
        self._command_id = ''
        latch = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self._status_pub = self.create_publisher(FallStatus, '/xw/fall/status', 10)
        self._result_pub = self.create_publisher(TaskResult, '/xw/task/result', 10)
        self.create_subscription(Bool, '/xw/fall/enable', self._on_enable, latch)
        self.create_subscription(FallStatus, '/xw/perception/fall', self._on_fall, 10)
        self.create_service(SessionControl, '/xw/session/fall/control', self._on_control)
        self.get_logger().info('fall session (stub) ready')

    def _on_enable(self, msg: Bool) -> None:
        self._active = bool(msg.data)

    def _on_control(self, req: SessionControl.Request, res: SessionControl.Response):
        self._command_id = req.command_id or 'fall-svc'
        self._active = bool(req.start)
        res.success = True
        res.message = 'fall detect started' if self._active else 'fall detect stopped'
        res.state = 'active' if self._active else 'idle'
        return res

    def _on_fall(self, msg: FallStatus) -> None:
        if not self._active:
            return
        self._status_pub.publish(msg)
        if msg.is_fallen:
            r = TaskResult()
            r.stamp = self.get_clock().now().to_msg()
            r.command_id = self._command_id
            r.capability = 'fall'
            r.code = 1
            r.message = 'fall detected'
            r.data_json = f'{{"confidence": {msg.confidence}}}'
            self._result_pub.publish(r)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FallSessionNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
