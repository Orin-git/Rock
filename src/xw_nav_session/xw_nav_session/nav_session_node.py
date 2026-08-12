#!/usr/bin/env python3
"""Navigation session skeleton."""

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool

from xw_interfaces.msg import TaskProgress, TaskResult
from xw_interfaces.srv import SessionControl


class NavSessionNode(Node):
    def __init__(self) -> None:
        super().__init__('xw_nav_session')
        self._active = False
        self._command_id = ''
        latch = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self._progress_pub = self.create_publisher(TaskProgress, '/xw/task/progress', 10)
        self._result_pub = self.create_publisher(TaskResult, '/xw/task/result', 10)
        self.create_subscription(Bool, '/xw/nav/enable', self._on_enable, latch)
        self.create_subscription(PoseStamped, '/xw/goal_pose', self._on_goal, 10)
        self.create_service(SessionControl, '/xw/session/nav/control', self._on_control)
        self.get_logger().info('nav session (stub) ready')

    def _on_enable(self, msg: Bool) -> None:
        self._active = bool(msg.data)
        self.get_logger().info(f'nav active={self._active}')

    def _on_control(self, req: SessionControl.Request, res: SessionControl.Response):
        self._command_id = req.command_id or 'nav-svc'
        self._active = bool(req.start)
        res.success = True
        res.message = 'nav started' if self._active else 'nav stopped'
        res.state = 'active' if self._active else 'idle'
        return res

    def _on_goal(self, msg: PoseStamped) -> None:
        if not self._active:
            self.get_logger().warn('goal ignored (nav inactive)')
            return
        x = float(msg.pose.position.x)
        y = float(msg.pose.position.y)
        self.get_logger().info(f'goal accepted x={x:.3f} y={y:.3f} (Nav2 TBD)')
        p = TaskProgress()
        p.stamp = self.get_clock().now().to_msg()
        p.command_id = self._command_id
        p.capability = 'nav'
        p.phase = 'goal_accepted'
        self._progress_pub.publish(p)
        r = TaskResult()
        r.stamp = self.get_clock().now().to_msg()
        r.command_id = self._command_id
        r.capability = 'nav'
        r.code = 3
        r.message = 'goal received (Nav2 not wired yet)'
        r.data_json = f'{{"x": {x}, "y": {y}, "frame_id": "{msg.header.frame_id}"}}'
        self._result_pub.publish(r)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = NavSessionNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
