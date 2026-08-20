#!/usr/bin/env python3
"""Angle + distance jog via /xw/cmd/motion."""

import math
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node

from xw_interfaces.msg import MotionStatus, TaskProgress, TaskResult
from xw_interfaces.srv import MotionCommand


class MotionNode(Node):
    IDLE, TURN, DRIVE, DONE = 0, 1, 2, 3

    def __init__(self) -> None:
        super().__init__('xw_motion')
        self.declare_parameter('linear_speed', 0.2)
        self.declare_parameter('angular_speed', 0.5)
        self.declare_parameter('angle_tol_deg', 3.0)
        self.declare_parameter('dist_tol_m', 0.03)
        self.declare_parameter('timeout_margin_sec', 5.0)
        self.declare_parameter('preempt', True)

        self._cb = ReentrantCallbackGroup()
        self._state = self.IDLE
        self._cmd_id = ''
        self._target_yaw = 0.0
        self._target_dist = 0.0
        self._start_x = 0.0
        self._start_y = 0.0
        self._odom_yaw = 0.0
        self._odom_x = 0.0
        self._odom_y = 0.0
        self._have_odom = False
        self._deadline = 0.0
        self._drive_sign = 1.0  # +1 forward, -1 backward

        self.create_subscription(Odometry, 'odom', self._on_odom, 10)
        self._cmd_pub = self.create_publisher(Twist, '/xw/cmd/motion', 10)
        self._status_pub = self.create_publisher(MotionStatus, '/xw/motion/status', 10)
        self._progress_pub = self.create_publisher(TaskProgress, '/xw/task/progress', 10)
        self._result_pub = self.create_publisher(TaskResult, '/xw/task/result', 10)
        self.create_service(
            MotionCommand, '/xw/motion/command', self._on_command, callback_group=self._cb
        )
        self.create_timer(0.05, self._tick)
        self.get_logger().info('motion node ready')

    def _on_odom(self, msg: Odometry) -> None:
        self._odom_x = msg.pose.pose.position.x
        self._odom_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self._odom_yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )
        self._have_odom = True

    def _on_command(self, req: MotionCommand.Request, res: MotionCommand.Response):
        if not self._have_odom:
            res.success = False
            res.message = 'no odom yet'
            return res

        preempt = bool(self.get_parameter('preempt').value)
        if self._state not in (self.IDLE, self.DONE):
            if not preempt:
                res.success = False
                res.message = 'busy'
                return res
            self._cmd_pub.publish(Twist())
            self._finish(1, 'preempted')

        self._cmd_id = req.command_id or f'motion-{self.get_clock().now().nanoseconds}'
        self._start_x = self._odom_x
        self._start_y = self._odom_y
        angle = math.radians(float(req.angle_deg))
        self._target_yaw = self._odom_yaw + angle
        dist_signed = float(req.distance_m)
        self._drive_sign = 1.0 if dist_signed >= 0.0 else -1.0
        self._target_dist = abs(dist_signed)
        lin = max(0.05, float(self.get_parameter('linear_speed').value))
        ang = max(0.05, float(self.get_parameter('angular_speed').value))
        margin = float(self.get_parameter('timeout_margin_sec').value)
        turn_t = abs(angle) / ang if abs(req.angle_deg) > 0.1 else 0.0
        drive_t = self._target_dist / lin if self._target_dist > 1e-3 else 0.0
        self._deadline = time.monotonic() + turn_t + drive_t + margin

        self._state = self.TURN if abs(req.angle_deg) > 0.1 else self.DRIVE
        if abs(req.angle_deg) <= 0.1 and self._target_dist < 1e-3:
            self._finish(0, 'noop')
            res.success = True
            res.message = 'noop'
            return res
        res.success = True
        direction = '往后走' if self._drive_sign < 0 else '往前走'
        dist_s = f'{self._target_dist:.1f}'.rstrip('0').rstrip('.')
        res.message = f'{direction} {dist_s}米'
        self._publish_status(
            self.TURN if self._state == self.TURN else self.DRIVE, 'started'
        )
        return res

    def _ang_diff(self, a: float, b: float) -> float:
        return (a - b + math.pi) % (2 * math.pi) - math.pi

    def _traveled(self) -> float:
        return math.hypot(self._odom_x - self._start_x, self._odom_y - self._start_y)

    def _tick(self) -> None:
        if self._state in (self.IDLE, self.DONE) or not self._have_odom:
            if self._state == self.IDLE:
                self._cmd_pub.publish(Twist())
            return

        if self._deadline > 0.0 and time.monotonic() > self._deadline:
            self._cmd_pub.publish(Twist())
            self._finish(2, '走动超时了')
            return

        out = Twist()
        ang_sp = float(self.get_parameter('angular_speed').value)
        lin_sp = float(self.get_parameter('linear_speed').value)
        ang_tol = math.radians(float(self.get_parameter('angle_tol_deg').value))
        dist_tol = float(self.get_parameter('dist_tol_m').value)

        if self._state == self.TURN:
            err = self._ang_diff(self._target_yaw, self._odom_yaw)
            if abs(err) < ang_tol:
                if self._target_dist > dist_tol:
                    self._start_x = self._odom_x
                    self._start_y = self._odom_y
                    self._state = self.DRIVE
                    self._publish_status(self.DRIVE, 'driving')
                else:
                    self._finish(0, 'done')
                self._cmd_pub.publish(Twist())
                return
            out.angular.z = ang_sp if err > 0 else -ang_sp
            self._cmd_pub.publish(out)
            self._publish_progress('turn')
            return

        if self._state == self.DRIVE:
            traveled = self._traveled()
            if traveled >= self._target_dist - dist_tol:
                self._finish(0, 'done')
                self._cmd_pub.publish(Twist())
                return
            out.linear.x = lin_sp * self._drive_sign
            self._cmd_pub.publish(out)
            pct = (
                100.0 * min(1.0, traveled / self._target_dist)
                if self._target_dist > 1e-6
                else 100.0
            )
            # Stable phase text (no distance spam) for pet / UI.
            self._publish_progress(
                'back' if self._drive_sign < 0 else 'fwd', percent=pct
            )

    def _publish_progress(self, phase: str, percent: float = 0.0) -> None:
        p = TaskProgress()
        p.stamp = self.get_clock().now().to_msg()
        p.command_id = self._cmd_id
        p.capability = 'motion'
        p.phase = phase
        p.percent = float(percent)
        self._progress_pub.publish(p)

    def _publish_status(self, status: int, message: str) -> None:
        m = MotionStatus()
        m.stamp = self.get_clock().now().to_msg()
        m.command_id = self._cmd_id
        m.status = status
        m.message = message
        self._status_pub.publish(m)
        self._publish_progress(message)

    def _finish(self, code: int, message: str) -> None:
        self._cmd_pub.publish(Twist())
        self._publish_status(self.DONE if code == 0 else 5, message)
        r = TaskResult()
        r.stamp = self.get_clock().now().to_msg()
        r.command_id = self._cmd_id
        r.capability = 'motion'
        r.code = code
        r.message = message
        self._result_pub.publish(r)
        self._state = self.IDLE
        self._deadline = 0.0


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MotionNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
