#!/usr/bin/env python3
"""Mock / future serial chassis: /cmd_vel in, odom + power out."""

import math

import rclpy
from geometry_msgs.msg import Quaternion, TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Bool
from tf2_ros import TransformBroadcaster

from xw_interfaces.msg import PowerState


def yaw_to_quat(yaw: float) -> Quaternion:
    q = Quaternion()
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q


class ChassisNode(Node):
    def __init__(self) -> None:
        super().__init__('xw_chassis')
        self.declare_parameter('use_sim_hw', True)
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('wheel_separation', 0.35)
        self.declare_parameter('max_linear', 0.5)
        self.declare_parameter('max_angular', 1.0)

        self._use_sim = bool(self.get_parameter('use_sim_hw').value)
        self._publish_tf = bool(self.get_parameter('publish_tf').value)
        self._base = str(self.get_parameter('base_frame').value)
        self._odom_frame = str(self.get_parameter('odom_frame').value)

        self._x = 0.0
        self._y = 0.0
        self._yaw = 0.0
        self._vx = 0.0
        self._wz = 0.0
        self._estop = False

        self._cmd_sub = self.create_subscription(Twist, 'cmd_vel', self._on_cmd, 10)
        self._estop_sub = self.create_subscription(Bool, 'emergency_stop', self._on_estop, 10)
        self._odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self._power_pub = self.create_publisher(PowerState, '/xw/power', 10)
        self._estop_pub = self.create_publisher(Bool, 'emergency_stop', 10)
        self._tf_broadcaster = TransformBroadcaster(self)
        self._timer = self.create_timer(0.05, self._tick)
        self._power_timer = self.create_timer(1.0, self._publish_power)

        self.get_logger().info(
            f'chassis started (use_sim_hw={self._use_sim})'
        )

    def _on_estop(self, msg: Bool) -> None:
        self._estop = bool(msg.data)
        if self._estop:
            self._vx = 0.0
            self._wz = 0.0

    def _on_cmd(self, msg: Twist) -> None:
        if self._estop:
            self._vx = 0.0
            self._wz = 0.0
            return
        max_lin = float(self.get_parameter('max_linear').value)
        max_ang = float(self.get_parameter('max_angular').value)
        self._vx = max(-max_lin, min(max_lin, msg.linear.x))
        self._wz = max(-max_ang, min(max_ang, msg.angular.z))

    def _tick(self) -> None:
        dt = 0.05
        if self._use_sim and not self._estop:
            self._yaw += self._wz * dt
            self._x += self._vx * math.cos(self._yaw) * dt
            self._y += self._vx * math.sin(self._yaw) * dt

        now = self.get_clock().now().to_msg()
        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = self._odom_frame
        odom.child_frame_id = self._base
        odom.pose.pose.position.x = self._x
        odom.pose.pose.position.y = self._y
        odom.pose.pose.orientation = yaw_to_quat(self._yaw)
        odom.twist.twist.linear.x = self._vx
        odom.twist.twist.angular.z = self._wz
        self._odom_pub.publish(odom)

        if self._publish_tf:
            t = TransformStamped()
            t.header.stamp = now
            t.header.frame_id = self._odom_frame
            t.child_frame_id = self._base
            t.transform.translation.x = self._x
            t.transform.translation.y = self._y
            t.transform.rotation = yaw_to_quat(self._yaw)
            self._tf_broadcaster.sendTransform(t)

    def _publish_power(self) -> None:
        p = PowerState()
        p.stamp = self.get_clock().now().to_msg()
        p.battery_percent = 88.0
        p.voltage = 24.5
        p.charging = False
        p.docked = False
        p.detail = 'mock'
        self._power_pub.publish(p)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ChassisNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
