#!/usr/bin/env python3
"""Hardware straight-line drift test via /xw/motion/command (closed-loop distance on odom).

Measures lateral drift and yaw change over a fixed distance to separate
physical bias (motors/IMU/wheels) from Nav2 software oscillation.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
from typing import Optional, Tuple

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool
from std_srvs.srv import Trigger

from xw_interfaces.msg import TaskResult
from xw_interfaces.srv import MotionCommand


def _yaw(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def _lateral_drift(start: Tuple[float, float, float], end: Tuple[float, float, float]) -> Tuple[float, float]:
    """Return (lateral_m, along_m) in start heading frame."""
    sx, sy, syaw = start
    ex, ey, _ = end
    dx, dy = ex - sx, ey - sy
    along = dx * math.cos(syaw) + dy * math.sin(syaw)
    lateral = -dx * math.sin(syaw) + dy * math.cos(syaw)
    return lateral, along


class StraightLineTest(Node):
    def __init__(self) -> None:
        super().__init__('nav_hw_straight_test')
        self._odom: Optional[Odometry] = None
        self._wheel: Optional[Odometry] = None
        self._imu_wz = 0.0
        self._result: Optional[TaskResult] = None
        self.create_subscription(Odometry, '/odom', self._on_odom, 10)
        self.create_subscription(Odometry, '/odom/wheel', self._on_wheel, 10)
        self.create_subscription(Imu, '/imu/data', self._on_imu, 10)
        self.create_subscription(TaskResult, '/xw/task/result', self._on_result, 10)
        self._motion = self.create_client(MotionCommand, '/xw/motion/command')
        self._nav_cancel = self.create_client(Trigger, '/xw/nav/cancel')

    def _on_odom(self, msg: Odometry) -> None:
        self._odom = msg

    def _on_wheel(self, msg: Odometry) -> None:
        self._wheel = msg

    def _on_imu(self, msg: Imu) -> None:
        self._imu_wz = msg.angular_velocity.z

    def _on_result(self, msg: TaskResult) -> None:
        if msg.capability == 'motion':
            self._result = msg

    def _pose(self, msg: Optional[Odometry]) -> Optional[Tuple[float, float, float]]:
        if msg is None:
            return None
        p = msg.pose.pose.position
        return p.x, p.y, _yaw(msg.pose.pose.orientation)

    def _wait_odom(self, timeout: float = 5.0) -> bool:
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self._odom is not None:
                return True
        return False

    def run(self, distance_m: float, pause_nav: bool) -> dict:
        if not self._wait_odom():
            return {'ok': False, 'error': 'no odom'}

        if pause_nav and self._nav_cancel.wait_for_service(timeout_sec=2.0):
            req = Trigger.Request()
            fut = self._nav_cancel.call_async(req)
            rclpy.spin_until_future_complete(self, fut, timeout_sec=3.0)

        if not self._motion.wait_for_service(timeout_sec=3.0):
            return {'ok': False, 'error': 'motion service unavailable'}

        start_odom = self._pose(self._odom)
        start_wheel = self._pose(self._wheel)
        self._result = None

        req = MotionCommand.Request()
        req.command_id = f'hw-test-{int(time.time())}'
        req.angle_deg = 0.0
        req.distance_m = float(distance_m)

        fut = self._motion.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
        if not fut.result() or not fut.result().success:
            return {'ok': False, 'error': fut.result().message if fut.result() else 'call failed'}

        self.get_logger().info(f'driving {distance_m}m ...')
        deadline = time.monotonic() + abs(distance_m) / 0.15 + 15.0
        imu_wz_samples = []
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            imu_wz_samples.append(self._imu_wz)
            if self._result is not None:
                break

        end_odom = self._pose(self._odom)
        end_wheel = self._pose(self._wheel)

        if self._result is None:
            return {'ok': False, 'error': 'timeout waiting motion result'}

        report = {
            'ok': self._result.code == 0,
            'motion_code': int(self._result.code),
            'motion_msg': self._result.message,
            'distance_cmd_m': distance_m,
        }

        if start_odom and end_odom:
            lat, along = _lateral_drift(start_odom, end_odom)
            dyaw = end_odom[2] - start_odom[2]
            while dyaw > math.pi:
                dyaw -= 2 * math.pi
            while dyaw < -math.pi:
                dyaw += 2 * math.pi
            report['odom'] = {
                'lateral_m': lat,
                'along_m': along,
                'yaw_change_deg': math.degrees(dyaw),
                'start': start_odom,
                'end': end_odom,
            }

        if start_wheel and end_wheel:
            lat_w, along_w = _lateral_drift(start_wheel, end_wheel)
            dyaw_w = start_wheel[2] - end_wheel[2]
            dyaw_w = (end_wheel[2] - start_wheel[2])
            while dyaw_w > math.pi:
                dyaw_w -= 2 * math.pi
            while dyaw_w < -math.pi:
                dyaw_w += 2 * math.pi
            report['wheel'] = {
                'lateral_m': lat_w,
                'along_m': along_w,
                'yaw_change_deg': math.degrees(dyaw_w),
            }
            if start_odom and end_odom:
                report['odom_wheel_yaw_diff_end_deg'] = math.degrees(
                    end_odom[2] - end_wheel[2]
                )

        if imu_wz_samples:
            mean_wz = sum(imu_wz_samples) / len(imu_wz_samples)
            report['imu'] = {
                'mean_wz_rad_s': mean_wz,
                'mean_wz_deg_s': math.degrees(mean_wz),
                'integrated_yaw_est_deg': math.degrees(
                    mean_wz * len(imu_wz_samples) * 0.05
                ),
            }

        return report


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument('--distance', type=float, default=2.0)
    parser.add_argument('--no-cancel-nav', action='store_true')
    parser.add_argument('--out', default='')
    args = parser.parse_args()

    rclpy.init()
    node = StraightLineTest()
    try:
        report = node.run(args.distance, pause_nav=not args.no_cancel_nav)
    finally:
        node.destroy_node()
        rclpy.shutdown()

    text = json.dumps(report, indent=2, ensure_ascii=False)
    print(text)
    if args.out:
        with open(args.out, 'w', encoding='utf-8') as f:
            f.write(text)
    sys.exit(0 if report.get('ok') else 1)


if __name__ == '__main__':
    main()
