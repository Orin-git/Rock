#!/usr/bin/env python3
"""xw_last_good_pose_writer — ≤1 Hz; writes maps/<map>/state/last_good_pose.yaml.

Write only after all gates hold for stable_sec. Never writes during Follow
(legacy_freeze open-loop). Proposal only — readers must validate + laser-verify.
"""

from __future__ import annotations

import math
import time
from typing import Optional, Tuple

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Int8, String
from tf2_ros import Buffer, TransformException, TransformListener

from xw_phase2c.last_good_pose import (
    LastGoodPose,
    compute_map_hash,
    pose_delta,
    quality_from_cov,
    write_last_good_pose,
    yaw_from_quat,
)


_LATCH = QoSProfile(
    depth=1,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    reliability=ReliabilityPolicy.RELIABLE,
)
_AMCL_QOS = QoSProfile(
    depth=1,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
)


class LastGoodPoseWriter(Node):
    def __init__(self) -> None:
        super().__init__('xw_last_good_pose_writer')
        self.declare_parameter('maps_dir', '/ros2_ws/maps')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('tick_hz', 1.0)
        self.declare_parameter('stable_sec', 5.0)
        self.declare_parameter('tf_stale_sec', 1.0)
        self.declare_parameter('cov_xy_good', 0.5)
        self.declare_parameter('cov_yaw_good', 0.25)
        self.declare_parameter('stable_xy_m', 0.15)
        self.declare_parameter('stable_yaw_rad', 0.12)
        self.declare_parameter('min_write_interval_sec', 10.0)
        # Production Follow uses legacy_freeze — block all follow writes by default.
        self.declare_parameter('block_write_during_follow', True)
        self.declare_parameter('enabled', True)

        self._tf = Buffer()
        self._tf_listener = TransformListener(self._tf, self)

        self._amcl: Optional[PoseWithCovarianceStamped] = None
        self._loc_status = 1
        self._follow_en = False
        self._map_name = ''
        self._good_since: Optional[float] = None
        self._anchor: Optional[Tuple[float, float, float]] = None
        self._last_write_mono = 0.0
        self._last_hash = ''

        self.create_subscription(PoseWithCovarianceStamped, 'amcl_pose', self._on_amcl, _AMCL_QOS)
        self.create_subscription(Int8, '/xw/localization_status', self._on_status, _LATCH)
        self.create_subscription(Bool, '/xw/follow/enable', self._on_follow, _LATCH)
        self.create_subscription(String, '/xw/nav/map_name', self._on_map, _LATCH)

        hz = max(0.2, float(self.get_parameter('tick_hz').value))
        self.create_timer(1.0 / hz, self._tick)
        self.get_logger().info(
            f'last_good_pose writer ready maps_dir={self.get_parameter("maps_dir").value} '
            f'stable_sec={self.get_parameter("stable_sec").value} '
            f'block_follow={self.get_parameter("block_write_during_follow").value}'
        )

    def _now(self) -> float:
        return time.monotonic()

    def _on_amcl(self, msg: PoseWithCovarianceStamped) -> None:
        self._amcl = msg

    def _on_status(self, msg: Int8) -> None:
        self._loc_status = int(msg.data)

    def _on_follow(self, msg: Bool) -> None:
        self._follow_en = bool(msg.data)
        if self._follow_en:
            self._good_since = None
            self._anchor = None

    def _on_map(self, msg: String) -> None:
        name = (msg.data or '').strip()
        if name and name != self._map_name:
            self._map_name = name
            self._good_since = None
            self._anchor = None
            self.get_logger().info(f'active map={name}')

    def _tf_fresh(self, parent: str, child: str) -> bool:
        stale = float(self.get_parameter('tf_stale_sec').value)
        try:
            tf = self._tf.lookup_transform(
                parent,
                child,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.05),
            )
            if tf.header.stamp.sec == 0 and tf.header.stamp.nanosec == 0:
                return True
            age = (self.get_clock().now() - rclpy.time.Time.from_msg(tf.header.stamp)).nanoseconds * 1e-9
            return age < stale
        except TransformException:
            return False

    def _pose_tuple(self) -> Optional[Tuple[float, float, float]]:
        if self._amcl is None:
            return None
        p = self._amcl.pose.pose
        return (float(p.position.x), float(p.position.y), yaw_from_quat(p.orientation))

    def _cov_xy_yaw(self) -> Tuple[float, float]:
        if self._amcl is None:
            return 999.0, 999.0
        c = self._amcl.pose.covariance
        return max(float(c[0]), float(c[7])), float(c[35])

    def _gates_ok(self) -> Tuple[bool, str]:
        if not bool(self.get_parameter('enabled').value):
            return False, 'disabled'
        if self._loc_status != 0:
            return False, f'status={self._loc_status}'
        if bool(self.get_parameter('block_write_during_follow').value) and self._follow_en:
            return False, 'follow_freeze_block'
        if not self._map_name:
            return False, 'no_active_map'
        if self._amcl is None:
            return False, 'no_amcl'
        map_f = str(self.get_parameter('map_frame').value)
        odom_f = str(self.get_parameter('odom_frame').value)
        base_f = str(self.get_parameter('base_frame').value)
        if not self._tf_fresh(map_f, odom_f):
            return False, 'tf_map_odom_stale'
        if not self._tf_fresh(map_f, base_f):
            return False, 'tf_map_base_stale'
        xy, yaw = self._cov_xy_yaw()
        if xy >= float(self.get_parameter('cov_xy_good').value):
            return False, f'cov_xy={xy:.3f}'
        if yaw >= float(self.get_parameter('cov_yaw_good').value):
            return False, f'cov_yaw={yaw:.3f}'
        maps_dir = str(self.get_parameter('maps_dir').value)
        h = compute_map_hash(maps_dir, self._map_name)
        if not h:
            return False, 'map_hash_unavailable'
        self._last_hash = h
        pose = self._pose_tuple()
        if pose is None:
            return False, 'no_pose'
        if self._anchor is None:
            self._anchor = pose
            return True, 'anchor_set'
        dxy, dyaw = pose_delta(pose, self._anchor)
        if dxy > float(self.get_parameter('stable_xy_m').value) or dyaw > float(
            self.get_parameter('stable_yaw_rad').value
        ):
            self._anchor = pose
            return False, 'pose_moved'
        return True, 'ok'

    def _tick(self) -> None:
        ok, reason = self._gates_ok()
        now = self._now()
        if not ok:
            if reason != 'anchor_set':
                self._good_since = None
            elif self._good_since is None:
                self._good_since = now
            return
        if self._good_since is None:
            self._good_since = now
            return
        need = float(self.get_parameter('stable_sec').value)
        if now - self._good_since < need:
            return
        if now - self._last_write_mono < float(self.get_parameter('min_write_interval_sec').value):
            return
        pose = self._pose_tuple()
        if pose is None:
            return
        xy, yaw_c = self._cov_xy_yaw()
        q = quality_from_cov(
            xy,
            xy,
            yaw_c,
            float(self.get_parameter('cov_xy_good').value),
            float(self.get_parameter('cov_yaw_good').value),
        )
        rec = LastGoodPose(
            map_name=self._map_name,
            map_hash=self._last_hash,
            timestamp=time.time(),
            x=pose[0],
            y=pose[1],
            yaw=pose[2],
            covariance=[xy, xy, yaw_c],
            source='amcl',
            quality=q,
        )
        try:
            path = write_last_good_pose(str(self.get_parameter('maps_dir').value), rec)
            self._last_write_mono = now
            self.get_logger().info(
                f'wrote last_good_pose {path} xy=({pose[0]:.2f},{pose[1]:.2f}) '
                f'q={q:.2f} hash={self._last_hash[:8]}…'
            )
        except OSError as exc:
            self.get_logger().error(f'write failed: {exc}')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LastGoodPoseWriter()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
