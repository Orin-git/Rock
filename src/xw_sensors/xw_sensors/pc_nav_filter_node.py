#!/usr/bin/env python3
"""Crop + voxel downsample depth PointCloud2 for Nav2 local costmap.

Subscribes gated raw clouds (already opened by set_pointcloud_nav) and publishes
sparse ``*_points_nav`` at a capped rate to keep Rock 5T CPU bounded.
"""

from __future__ import annotations

import math
import struct
from typing import List, Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header

_PC_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)


def _find_xyz_offsets(msg: PointCloud2) -> Optional[Tuple[int, int, int, int]]:
    """Return (x_off, y_off, z_off, point_step) for FLOAT32 xyz, or None."""
    offs = {}
    for f in msg.fields:
        if f.name in ('x', 'y', 'z') and f.datatype == PointField.FLOAT32:
            offs[f.name] = int(f.offset)
    if len(offs) < 3:
        return None
    return offs['x'], offs['y'], offs['z'], int(msg.point_step)


def _pack_xyz_cloud(header: Header, points: List[Tuple[float, float, float]]) -> PointCloud2:
    msg = PointCloud2()
    msg.header = header
    msg.height = 1
    msg.width = len(points)
    msg.is_bigendian = False
    msg.is_dense = True
    msg.point_step = 12
    msg.row_step = msg.point_step * msg.width
    msg.fields = [
        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
    ]
    buf = bytearray(msg.row_step)
    for i, (x, y, z) in enumerate(points):
        struct.pack_into('<fff', buf, i * 12, x, y, z)
    msg.data = bytes(buf)
    return msg


class PcNavFilterNode(Node):
    def __init__(self) -> None:
        super().__init__('xw_pc_nav_filter')
        self.declare_parameter('input_topics', [
            '/camera/front_up/depth/points',
            '/camera/front_down/depth/points',
        ])
        self.declare_parameter('output_topics', [
            '/camera/front_up/depth/points_nav',
            '/camera/front_down/depth/points_nav',
        ])
        # Optical frame: Z forward depth, X right, Y down — crop in sensor frame.
        self.declare_parameter('z_min', 0.20)
        self.declare_parameter('z_max', 2.50)
        self.declare_parameter('abs_x_max', 1.20)
        self.declare_parameter('y_min', -0.80)
        self.declare_parameter('y_max', 0.40)
        self.declare_parameter('voxel_leaf', 0.06)
        self.declare_parameter('max_rate_hz', 5.0)
        self.declare_parameter('max_points_out', 2500)
        self.declare_parameter('stride', 4)

        inputs = list(self.get_parameter('input_topics').value)
        outputs = list(self.get_parameter('output_topics').value)
        if len(inputs) != len(outputs):
            raise RuntimeError('input_topics and output_topics length mismatch')

        self._pubs = []
        self._last_pub = [0.0] * len(inputs)
        for i, (tin, tout) in enumerate(zip(inputs, outputs)):
            pub = self.create_publisher(PointCloud2, tout, _PC_QOS)
            self._pubs.append(pub)
            self.create_subscription(
                PointCloud2,
                tin,
                lambda msg, idx=i: self._on_cloud(idx, msg),
                _PC_QOS,
            )
        self.get_logger().info(
            f'pc_nav_filter ready: {len(inputs)} streams → *_points_nav @'
            f'{float(self.get_parameter("max_rate_hz").value):.1f} Hz'
        )

    def _on_cloud(self, idx: int, msg: PointCloud2) -> None:
        rate = float(self.get_parameter('max_rate_hz').value)
        now = self.get_clock().now().nanoseconds * 1e-9
        min_dt = 1.0 / max(rate, 0.5)
        if now - self._last_pub[idx] < min_dt:
            return

        xyz = _find_xyz_offsets(msg)
        if xyz is None or msg.width * msg.height == 0:
            return
        x_off, y_off, z_off, step = xyz
        if step <= 0 or len(msg.data) < step:
            return

        z_min = float(self.get_parameter('z_min').value)
        z_max = float(self.get_parameter('z_max').value)
        abs_x_max = float(self.get_parameter('abs_x_max').value)
        y_min = float(self.get_parameter('y_min').value)
        y_max = float(self.get_parameter('y_max').value)
        leaf = max(float(self.get_parameter('voxel_leaf').value), 0.02)
        stride = max(int(self.get_parameter('stride').value), 1)
        max_out = max(int(self.get_parameter('max_points_out').value), 100)

        data = msg.data
        n = msg.width * msg.height
        voxels = {}
        for i in range(0, n, stride):
            base = i * step
            if base + max(x_off, y_off, z_off) + 4 > len(data):
                break
            try:
                x = struct.unpack_from('<f', data, base + x_off)[0]
                y = struct.unpack_from('<f', data, base + y_off)[0]
                z = struct.unpack_from('<f', data, base + z_off)[0]
            except struct.error:
                break
            if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(z)):
                continue
            if z < z_min or z > z_max:
                continue
            if abs(x) > abs_x_max:
                continue
            if y < y_min or y > y_max:
                continue
            key = (int(math.floor(x / leaf)), int(math.floor(y / leaf)), int(math.floor(z / leaf)))
            if key not in voxels:
                voxels[key] = (x, y, z)
                if len(voxels) >= max_out:
                    break

        points = list(voxels.values())
        # Preserve acquisition stamp (do not stamp with now()).
        out = _pack_xyz_cloud(msg.header, points)
        self._pubs[idx].publish(out)
        self._last_pub[idx] = now


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PcNavFilterNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
