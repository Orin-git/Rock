#!/usr/bin/env python3
"""Crop + voxel + outlier filters for Nav2 local costmap depth clouds.

Pipeline (market-standard depth cleanup, same order as typical PCL stacks):
  1. Pass-through / ROI crop (optical frame: Z forward, X right, Y down)
  2. Voxel downsample
  3. Statistical outlier removal (SOR)
  4. Radius outlier removal

Subscribes gated raw clouds (opened by set_pointcloud_nav) and publishes sparse
``*_points_nav`` at a capped rate for Rock 5T. All stages are yaml-tunable;
SOR/Radius can be toggled independently for A/B tuning.
"""

from __future__ import annotations

import math
import struct
from typing import Optional, Tuple

import numpy as np
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


def _pack_xyz_cloud(header: Header, points: np.ndarray) -> PointCloud2:
    msg = PointCloud2()
    msg.header = header
    msg.height = 1
    n = int(points.shape[0])
    msg.width = n
    msg.is_bigendian = False
    msg.is_dense = True
    msg.point_step = 12
    msg.row_step = msg.point_step * msg.width
    msg.fields = [
        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
    ]
    if n == 0:
        msg.data = b''
        return msg
    # Contiguous float32 xyz → bytes (little-endian host assumed, same as before).
    msg.data = np.ascontiguousarray(points, dtype=np.float32).tobytes()
    return msg


def _pairwise_sq(pts: np.ndarray) -> np.ndarray:
    """NxN squared Euclidean distances (float32). N is post-voxel (≤ few k)."""
    # ||a-b||^2 = ||a||^2 + ||b||^2 - 2 a·b
    sq = np.einsum('ij,ij->i', pts, pts, dtype=np.float32)
    d2 = sq[:, None] + sq[None, :] - 2.0 * (pts @ pts.T)
    np.maximum(d2, 0.0, out=d2)
    return d2


def statistical_outlier_removal(
    pts: np.ndarray, mean_k: int, stddev_mul: float
) -> np.ndarray:
    """PCL-style StatisticalOutlierRemoval on (N,3) xyz."""
    n = pts.shape[0]
    k = max(int(mean_k), 1)
    if n <= k + 1:
        return pts
    d2 = _pairwise_sq(pts)
    # Exclude self: partition so index 0..k are the k+1 smallest (self + k nbrs).
    part = np.partition(d2, k, axis=1)[:, 1 : k + 1]
    mean_dist = np.sqrt(part, dtype=np.float32).mean(axis=1)
    mu = float(mean_dist.mean())
    sigma = float(mean_dist.std())
    keep = mean_dist <= (mu + float(stddev_mul) * sigma)
    return pts[keep]


def radius_outlier_removal(
    pts: np.ndarray, radius: float, min_neighbors: int
) -> np.ndarray:
    """PCL-style RadiusOutlierRemoval on (N,3) xyz."""
    n = pts.shape[0]
    if n == 0:
        return pts
    r2 = float(radius) * float(radius)
    need = max(int(min_neighbors), 1)
    d2 = _pairwise_sq(pts)
    # Count neighbors inside radius, excluding self.
    counts = (d2 <= r2).sum(axis=1) - 1
    return pts[counts >= need]


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
        # Optical frame ROI (PassThrough / CropBox equivalent).
        self.declare_parameter('z_min', 0.20)
        self.declare_parameter('z_max', 2.50)
        self.declare_parameter('abs_x_max', 1.20)
        self.declare_parameter('y_min', -0.80)
        self.declare_parameter('y_max', 0.40)
        self.declare_parameter('voxel_leaf', 0.06)
        self.declare_parameter('max_rate_hz', 5.0)
        self.declare_parameter('max_points_out', 2500)
        self.declare_parameter('stride', 4)
        # Statistical outlier removal (after voxel).
        self.declare_parameter('sor_enable', True)
        self.declare_parameter('sor_mean_k', 8)
        self.declare_parameter('sor_stddev_mul', 1.0)
        # Radius outlier removal (after SOR).
        self.declare_parameter('radius_enable', True)
        self.declare_parameter('radius_search', 0.12)
        self.declare_parameter('radius_min_neighbors', 5)
        # Optional per-stream overrides (same order as input_topics).
        self.declare_parameter('stream_y_min', [-0.80, -1.20])
        self.declare_parameter('stream_y_max', [0.40, 1.00])
        self.declare_parameter('stream_stride', [4, 2])
        self.declare_parameter('stream_sor_enable', [True, False])
        self.declare_parameter('stream_radius_enable', [True, False])

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
        sor_on = bool(self.get_parameter('sor_enable').value)
        rad_on = bool(self.get_parameter('radius_enable').value)
        self.get_logger().info(
            f'pc_nav_filter ready: {len(inputs)} streams → *_points_nav @'
            f'{float(self.get_parameter("max_rate_hz").value):.1f} Hz '
            f'(crop+voxel'
            f'{"+SOR" if sor_on else ""}'
            f'{"+radius" if rad_on else ""})'
        )

    def _stream_param(self, name: str, idx: int, default):
        vals = list(self.get_parameter(name).value)
        if idx < len(vals):
            return vals[idx]
        return default

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
        y_min = float(self._stream_param(
            'stream_y_min', idx, float(self.get_parameter('y_min').value)))
        y_max = float(self._stream_param(
            'stream_y_max', idx, float(self.get_parameter('y_max').value)))
        leaf = max(float(self.get_parameter('voxel_leaf').value), 0.02)
        stride = max(int(self._stream_param(
            'stream_stride', idx, int(self.get_parameter('stride').value))), 1)
        max_out = max(int(self.get_parameter('max_points_out').value), 100)
        sor_on = bool(self._stream_param(
            'stream_sor_enable', idx, bool(self.get_parameter('sor_enable').value)))
        rad_on = bool(self._stream_param(
            'stream_radius_enable', idx, bool(self.get_parameter('radius_enable').value)))

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
            key = (
                int(math.floor(x / leaf)),
                int(math.floor(y / leaf)),
                int(math.floor(z / leaf)),
            )
            if key not in voxels:
                voxels[key] = (x, y, z)
                if len(voxels) >= max_out:
                    break

        if not voxels:
            out = _pack_xyz_cloud(msg.header, np.empty((0, 3), dtype=np.float32))
            self._pubs[idx].publish(out)
            self._last_pub[idx] = now
            return

        pts = np.asarray(list(voxels.values()), dtype=np.float32)

        if sor_on:
            pts = statistical_outlier_removal(
                pts,
                int(self.get_parameter('sor_mean_k').value),
                float(self.get_parameter('sor_stddev_mul').value),
            )

        if rad_on and pts.shape[0] > 0:
            pts = radius_outlier_removal(
                pts,
                float(self.get_parameter('radius_search').value),
                int(self.get_parameter('radius_min_neighbors').value),
            )

        # Preserve acquisition stamp (do not stamp with now()).
        out = _pack_xyz_cloud(msg.header, pts)
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
