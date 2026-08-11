#!/usr/bin/env python3
"""SLAM session: launch/stop slam_toolbox, capture start pose, autosave on stop."""

from __future__ import annotations

import json
import math
import os
import signal
import subprocess
import threading
import time
from datetime import datetime
from pathlib import Path
from typing import Optional, Tuple

import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Pose2D
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import Bool
from tf2_ros import Buffer, TransformListener

from xw_interfaces.msg import TaskProgress, TaskResult
from xw_interfaces.srv import MapManage, SessionControl, SlamSessionInfo


def _quat_to_yaw(z: float, w: float) -> float:
    return math.atan2(2.0 * w * z, 1.0 - 2.0 * z * z)


class SlamSessionNode(Node):
    def __init__(self) -> None:
        super().__init__('xw_slam_session')
        self.declare_parameter('mapper_params', '')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('maps_dir', os.environ.get('XW_MAPS', '/ros2_ws/maps'))

        self._active = False
        self._command_id = ''
        self._saved_once = False
        self._start_pose: Optional[Tuple[float, float, float]] = None
        self._proc: Optional[subprocess.Popen] = None
        self._capture_thread: Optional[threading.Thread] = None
        self._lock = threading.Lock()

        latch = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self._progress_pub = self.create_publisher(TaskProgress, '/xw/task/progress', 10)
        self._result_pub = self.create_publisher(TaskResult, '/xw/task/result', 10)
        self._start_pose_pub = self.create_publisher(Pose2D, '/xw/slam/start_pose', latch)

        self.create_subscription(Bool, '/xw/slam/enable', self._on_enable, latch)
        self.create_subscription(Bool, '/xw/slam/map_saved', self._on_map_saved, 10)
        self.create_service(SessionControl, '/xw/session/slam/control', self._on_control)
        self.create_service(SlamSessionInfo, '/xw/session/slam/info', self._on_info)

        self._map_cli = self.create_client(MapManage, '/xw/map/manage')
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self._publish_start_pose(None)
        self.get_logger().info('slam session ready')

    def _params_file(self) -> str:
        configured = str(self.get_parameter('mapper_params').value or '').strip()
        if configured and Path(configured).is_file():
            return configured
        share = get_package_share_directory('xw_slam_session')
        return str(Path(share) / 'config' / 'mapper_params.yaml')

    def _on_enable(self, msg: Bool) -> None:
        self._set_active(bool(msg.data), 'enable-topic')

    def _on_control(self, req: SessionControl.Request, res: SessionControl.Response):
        self._command_id = req.command_id or 'slam-svc'
        self._set_active(bool(req.start), 'service')
        res.success = True
        res.message = 'slam started' if self._active else 'slam stopped'
        res.state = 'active' if self._active else 'idle'
        return res

    def _on_info(self, _req: SlamSessionInfo.Request, res: SlamSessionInfo.Response):
        with self._lock:
            res.active = self._active
            res.saved_once = self._saved_once
            if self._start_pose is not None:
                res.has_start_pose = True
                res.start_x, res.start_y, res.start_yaw = self._start_pose
            else:
                res.has_start_pose = False
                res.start_x = res.start_y = res.start_yaw = 0.0
            res.message = 'ok'
        return res

    def _on_map_saved(self, msg: Bool) -> None:
        if msg.data:
            with self._lock:
                self._saved_once = True
            self.get_logger().info('map save acknowledged (saved_once=true)')

    def _publish_start_pose(self, pose: Optional[Tuple[float, float, float]]) -> None:
        msg = Pose2D()
        if pose is None:
            msg.x = float('nan')
            msg.y = float('nan')
            msg.theta = float('nan')
        else:
            msg.x, msg.y, msg.theta = pose
        self._start_pose_pub.publish(msg)

    def _set_active(self, active: bool, source: str) -> None:
        if active and self._active:
            return
        if not active and not self._active and self._proc is None:
            return

        if active:
            if self._start_slam():
                with self._lock:
                    self._active = True
                    self._saved_once = False
                    self._start_pose = None
                self._publish_start_pose(None)
                self.get_logger().info(f'slam active ({source})')
                p = TaskProgress()
                p.stamp = self.get_clock().now().to_msg()
                p.command_id = self._command_id or source
                p.capability = 'slam'
                p.phase = 'active'
                self._progress_pub.publish(p)
                self._begin_pose_capture()
            else:
                self.get_logger().error('failed to start slam_toolbox')
                r = TaskResult()
                r.stamp = self.get_clock().now().to_msg()
                r.command_id = self._command_id or source
                r.capability = 'slam'
                r.code = 1
                r.message = 'slam_toolbox start failed'
                self._result_pub.publish(r)
        else:
            threading.Thread(
                target=self._stop_slam_with_autosave,
                args=(source,),
                daemon=True,
            ).start()

    def _start_slam(self) -> bool:
        self._stop_child(graceful=True)
        params = self._params_file()
        if not Path(params).is_file():
            self.get_logger().error(f'mapper params missing: {params}')
            return False
        cmd = [
            'ros2', 'run', 'slam_toolbox', 'async_slam_toolbox_node',
            '--ros-args',
            '--params-file', params,
            '-r', '__node:=slam_toolbox',
        ]
        try:
            self._proc = subprocess.Popen(
                cmd,
                preexec_fn=os.setsid,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.STDOUT,
            )
        except OSError as exc:
            self.get_logger().error(f'Popen slam failed: {exc}')
            self._proc = None
            return False
        time.sleep(1.0)
        if self._proc.poll() is not None:
            self.get_logger().error(f'slam exited early code={self._proc.returncode}')
            self._proc = None
            return False
        self.get_logger().info(f'slam_toolbox started pid={self._proc.pid} params={params}')
        return True

    def _stop_child(self, graceful: bool = True) -> None:
        proc = self._proc
        self._proc = None
        if proc is None:
            return
        try:
            pgid = os.getpgid(proc.pid)
        except ProcessLookupError:
            return
        try:
            if graceful:
                os.killpg(pgid, signal.SIGTERM)
                try:
                    proc.wait(timeout=8)
                    return
                except subprocess.TimeoutExpired:
                    pass
            os.killpg(pgid, signal.SIGKILL)
            proc.wait(timeout=3)
        except (ProcessLookupError, OSError, subprocess.TimeoutExpired):
            pass

    def _begin_pose_capture(self) -> None:
        if self._capture_thread and self._capture_thread.is_alive():
            return

        def worker() -> None:
            map_frame = str(self.get_parameter('map_frame').value)
            base_frame = str(self.get_parameter('base_frame').value)
            deadline = time.monotonic() + 30.0
            while time.monotonic() < deadline:
                with self._lock:
                    if not self._active:
                        return
                pose = self._lookup_pose(map_frame, base_frame)
                if pose is not None:
                    with self._lock:
                        self._start_pose = pose
                    self._publish_start_pose(pose)
                    self.get_logger().info(
                        f'start pose captured x={pose[0]:.3f} y={pose[1]:.3f} yaw={pose[2]:.3f}'
                    )
                    return
                time.sleep(0.5)
            self.get_logger().warn('start pose capture timed out (no charger on save)')

        self._capture_thread = threading.Thread(target=worker, daemon=True)
        self._capture_thread.start()

    def _lookup_pose(self, map_frame: str, base_frame: str) -> Optional[Tuple[float, float, float]]:
        for child in (base_frame, 'base_link', 'base_footprint'):
            try:
                tf = self._tf_buffer.lookup_transform(
                    map_frame,
                    child,
                    Time(),
                    timeout=Duration(seconds=0.2),
                )
                t = tf.transform.translation
                q = tf.transform.rotation
                return float(t.x), float(t.y), _quat_to_yaw(float(q.z), float(q.w))
            except Exception:  # noqa: BLE001
                continue
        return None

    def _stop_slam_with_autosave(self, source: str) -> None:
        with self._lock:
            need_autosave = self._active and not self._saved_once
            pose = self._start_pose
            self._active = False

        if need_autosave:
            name = 'autosave_' + datetime.now().strftime('%Y%m%d_%H%M%S')
            self.get_logger().info(f'unsaved session → autosave {name}')
            self._call_map_save(name, pose)

        self._stop_child(graceful=True)
        with self._lock:
            self._start_pose = None
            self._saved_once = False
        self._publish_start_pose(None)

        r = TaskResult()
        r.stamp = self.get_clock().now().to_msg()
        r.command_id = self._command_id or source
        r.capability = 'slam'
        r.code = 0
        r.message = 'stopped'
        self._result_pub.publish(r)
        self.get_logger().info(f'slam stopped ({source})')

    def _call_map_save(
        self, name: str, pose: Optional[Tuple[float, float, float]]
    ) -> None:
        if not self._map_cli.wait_for_service(timeout_sec=3.0):
            self.get_logger().error('map manage unavailable for autosave')
            return
        req = MapManage.Request()
        req.operation = 1
        req.map_name = name
        if pose is not None:
            req.data_json = json.dumps({
                'charger': {'x': pose[0], 'y': pose[1], 'yaw': pose[2]},
            })
        fut = self._map_cli.call_async(req)
        deadline = time.monotonic() + 30.0
        while time.monotonic() < deadline and not fut.done():
            time.sleep(0.05)
        if fut.done() and fut.result() is not None:
            res = fut.result()
            self.get_logger().info(f'autosave: {res.message}')
        else:
            self.get_logger().error('autosave timed out')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SlamSessionNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node._stop_child(graceful=False)  # noqa: SLF001
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
