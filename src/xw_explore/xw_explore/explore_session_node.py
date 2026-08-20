#!/usr/bin/env python3
"""Explore session: while MAPPING, bring up Nav2 (no AMCL) + frontier node.

Orthogonal latch /xw/explore/enable (requires supervisor mode=MAPPING).
Finished → optional map save → clear latch via /xw/explore/request_disable.
"""

from __future__ import annotations

import json
import os
import signal
import subprocess
import threading
import time
from pathlib import Path
from typing import Optional

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String

from xw_interfaces.msg import TaskProgress, TaskResult
from xw_interfaces.srv import MapManage


class ExploreSessionNode(Node):
    def __init__(self) -> None:
        super().__init__('xw_explore_session')
        self.declare_parameter('nav2_params', '')
        self.declare_parameter('nav2_ready_timeout_sec', 35.0)
        self.declare_parameter('auto_save_on_finish', True)
        self.declare_parameter('maps_dir', os.environ.get('XW_MAPS', '/ros2_ws/maps'))

        self._cb = ReentrantCallbackGroup()
        self._lock = threading.Lock()
        self._enabled = False
        self._map_name = ''
        self._phase = 'idle'
        self._message = '待命'
        self._iteration = 0
        self._nav_proc: Optional[subprocess.Popen] = None
        self._frontier_proc: Optional[subprocess.Popen] = None
        self._finish_handled = False

        latch = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self._progress_pub = self.create_publisher(TaskProgress, '/xw/task/progress', 10)
        self._result_pub = self.create_publisher(TaskResult, '/xw/task/result', 10)
        self._status_pub = self.create_publisher(String, '/xw/explore/status', latch)
        self._request_disable_pub = self.create_publisher(Bool, '/xw/explore/request_disable', 10)

        self.create_subscription(Bool, '/xw/explore/enable', self._on_enable, latch, callback_group=self._cb)
        self.create_subscription(String, '/xw/explore/map_name', self._on_map_name, latch, callback_group=self._cb)
        self.create_subscription(Bool, '/xw/explore/finished', self._on_finished, 10, callback_group=self._cb)

        self._map_cli = self.create_client(MapManage, '/xw/map/manage', callback_group=self._cb)
        self.create_timer(1.0, self._publish_status, callback_group=self._cb)
        self._publish_status()
        self.get_logger().info('explore session ready')

    def _params_file(self) -> str:
        configured = str(self.get_parameter('nav2_params').value or '').strip()
        if configured and Path(configured).is_file():
            return configured
        share = get_package_share_directory('xw_explore')
        return str(Path(share) / 'config' / 'nav2_params_explore.yaml')

    def _on_map_name(self, msg: String) -> None:
        name = (msg.data or '').strip()
        with self._lock:
            if name:
                self._map_name = name

    def _on_enable(self, msg: Bool) -> None:
        want = bool(msg.data)
        with self._lock:
            already = self._enabled
        if want and already:
            return
        if not want and not already and self._nav_proc is None and self._frontier_proc is None:
            return
        if want:
            threading.Thread(target=self._start, daemon=True).start()
        else:
            threading.Thread(target=self._stop, args=('disable', False), daemon=True).start()

    def _on_finished(self, msg: Bool) -> None:
        if not msg.data:
            return
        with self._lock:
            if self._finish_handled or not self._enabled:
                return
            self._finish_handled = True
        self.get_logger().info('frontier finished → auto stop/save')
        threading.Thread(target=self._stop, args=('finished', True), daemon=True).start()

    def _set_phase(self, phase: str, message: str) -> None:
        with self._lock:
            self._phase = phase
            self._message = message
        self._publish_status()
        p = TaskProgress()
        p.stamp = self.get_clock().now().to_msg()
        p.command_id = 'explore'
        p.capability = 'explore'
        p.phase = phase
        p.detail = message
        self._progress_pub.publish(p)

    def _publish_status(self) -> None:
        with self._lock:
            body = {
                'enabled': self._enabled,
                'phase': self._phase,
                'message': self._message,
                'map_name': self._map_name,
                'iteration': self._iteration,
                'active': self._enabled and self._phase in ('starting', 'exploring'),
            }
        msg = String()
        msg.data = json.dumps(body, ensure_ascii=False)
        self._status_pub.publish(msg)

    def _emit_result(self, code: int, message: str) -> None:
        r = TaskResult()
        r.stamp = self.get_clock().now().to_msg()
        r.command_id = 'explore'
        r.capability = 'explore'
        r.code = int(code)
        r.message = message
        self._result_pub.publish(r)

    def _start(self) -> None:
        with self._lock:
            self._enabled = True
            self._finish_handled = False
            map_name = self._map_name
        self._set_phase('starting', f'启动探索栈… ({map_name or "未命名"})')

        if not self._start_nav2():
            self._set_phase('fail', '探索 Nav2 启动失败')
            self._emit_result(1, 'explore nav2 start failed')
            with self._lock:
                self._enabled = False
            self._request_disable_pub.publish(Bool(data=True))
            return

        # Wait for navigate_to_pose action server
        timeout = float(self.get_parameter('nav2_ready_timeout_sec').value)
        deadline = time.time() + timeout
        while time.time() < deadline:
            with self._lock:
                if not self._enabled:
                    self._stop_children()
                    return
            if self._nav_proc is not None and self._nav_proc.poll() is not None:
                self._set_phase('fail', '探索 Nav2 进程退出')
                self._emit_result(1, 'explore nav2 exited')
                with self._lock:
                    self._enabled = False
                self._request_disable_pub.publish(Bool(data=True))
                return
            # Heuristic: give lifecycle time; frontier waits on action client too
            if time.time() > deadline - timeout + 8.0:
                break
            time.sleep(0.5)

        if not self._start_frontier():
            self._stop_children()
            self._set_phase('fail', 'Frontier 节点启动失败')
            self._emit_result(1, 'frontier start failed')
            with self._lock:
                self._enabled = False
            self._request_disable_pub.publish(Bool(data=True))
            return

        self._set_phase('exploring', '自主探索中')
        self.get_logger().info('explore stack running')

    def _stop(self, reason: str, do_save: bool) -> None:
        with self._lock:
            map_name = self._map_name
            was = self._enabled
            self._enabled = False

        if was:
            self._set_phase('stopping', '停止探索…')
        self._stop_children()

        saved_msg = ''
        auto_save = bool(self.get_parameter('auto_save_on_finish').value)
        if do_save and auto_save and map_name:
            ok, saved_msg = self._save_map(map_name)
            if ok:
                self._set_phase('success', f'探索完成，已保存 {map_name}')
                self._emit_result(0, f'explore finished, saved {map_name}')
            else:
                self._set_phase('fail', f'探索结束但保存失败: {saved_msg}')
                self._emit_result(1, f'save failed: {saved_msg}')
        elif reason == 'finished':
            self._set_phase('success', '探索完成')
            self._emit_result(0, 'explore finished')
        else:
            self._set_phase('idle', '待命')
            self._emit_result(2 if reason == 'disable' else 0, f'explore stopped ({reason})')

        if reason == 'finished':
            # Ask supervisor to clear the orthogonal latch
            self._request_disable_pub.publish(Bool(data=True))

        self.get_logger().info(f'explore stopped reason={reason} save={do_save} map={map_name}')

    def _start_nav2(self) -> bool:
        self._stop_nav2()
        params = self._params_file()
        if not Path(params).is_file():
            self.get_logger().error(f'explore nav2 params missing: {params}')
            return False
        share = get_package_share_directory('xw_explore')
        bt = str(Path(share) / 'behavior_trees' / 'navigate_to_pose_explore.xml')
        cmd = [
            'ros2', 'launch', 'xw_explore', 'explore_nav2.launch.py',
            f'params_file:={params}',
            f'default_bt_xml:={bt}',
            'use_sim_time:=false',
            'autostart:=true',
        ]
        try:
            self._nav_proc = subprocess.Popen(
                cmd,
                preexec_fn=os.setsid,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.STDOUT,
            )
        except OSError as exc:
            self.get_logger().error(f'Popen explore nav2 failed: {exc}')
            self._nav_proc = None
            return False
        time.sleep(3.0)
        if self._nav_proc.poll() is not None:
            self.get_logger().error(f'explore nav2 exited early code={self._nav_proc.returncode}')
            self._nav_proc = None
            return False
        self.get_logger().info(f'explore nav2 started pid={self._nav_proc.pid}')
        return True

    def _start_frontier(self) -> bool:
        self._stop_frontier()
        cmd = [
            'ros2', 'run', 'xw_explore', 'explore_node',
            '--ros-args',
            '-r', '__node:=xw_explore_frontier',
            '-p', 'robot_frame:=base_link',
            '-p', 'cmd_vel_topic:=/xw/cmd/nav',
        ]
        try:
            self._frontier_proc = subprocess.Popen(
                cmd,
                preexec_fn=os.setsid,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.STDOUT,
            )
        except OSError as exc:
            self.get_logger().error(f'Popen frontier failed: {exc}')
            self._frontier_proc = None
            return False
        time.sleep(1.0)
        if self._frontier_proc.poll() is not None:
            self.get_logger().error(f'frontier exited early code={self._frontier_proc.returncode}')
            self._frontier_proc = None
            return False
        self.get_logger().info(f'frontier started pid={self._frontier_proc.pid}')
        return True

    def _stop_children(self) -> None:
        self._stop_frontier()
        self._stop_nav2()

    def _kill_pg(self, proc: Optional[subprocess.Popen], label: str) -> None:
        if proc is None:
            return
        try:
            pgid = os.getpgid(proc.pid)
        except ProcessLookupError:
            return
        try:
            os.killpg(pgid, signal.SIGTERM)
            try:
                proc.wait(timeout=10)
                return
            except subprocess.TimeoutExpired:
                pass
            os.killpg(pgid, signal.SIGKILL)
            proc.wait(timeout=3)
        except (ProcessLookupError, OSError, subprocess.TimeoutExpired) as exc:
            self.get_logger().warn(f'stop {label}: {exc}')

    def _stop_nav2(self) -> None:
        proc = self._nav_proc
        self._nav_proc = None
        self._kill_pg(proc, 'nav2')

    def _stop_frontier(self) -> None:
        proc = self._frontier_proc
        self._frontier_proc = None
        self._kill_pg(proc, 'frontier')

    def _save_map(self, map_name: str) -> tuple[bool, str]:
        if not self._map_cli.wait_for_service(timeout_sec=3.0):
            return False, 'map manage unavailable'
        req = MapManage.Request()
        req.operation = 1  # SAVE
        req.map_name = map_name
        fut = self._map_cli.call_async(req)
        deadline = time.time() + 60.0
        while time.time() < deadline:
            if fut.done():
                break
            time.sleep(0.05)
        if not fut.done() or fut.result() is None:
            return False, 'save timeout'
        res = fut.result()
        ok = bool(getattr(res, 'success', False))
        return ok, str(getattr(res, 'message', '') or '')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ExploreSessionNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node._stop_children()  # noqa: SLF001
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
