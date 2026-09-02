#!/usr/bin/env python3
"""Navigation session: bring up Nav2, single/multi goal, TaskProgress/Result."""

from __future__ import annotations

import json
import math
import os
import signal
import subprocess
import threading
import time
from pathlib import Path
from typing import Any, Dict, List, Optional

import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String

from xw_interfaces.msg import TaskProgress, TaskResult
from xw_interfaces.srv import SessionControl, WaypointManage


def _yaw_to_quat(yaw: float):
    from geometry_msgs.msg import Quaternion

    q = Quaternion()
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q


def _point_list_name(map_name: str) -> str:
    name = (map_name or '').strip()
    if name.endswith('_pointList'):
        return name
    return f'{name}_pointList'


class NavSessionNode(Node):
    def __init__(self) -> None:
        super().__init__('xw_nav_session')
        self.declare_parameter('maps_dir', os.environ.get('XW_MAPS', '/ros2_ws/maps'))
        self.declare_parameter('nav2_params', '')
        self.declare_parameter('nav2_launch_pkg', 'xw_nav_session')
        self.declare_parameter('use_nav2', True)

        self._cb = ReentrantCallbackGroup()
        self._lock = threading.Lock()
        self._active = False
        self._command_id = ''
        self._map_name = ''
        self._proc: Optional[subprocess.Popen] = None
        self._nav_goal_handle = None
        self._patrol_thread: Optional[threading.Thread] = None
        self._patrol_stop = threading.Event()
        self._patrol_active = False
        self._follow_en = False
        self._recharge_en = False

        latch = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self._progress_pub = self.create_publisher(TaskProgress, '/xw/task/progress', 10)
        self._result_pub = self.create_publisher(TaskResult, '/xw/task/result', 10)

        self.create_subscription(Bool, '/xw/nav/enable', self._on_enable, latch, callback_group=self._cb)
        self.create_subscription(PoseStamped, '/xw/goal_pose', self._on_goal, 10, callback_group=self._cb)
        self.create_subscription(String, '/xw/nav/patrol_cmd', self._on_patrol_cmd, 10, callback_group=self._cb)
        self.create_subscription(Bool, '/xw/nav/cancel', self._on_cancel, 10, callback_group=self._cb)
        self.create_subscription(
            Bool, '/xw/follow/enable', self._on_follow_enable, latch, callback_group=self._cb
        )
        self.create_subscription(
            Bool, '/xw/recharge/enable', self._on_recharge_enable, latch, callback_group=self._cb
        )
        self.create_service(SessionControl, '/xw/session/nav/control', self._on_control, callback_group=self._cb)

        self._wp_cli = self.create_client(WaypointManage, '/xw/map/waypoint', callback_group=self._cb)
        self._nav_client = ActionClient(
            self, NavigateToPose, 'navigate_to_pose', callback_group=self._cb
        )

        # map_name from supervisor payload is not on enable topic — publish side-channel
        self.create_subscription(
            String, '/xw/nav/map_name', self._on_map_name, latch, callback_group=self._cb
        )

        self.get_logger().info('nav session ready')

    def _params_file(self) -> str:
        configured = str(self.get_parameter('nav2_params').value or '').strip()
        if configured and Path(configured).is_file():
            return configured
        share = get_package_share_directory('xw_nav_session')
        return str(Path(share) / 'config' / 'nav2_params.yaml')

    def _maps_dir(self) -> Path:
        return Path(str(self.get_parameter('maps_dir').value))

    def _map_yaml_path(self, map_name: str) -> Optional[Path]:
        name = (map_name or '').strip()
        if not name:
            return None
        p = self._maps_dir() / f'{name}.yaml'
        return p if p.is_file() else None

    def _on_map_name(self, msg: String) -> None:
        name = (msg.data or '').strip()
        if name:
            with self._lock:
                self._map_name = name
            self.get_logger().info(f'nav map_name={name}')

    def _on_enable(self, msg: Bool) -> None:
        self._set_active(bool(msg.data), 'enable-topic')

    def _on_control(self, req: SessionControl.Request, res: SessionControl.Response):
        self._command_id = req.command_id or 'nav-svc'
        self._set_active(bool(req.start), 'service')
        res.success = True
        res.message = 'nav started' if self._active else 'nav stopped'
        res.state = 'active' if self._active else 'idle'
        return res

    def _on_cancel(self, msg: Bool) -> None:
        if msg.data:
            # Soft cancel only — never stop Nav2 process here
            self._cancel_navigation('cancel-topic')

    def _on_follow_enable(self, msg: Bool) -> None:
        """Follow task preempts point/patrol; Nav2 process stays up."""
        en = bool(msg.data)
        was = self._follow_en
        self._follow_en = en
        if en and not was:
            self._cancel_navigation('follow-preempt')
            self.get_logger().info('follow on → cancelled point/patrol (Nav2 kept)')
        elif not en and was:
            self.get_logger().info('follow off → point/patrol accepted again')

    def _on_recharge_enable(self, msg: Bool) -> None:
        en = bool(msg.data)
        was = self._recharge_en
        self._recharge_en = en
        if en and not was:
            self._cancel_navigation('recharge-preempt')
            self.get_logger().info('recharge on → cancelled point/patrol (Nav2 kept)')
        elif not en and was:
            self.get_logger().info('recharge off → point/patrol accepted again')

    def _set_active(self, active: bool, source: str) -> None:
        if active and self._active:
            return
        if not active and not self._active and self._proc is None:
            return

        if active:
            if self._start_nav2():
                with self._lock:
                    self._active = True
                self._emit_progress('active', source)
                self.get_logger().info(f'nav active ({source}) map={self._map_name}')
            else:
                self._emit_result(1, 'Nav2 start failed', source)
        else:
            self._cancel_navigation(source)
            self._stop_nav2()
            with self._lock:
                self._active = False
            self._emit_result(0, 'stopped', source)
            self.get_logger().info(f'nav stopped ({source})')

    def _start_nav2(self) -> bool:
        if not bool(self.get_parameter('use_nav2').value):
            self.get_logger().warn('use_nav2=false — session only (no Nav2 process)')
            return True

        self._stop_nav2()
        with self._lock:
            map_name = self._map_name
        yaml_path = self._map_yaml_path(map_name)
        if yaml_path is None:
            self.get_logger().error(
                f'map yaml missing for map_name={map_name!r} under {self._maps_dir()}'
            )
            return False

        params = self._params_file()
        if not Path(params).is_file():
            self.get_logger().error(f'nav2 params missing: {params}')
            return False

        cmd = [
            'ros2', 'launch', 'xw_nav_session', 'nav2.launch.py',
            f'map:={yaml_path}',
            f'params_file:={params}',
            'use_sim_time:=false',
            'autostart:=true',
        ]
        try:
            self._proc = subprocess.Popen(
                cmd,
                preexec_fn=os.setsid,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.STDOUT,
            )
        except OSError as exc:
            self.get_logger().error(f'Popen nav2 failed: {exc}')
            self._proc = None
            return False

        time.sleep(3.0)
        if self._proc.poll() is not None:
            self.get_logger().error(f'nav2 exited early code={self._proc.returncode}')
            self._proc = None
            return False
        self.get_logger().info(f'nav2 started pid={self._proc.pid} map={yaml_path}')
        return True

    def _stop_nav2(self) -> None:
        proc = self._proc
        self._proc = None
        if proc is None:
            return
        try:
            pgid = os.getpgid(proc.pid)
        except ProcessLookupError:
            return
        try:
            os.killpg(pgid, signal.SIGTERM)
            try:
                proc.wait(timeout=12)
                return
            except subprocess.TimeoutExpired:
                pass
            os.killpg(pgid, signal.SIGKILL)
            proc.wait(timeout=3)
        except (ProcessLookupError, OSError, subprocess.TimeoutExpired):
            pass

    def _on_goal(self, msg: PoseStamped) -> None:
        if not self._active:
            self.get_logger().warn('goal ignored (nav inactive)')
            return
        if self._follow_en:
            self.get_logger().warn('goal rejected (follow active — stop follow first)')
            self._emit_result(1, 'rejected: follow active', 'goal')
            return
        if self._recharge_en:
            self.get_logger().warn('goal rejected (recharge active)')
            self._emit_result(1, 'rejected: recharge active', 'goal')
            return
        # Single goal preempts patrol / prior point goal; must clear _patrol_stop
        # or _navigate_blocking cancels the new goal immediately (~100ms).
        self._cancel_navigation('goal-preempt')
        self._patrol_stop.clear()
        self._command_id = self._command_id or 'goal'
        x = float(msg.pose.position.x)
        y = float(msg.pose.position.y)
        self._emit_progress('goal_accepted', self._command_id, {'x': x, 'y': y})
        threading.Thread(
            target=self._run_single_goal,
            args=(msg, self._command_id),
            daemon=True,
        ).start()

    def _on_patrol_cmd(self, msg: String) -> None:
        if not self._active:
            self.get_logger().warn('patrol ignored (nav inactive)')
            return
        if self._follow_en:
            self.get_logger().warn('patrol rejected (follow active — stop follow first)')
            self._emit_result(1, 'rejected: follow active', 'patrol')
            return
        if self._recharge_en:
            self.get_logger().warn('patrol rejected (recharge active)')
            self._emit_result(1, 'rejected: recharge active', 'patrol')
            return
        try:
            payload = json.loads(msg.data or '{}')
        except json.JSONDecodeError:
            self.get_logger().error('patrol_cmd JSON invalid')
            return
        action = str(payload.get('action') or 'start').lower()
        if action in ('stop', 'cancel'):
            self._cancel_navigation('patrol-stop')
            self._emit_result(0, 'patrol stopped', self._command_id)
            return

        map_name = str(payload.get('map_name') or self._map_name).strip()
        loop = bool(payload.get('loop', False))
        names = payload.get('waypoints')  # optional ordered names
        self._command_id = str(payload.get('command_id') or 'patrol')
        poses = self._load_waypoint_poses(map_name, names)
        if not poses:
            self._emit_result(1, f'no waypoints for {map_name}', self._command_id)
            return
        self._stop_patrol_loop()
        self._patrol_stop.clear()
        self._patrol_active = True
        self._patrol_thread = threading.Thread(
            target=self._run_patrol,
            args=(poses, loop, self._command_id),
            daemon=True,
        )
        self._patrol_thread.start()

    def _load_waypoint_poses(
        self, map_name: str, names: Optional[List[str]]
    ) -> List[PoseStamped]:
        if not self._wp_cli.wait_for_service(timeout_sec=2.0):
            self.get_logger().error('waypoint service unavailable')
            return []
        req = WaypointManage.Request()
        req.operation = 2
        req.map_name = map_name
        fut = self._wp_cli.call_async(req)
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline and not fut.done():
            time.sleep(0.05)
        if not fut.done() or fut.result() is None:
            return []
        res = fut.result()
        try:
            data = json.loads(res.data_json or '{}')
        except json.JSONDecodeError:
            return []
        wps: List[Dict[str, Any]] = list(data.get('waypoints') or [])
        if data.get('charger') and isinstance(data['charger'], dict):
            c = data['charger']
            wps.insert(0, {
                'name': 'charger',
                'x': c.get('x'),
                'y': c.get('y'),
                'yaw': c.get('yaw', c.get('theta', 0.0)),
            })
        if names:
            by_name = {str(w.get('name') or w.get('id') or ''): w for w in wps}
            ordered = [by_name[n] for n in names if n in by_name]
        else:
            ordered = wps

        poses: List[PoseStamped] = []
        for w in ordered:
            try:
                x = float(w['x'])
                y = float(w['y'])
            except (KeyError, TypeError, ValueError):
                continue
            yaw = float(w.get('yaw', w.get('theta', 0.0)) or 0.0)
            ps = PoseStamped()
            ps.header.frame_id = 'map'
            ps.pose.position.x = x
            ps.pose.position.y = y
            ps.pose.orientation = _yaw_to_quat(yaw)
            poses.append(ps)
        return poses

    def _run_patrol(self, poses: List[PoseStamped], loop: bool, command_id: str) -> None:
        self._emit_progress('patrol_start', command_id, {'count': len(poses), 'loop': loop})
        idx = 0
        while not self._patrol_stop.is_set() and self._active:
            pose = poses[idx % len(poses)]
            pose.header.stamp = self.get_clock().now().to_msg()
            self._emit_progress(
                'patrol_waypoint',
                command_id,
                {'index': idx % len(poses), 'x': pose.pose.position.x, 'y': pose.pose.position.y},
            )
            ok = self._navigate_blocking(pose, command_id)
            if self._patrol_stop.is_set():
                break
            if not ok:
                self._emit_result(1, f'patrol failed at index {idx % len(poses)}', command_id)
                self._patrol_active = False
                return
            idx += 1
            if not loop and idx >= len(poses):
                self._emit_result(0, 'patrol complete', command_id)
                self._patrol_active = False
                return
        self._patrol_active = False

    def _run_single_goal(self, pose: PoseStamped, command_id: str) -> None:
        pose.header.stamp = self.get_clock().now().to_msg()
        if not pose.header.frame_id:
            pose.header.frame_id = 'map'
        ok = self._navigate_blocking(pose, command_id)
        if ok:
            self._emit_result(0, 'goal succeeded', command_id, {
                'x': pose.pose.position.x,
                'y': pose.pose.position.y,
            })
        else:
            self._emit_result(1, 'goal failed/aborted', command_id)

    def _navigate_blocking(self, pose: PoseStamped, command_id: str) -> bool:
        if not bool(self.get_parameter('use_nav2').value):
            self.get_logger().warn('use_nav2=false — simulating success')
            time.sleep(0.2)
            return True

        if not self._nav_client.wait_for_server(timeout_sec=15.0):
            self.get_logger().error('navigate_to_pose action server not ready')
            return False

        goal = NavigateToPose.Goal()
        goal.pose = pose
        send_fut = self._nav_client.send_goal_async(goal)
        deadline = time.monotonic() + 10.0
        while time.monotonic() < deadline and not send_fut.done():
            time.sleep(0.05)
        if not send_fut.done():
            self.get_logger().error('send_goal timed out')
            return False
        gh = send_fut.result()
        # Cancel-then-send race: the server may still be finalizing the
        # previous goal and reject the new one. Retry shortly before failing.
        if gh is None or not gh.accepted:
            for _attempt in range(2):
                self.get_logger().warn('goal rejected, retrying')
                time.sleep(0.4)
                send_fut = self._nav_client.send_goal_async(goal)
                deadline = time.monotonic() + 10.0
                while time.monotonic() < deadline and not send_fut.done():
                    time.sleep(0.05)
                if not send_fut.done():
                    continue
                gh = send_fut.result()
                if gh is not None and gh.accepted:
                    break
            if gh is None or not gh.accepted:
                self.get_logger().warn('goal rejected (after retries)')
                return False

        with self._lock:
            self._nav_goal_handle = gh

        self._emit_progress('executing', command_id)
        result_fut = gh.get_result_async()
        while not result_fut.done():
            if self._patrol_stop.is_set() or not self._active:
                try:
                    gh.cancel_goal_async()
                except Exception:  # noqa: BLE001
                    pass
                return False
            time.sleep(0.1)

        with self._lock:
            self._nav_goal_handle = None

        result = result_fut.result()
        # status 4 = SUCCEEDED in action_msgs
        status = int(getattr(result, 'status', 0))
        return status == 4

    def _stop_patrol_loop(self) -> None:
        self._patrol_stop.set()
        self._patrol_active = False

    def _cancel_navigation(self, source: str) -> None:
        self._stop_patrol_loop()
        with self._lock:
            gh = self._nav_goal_handle
            self._nav_goal_handle = None
        if gh is not None:
            try:
                gh.cancel_goal_async()
            except Exception as exc:  # noqa: BLE001
                self.get_logger().warn(f'cancel failed: {exc}')
        self.get_logger().info(f'nav cancel ({source})')

    def _emit_progress(self, phase: str, command_id: str = '', data: Optional[dict] = None) -> None:
        p = TaskProgress()
        p.stamp = self.get_clock().now().to_msg()
        p.command_id = command_id or self._command_id
        p.capability = 'nav'
        p.phase = phase
        if data:
            p.detail = json.dumps(data, ensure_ascii=False)
        self._progress_pub.publish(p)

    def _emit_result(
        self,
        code: int,
        message: str,
        command_id: str = '',
        data: Optional[dict] = None,
    ) -> None:
        r = TaskResult()
        r.stamp = self.get_clock().now().to_msg()
        r.command_id = command_id or self._command_id
        r.capability = 'nav'
        r.code = int(code)
        r.message = message
        if data:
            r.data_json = json.dumps(data, ensure_ascii=False)
        self._result_pub.publish(r)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = NavSessionNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node._cancel_navigation('shutdown')  # noqa: SLF001
        node._stop_nav2()  # noqa: SLF001
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
