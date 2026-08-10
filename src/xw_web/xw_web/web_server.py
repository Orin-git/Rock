#!/usr/bin/env python3
"""SPA + ROS JSON bridge (HTTP) for Xiaowei Gen2 console."""

from __future__ import annotations

import functools
import json
import os
import threading
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any, Dict, List, Optional
from urllib.parse import urlparse

import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from xw_interfaces.msg import RobotState, TaskProgress, TaskResult
from xw_interfaces.srv import MapManage, SetMode


def _state_to_dict(msg: RobotState) -> Dict[str, Any]:
    return {
        'mode': int(msg.mode),
        'mode_name': msg.mode_name,
        'run_mode': int(msg.run_mode),
        'emergency_stop': bool(msg.emergency_stop),
        'safety_ok': bool(msg.safety_ok),
        'localization_ok': bool(msg.localization_ok),
        'active_map': msg.active_map,
        'profile': msg.profile,
        'detail': msg.detail,
        'power': {
            'battery_percent': float(msg.power.battery_percent),
            'voltage': float(msg.power.voltage),
            'charging': bool(msg.power.charging),
            'docked': bool(msg.power.docked),
            'detail': msg.power.detail,
        },
    }


class BridgeNode(Node):
    def __init__(self) -> None:
        super().__init__('xw_web_bridge')
        self._lock = threading.Lock()
        self._state: Dict[str, Any] = {
            'mode': 0,
            'mode_name': 'IDLE',
            'run_mode': 1,
            'emergency_stop': False,
            'safety_ok': True,
            'localization_ok': True,
            'active_map': '',
            'profile': 'normal',
            'detail': 'waiting',
            'power': {
                'battery_percent': 0.0,
                'voltage': 0.0,
                'charging': False,
                'docked': False,
                'detail': '',
            },
        }
        self._tasks: List[str] = []

        self.create_subscription(RobotState, '/xw/robot_state', self._on_state, 10)
        self.create_subscription(TaskProgress, '/xw/task/progress', self._on_progress, 10)
        self.create_subscription(TaskResult, '/xw/task/result', self._on_result, 10)
        self._teleop_pub = self.create_publisher(Twist, '/xw/cmd/teleop', 10)
        self._set_mode = self.create_client(SetMode, '/xw/supervisor/set_mode')
        self._map_mgr = self.create_client(MapManage, '/xw/map/manage')
        domain = os.environ.get('ROS_DOMAIN_ID', '?')
        self.get_logger().info(f'web ROS bridge ready (DOMAIN={domain})')

    def _push_task(self, line: str) -> None:
        with self._lock:
            self._tasks.insert(0, line)
            del self._tasks[80:]

    def _on_state(self, msg: RobotState) -> None:
        with self._lock:
            self._state = _state_to_dict(msg)

    def _on_progress(self, msg: TaskProgress) -> None:
        self._push_task(f'[progress] {msg.capability} {msg.phase}')

    def _on_result(self, msg: TaskResult) -> None:
        self._push_task(f'[result] {msg.capability} code={msg.code} {msg.message}')

    def snapshot(self) -> Dict[str, Any]:
        with self._lock:
            return {
                'ok': True,
                'state': dict(self._state),
                'tasks': list(self._tasks[:40]),
                'ros_domain_id': os.environ.get('ROS_DOMAIN_ID', ''),
            }

    def publish_teleop(self, linear_x: float, angular_z: float) -> Dict[str, Any]:
        t = Twist()
        t.linear.x = float(linear_x)
        t.angular.z = float(angular_z)
        self._teleop_pub.publish(t)
        return {'ok': True}

    def set_mode(self, mode: int, payload: Optional[Dict[str, Any]], command_id: str) -> Dict[str, Any]:
        if not self._set_mode.wait_for_service(timeout_sec=2.0):
            return {'ok': False, 'message': 'set_mode service unavailable'}
        req = SetMode.Request()
        req.mode = int(mode)
        req.payload_json = json.dumps(payload or {})
        req.command_id = command_id or f'web-{self.get_clock().now().nanoseconds}'
        fut = self._set_mode.call_async(req)
        for _ in range(60):
            if fut.done():
                break
            threading.Event().wait(0.05)
        if not fut.done() or fut.result() is None:
            return {'ok': False, 'message': 'set_mode timeout'}
        res = fut.result()
        self._push_task(f'[set_mode] {res.message} active={res.active_mode}')
        return {
            'ok': bool(res.success),
            'message': res.message,
            'active_mode': int(res.active_mode),
        }

    def map_manage(
        self, operation: int, map_name: str = '', new_name: str = '', data_json: str = ''
    ) -> Dict[str, Any]:
        if not self._map_mgr.wait_for_service(timeout_sec=2.0):
            return {'ok': False, 'message': 'map service unavailable'}
        req = MapManage.Request()
        req.operation = int(operation)
        req.map_name = map_name
        req.new_name = new_name
        req.data_json = data_json
        fut = self._map_mgr.call_async(req)
        for _ in range(60):
            if fut.done():
                break
            threading.Event().wait(0.05)
        if not fut.done() or fut.result() is None:
            return {'ok': False, 'message': 'map timeout'}
        res = fut.result()
        self._push_task(f'[map] op={operation} {res.message}')
        return {
            'ok': bool(res.success),
            'message': res.message,
            'map_list': list(res.map_list),
            'data_json': res.data_json,
        }


class ApiHandler(SimpleHTTPRequestHandler):
    bridge: Optional[BridgeNode] = None

    def end_headers(self) -> None:
        self.send_header('Cache-Control', 'no-store')
        self.send_header('Access-Control-Allow-Origin', '*')
        self.send_header('Access-Control-Allow-Methods', 'GET, POST, OPTIONS')
        self.send_header('Access-Control-Allow-Headers', 'Content-Type')
        super().end_headers()

    def do_OPTIONS(self) -> None:  # noqa: N802
        self.send_response(204)
        self.end_headers()

    def log_message(self, fmt: str, *args) -> None:
        return

    def _json(self, code: int, obj: Dict[str, Any]) -> None:
        body = json.dumps(obj, ensure_ascii=False).encode('utf-8')
        self.send_response(code)
        self.send_header('Content-Type', 'application/json; charset=utf-8')
        self.send_header('Content-Length', str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _read_json(self) -> Dict[str, Any]:
        n = int(self.headers.get('Content-Length') or 0)
        if n <= 0:
            return {}
        raw = self.rfile.read(n).decode('utf-8', errors='replace')
        try:
            return json.loads(raw) if raw else {}
        except json.JSONDecodeError:
            return {}

    def do_GET(self) -> None:  # noqa: N802
        path = urlparse(self.path).path
        if path == '/api/health':
            return self._json(200, {'ok': True, 'bridge': self.bridge is not None})
        if path == '/api/state':
            if not self.bridge:
                return self._json(503, {'ok': False, 'message': 'bridge offline'})
            return self._json(200, self.bridge.snapshot())
        return super().do_GET()

    def do_POST(self) -> None:  # noqa: N802
        if not self.bridge:
            return self._json(503, {'ok': False, 'message': 'bridge offline'})
        path = urlparse(self.path).path
        data = self._read_json()
        if path == '/api/teleop':
            return self._json(
                200,
                self.bridge.publish_teleop(
                    float(data.get('linear_x', 0.0)),
                    float(data.get('angular_z', 0.0)),
                ),
            )
        if path == '/api/set_mode':
            payload = data.get('payload') if isinstance(data.get('payload'), dict) else {}
            return self._json(
                200,
                self.bridge.set_mode(
                    int(data.get('mode', 0)),
                    payload,
                    str(data.get('command_id') or ''),
                ),
            )
        if path == '/api/map':
            return self._json(
                200,
                self.bridge.map_manage(
                    int(data.get('operation', 2)),
                    str(data.get('map_name') or ''),
                    str(data.get('new_name') or ''),
                    str(data.get('data_json') or ''),
                ),
            )
        return self._json(404, {'ok': False, 'message': 'not found'})


def _resolve_web_root(param_root: str) -> Path:
    candidates = [
        Path(param_root) if param_root else None,
        Path('/ros2_ws/install/xw_web/share/xw_web/public'),
        Path(os.environ.get('XW_WS', '/ros2_ws')) / 'src' / 'xw_web' / 'public',
        Path(__file__).resolve().parents[2] / 'public',
    ]
    for c in candidates:
        if c and c.is_dir():
            return c.resolve()
    return Path('/ros2_ws/src/xw_web/public')


def main(args=None) -> None:
    rclpy.init(args=args)
    bridge = BridgeNode()
    bridge.declare_parameter('port', 9000)
    bridge.declare_parameter('web_root', '')

    port = int(bridge.get_parameter('port').value)
    root = _resolve_web_root(str(bridge.get_parameter('web_root').value))
    bridge.get_logger().info(f'HTTP SPA+API :{port} root={root}')

    ApiHandler.bridge = bridge
    handler = functools.partial(ApiHandler, directory=str(root))
    httpd = ThreadingHTTPServer(('0.0.0.0', port), handler)
    httpd.daemon_threads = True
    threading.Thread(target=httpd.serve_forever, daemon=True).start()

    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(bridge)
    try:
        executor.spin()
    finally:
        httpd.shutdown()
        executor.shutdown()
        bridge.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
