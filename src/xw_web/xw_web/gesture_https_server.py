#!/usr/bin/env python3
"""HTTPS static server + /api/cmd_vel for Gen2 gesture teleop.

Serves xw_web/public over HTTPS so browser getUserMedia works on LAN IPs.
Publishes geometry_msgs/Twist to /xw/cmd/teleop (arbiter → safety → /cmd_vel).
"""

from __future__ import annotations

import argparse
import json
import os
import ssl
import subprocess
import sys
import threading
import traceback
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Optional

import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Bool

DEFAULT_PORT = 9443


class GestureBridge(Node):
    def __init__(self) -> None:
        super().__init__('xw_gesture_https_bridge')
        self.cmd_pub = self.create_publisher(Twist, '/xw/cmd/teleop', 10)
        self._safety_ok = True
        self._pub_lock = threading.Lock()
        self.create_subscription(Bool, 'safety_status', self._on_safety, 10)

    def _on_safety(self, msg: Bool) -> None:
        self._safety_ok = bool(msg.data)

    def publish_cmd(self, linear_x: float, angular_z: float) -> None:
        msg = Twist()
        msg.linear.x = float(linear_x)
        msg.angular.z = float(angular_z)
        with self._pub_lock:
            self.cmd_pub.publish(msg)

    @property
    def safety_ok(self) -> bool:
        return self._safety_ok


def ensure_certs(cert_dir: str) -> tuple[str, str]:
    os.makedirs(cert_dir, exist_ok=True)
    cert_file = os.path.join(cert_dir, 'cert.pem')
    key_file = os.path.join(cert_dir, 'key.pem')
    if os.path.isfile(cert_file) and os.path.isfile(key_file):
        return cert_file, key_file

    print(f'[gesture_https] generating self-signed cert in {cert_dir}')
    san = 'subjectAltName=DNS:localhost,IP:127.0.0.1'
    try:
        import socket

        host = socket.gethostname()
        ips = set()
        for info in socket.getaddrinfo(host, None):
            ip = info[4][0]
            if ':' not in ip and not ip.startswith('127.'):
                ips.add(ip)
        for ip in sorted(ips):
            san += f',IP:{ip}'
    except Exception:  # noqa: BLE001
        pass

    subprocess.check_call(
        [
            'openssl',
            'req',
            '-x509',
            '-newkey',
            'rsa:2048',
            '-sha256',
            '-days',
            '3650',
            '-nodes',
            '-keyout',
            key_file,
            '-out',
            cert_file,
            '-subj',
            '/CN=gesture-teleop/O=XiaoweiGen2/C=CN',
            '-addext',
            san,
        ]
    )
    return cert_file, key_file


def resolve_web_dir(explicit: str) -> str:
    if explicit:
        p = Path(explicit)
        if p.is_dir():
            return str(p.resolve())
    candidates = [
        Path(os.environ.get('XW_WS', '/ros2_ws')) / 'src' / 'xw_web' / 'public',
        Path(get_package_share_directory('xw_web')) / 'public',
        Path(__file__).resolve().parents[2] / 'public',
    ]
    for c in candidates:
        if c.is_dir():
            return str(c.resolve())
    raise FileNotFoundError('xw_web public directory not found')


def resolve_cert_dir(explicit: str, web_dir: str) -> str:
    if explicit:
        return os.path.abspath(explicit)
    share = Path(get_package_share_directory('xw_web'))
    for c in (
        share / 'certs' / 'gesture',
        Path(web_dir).parent / 'certs' / 'gesture',
        Path(__file__).resolve().parents[2] / 'certs' / 'gesture',
    ):
        if (c / 'cert.pem').is_file() and (c / 'key.pem').is_file():
            return str(c)
    return str(share / 'certs' / 'gesture')


def make_handler(web_dir: str, bridge: GestureBridge):
    class Handler(SimpleHTTPRequestHandler):
        # Prefer close: keep-alive + SSL + ThreadingHTTPServer is flaky on some boards.
        protocol_version = 'HTTP/1.0'

        def __init__(self, *args, **kwargs):
            super().__init__(*args, directory=web_dir, **kwargs)

        def log_message(self, fmt: str, *args) -> None:
            path = getattr(self, 'path', '') or ''
            if path.startswith('/api/cmd_vel'):
                return
            sys.stdout.write('[gesture_https] ' + (fmt % args) + '\n')
            sys.stdout.flush()

        def _send_json(self, code: int, payload: dict) -> None:
            body = json.dumps(payload).encode('utf-8')
            try:
                self.send_response(code)
                self.send_header('Content-Type', 'application/json; charset=utf-8')
                self.send_header('Content-Length', str(len(body)))
                self.send_header('Cache-Control', 'no-store')
                self.send_header('Access-Control-Allow-Origin', '*')
                self.send_header('Connection', 'close')
                self.end_headers()
                self.wfile.write(body)
                self.wfile.flush()
            finally:
                self.close_connection = True

        def do_OPTIONS(self) -> None:
            self.send_response(204)
            self.send_header('Access-Control-Allow-Origin', '*')
            self.send_header('Access-Control-Allow-Methods', 'GET, POST, OPTIONS')
            self.send_header('Access-Control-Allow-Headers', 'Content-Type')
            self.send_header('Content-Length', '0')
            self.send_header('Connection', 'close')
            self.end_headers()
            self.close_connection = True

        def do_GET(self) -> None:
            path = self.path.split('?', 1)[0]
            if path == '/api/status':
                self._send_json(
                    200,
                    {
                        'ok': True,
                        'ros': True,
                        'safety_ok': bridge.safety_ok,
                        'topic': '/xw/cmd/teleop',
                    },
                )
                return
            return super().do_GET()

        def do_POST(self) -> None:
            path = self.path.split('?', 1)[0]
            try:
                if path != '/api/cmd_vel':
                    self._send_json(404, {'ok': False, 'error': 'not found'})
                    return

                length = int(self.headers.get('Content-Length', '0') or '0')
                if length < 0 or length > 4096:
                    self._send_json(400, {'ok': False, 'error': 'bad content-length'})
                    return
                raw = self.rfile.read(length) if length > 0 else b'{}'
                try:
                    data = json.loads(raw.decode('utf-8') or '{}')
                    linear_x = float(data.get('linear_x', 0.0))
                    angular_z = float(data.get('angular_z', 0.0))
                except (ValueError, json.JSONDecodeError, UnicodeDecodeError) as exc:
                    self._send_json(400, {'ok': False, 'error': str(exc)})
                    return

                linear_x = max(-1.5, min(1.5, linear_x))
                angular_z = max(-2.5, min(2.5, angular_z))
                try:
                    bridge.publish_cmd(linear_x, angular_z)
                except Exception as exc:  # noqa: BLE001
                    sys.stderr.write(f'[gesture_https] publish failed: {exc}\n')
                    sys.stderr.flush()
                    self._send_json(500, {'ok': False, 'error': f'publish: {exc}'})
                    return

                self._send_json(
                    200, {'ok': True, 'linear_x': linear_x, 'angular_z': angular_z}
                )
            except Exception as exc:  # noqa: BLE001
                sys.stderr.write(
                    '[gesture_https] POST handler crash:\n'
                    + traceback.format_exc()
                    + '\n'
                )
                sys.stderr.flush()
                try:
                    self._send_json(500, {'ok': False, 'error': str(exc)})
                except Exception:  # noqa: BLE001
                    self.close_connection = True

    return Handler


def spin_ros(bridge: GestureBridge, stop_event: threading.Event) -> None:
    while rclpy.ok() and not stop_event.is_set():
        try:
            rclpy.spin_once(bridge, timeout_sec=0.1)
        except Exception as exc:  # noqa: BLE001
            sys.stderr.write(f'[gesture_https] spin_once: {exc}\n')
            sys.stderr.flush()


def main(argv: Optional[list] = None) -> int:
    if argv is None:
        argv = sys.argv[1:]
    # ros2 run / launch injects --ros-args …; strip before argparse.
    if '--ros-args' in argv:
        argv = argv[: argv.index('--ros-args')]
    if '--' in argv:
        argv = [a for a in argv if a != '--']

    parser = argparse.ArgumentParser(description='Gen2 gesture teleop HTTPS bridge')
    parser.add_argument('--web-dir', default='', help='Static web root (xw_web/public)')
    parser.add_argument('--port', type=int, default=DEFAULT_PORT)
    parser.add_argument('--cert-dir', default='', help='Directory for cert.pem/key.pem')
    args = parser.parse_args(argv)

    try:
        web_dir = resolve_web_dir(args.web_dir)
    except FileNotFoundError as exc:
        print(f'[gesture_https] {exc}', file=sys.stderr)
        return 1

    cert_dir = resolve_cert_dir(args.cert_dir, web_dir)
    cert_file, key_file = ensure_certs(cert_dir)

    rclpy.init()
    bridge = GestureBridge()
    stop_event = threading.Event()
    ros_thread = threading.Thread(target=spin_ros, args=(bridge, stop_event), daemon=True)
    ros_thread.start()

    handler = make_handler(web_dir, bridge)
    httpd = ThreadingHTTPServer(('0.0.0.0', args.port), handler)
    httpd.daemon_threads = True
    context = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
    context.minimum_version = ssl.TLSVersion.TLSv1_2
    context.load_cert_chain(certfile=cert_file, keyfile=key_file)
    httpd.socket = context.wrap_socket(httpd.socket, server_side=True)

    print(f'[gesture_https] serving {web_dir} on https://0.0.0.0:{args.port}')
    print('[gesture_https] publishing → /xw/cmd/teleop')
    print('[gesture_https] first visit: accept self-signed certificate in browser')
    sys.stdout.flush()

    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        stop_event.set()
        httpd.server_close()
        bridge.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
