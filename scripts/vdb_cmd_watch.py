#!/usr/bin/env python3
"""Passive watcher for /xw/visual_db/build — logs every command, acts on none.

Deliberately does NOT start or stop anything: its only job is to fingerprint
whatever re-launched the build session at 09:22:47 on 2026-09-10.
"""
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

LOG = '/ros2_ws/log/vdb_cmd_watch.log'


def stamp():
    return time.strftime('%Y-%m-%d %H:%M:%S')


def main():
    rclpy.init()
    node = Node('vdb_cmd_watch')
    with open(LOG, 'a', buffering=1, encoding='utf-8') as fh:
        def cb(msg):
            fh.write(f'[{stamp()}] PAYLOAD {msg.data}\n')

        node.create_subscription(String, '/xw/visual_db/build', cb, 10)
        fh.write(f'[{stamp()}] WATCHER ARMED\n')
        try:
            rclpy.spin(node)
        except Exception as exc:  # noqa: BLE001
            fh.write(f'[{stamp()}] WATCHER EXIT {exc!r}\n')


main()
