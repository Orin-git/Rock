#!/usr/bin/env python3
"""Append every /xw/visual_db/build_status state change to a line-buffered trace.

The node's own stdout log is block-buffered when redirected to a file, so it
lags by ~8 KB. This subscribes to the latched status topic instead and flushes
per line, giving a live state timeline with durations for the acceptance table.

Read-only: traces, never commands.
"""
import json
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy,
)
from std_msgs.msg import String

LOG = '/ros2_ws/log/build_status_trace.log'
QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,  # must not be volatile: the
    history=HistoryPolicy.KEEP_LAST,              # topic is latched
    depth=1,
)

rclpy.init()
node = Node('build_status_trace')
fh = open(LOG, 'a', buffering=1, encoding='utf-8')
fh.write(f'# trace armed {time.strftime("%F %T")}\n')

last_key = None
last_state = None
last_state_at = None


def cb(msg):
    global last_key, last_state, last_state_at
    try:
        d = json.loads(msg.data)
    except Exception:  # noqa: BLE001
        return
    state = str(d.get('state'))
    message = str(d.get('message') or '')
    key = (state, message)
    if key == last_key:
        return
    last_key = key
    now = time.time()
    held = f'{now - last_state_at:6.1f}s' if last_state_at else '   ---  '
    if state != last_state:
        last_state = state
        last_state_at = now
    prog = d.get('progress') or {}
    cs = d.get('completion_summary') or {}
    fh.write(
        f'[{time.strftime("%F %T")}] ({held}) state={state:<12} '
        f'msg={message:<44} nav={prog.get("reached")}/{prog.get("planned")} '
        f'new_cells={prog.get("new_cells")} covered={cs.get("covered")} '
        f'eligible={cs.get("eligible")} spatial={cs.get("spatial_coverage_ratio")}\n'
    )


node.create_subscription(String, '/xw/visual_db/build_status', cb, QOS)
fh.write(f'# subscribed\n')
rclpy.spin(node)
