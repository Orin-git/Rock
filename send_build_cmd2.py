#!/usr/bin/env python3
"""Publish one command to /xw/visual_db/build -- race-free version.

The original waited only until `get_subscription_count() > 0`, then published.
Measured 2026-09-14: on this domain the FIRST subscriber to match is
/ros2_ws/scripts/vdb_cmd_watch.py at ~3.8 s, while the build orchestrator's own
subscription does not match until ~6.4 s. The original therefore published into
the watcher only, and the build node never saw the command -- which looks
exactly like "the build node ignored the start".

This version waits until the matched-subscription count has been UNCHANGED for
STABLE_SEC seconds before publishing, and still publishes exactly once.
"""
import sys
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

if len(sys.argv) < 2:
    print('usage: send_build_cmd2.py <json-payload> [stable_sec]')
    sys.exit(2)
payload = sys.argv[1]
stable_sec = float(sys.argv[2]) if len(sys.argv) > 2 else 6.0

rclpy.init()
node = Node('vdb_cmd_sender2')
pub = node.create_publisher(String, '/xw/visual_db/build', 10)

t0 = time.time()
last = -1
stable_since = time.time()
while time.time() - t0 < 60.0:
    rclpy.spin_once(node, timeout_sec=0.2)
    c = pub.get_subscription_count()
    if c != last:
        last = c
        stable_since = time.time()
        print(f'  t={time.time()-t0:5.1f}s  matched={c}')
    if c > 0 and (time.time() - stable_since) >= stable_sec:
        break

print(f'  stable at matched={last} for {stable_sec}s (waited {time.time()-t0:.1f}s)')
if last <= 0:
    print('  NO SUBSCRIBERS MATCHED -- refusing to publish into the void')
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(1)

msg = String()
msg.data = payload
pub.publish(msg)
print(f'  published ONCE to {last} subscriber(s): {payload}')
time.sleep(1.0)
node.destroy_node()
rclpy.shutdown()
