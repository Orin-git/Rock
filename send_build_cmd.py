#!/usr/bin/env python3
"""Publish one command to /xw/visual_db/build.

rclpy directly, not `ros2 topic pub`: nested ssh+docker quoting mangled the JSON
payload more than once. Default QoS matches the orchestrator's subscription
(`build_orchestrator_node.py:218`, plain depth 10).
"""
import sys
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

if len(sys.argv) < 2:
    print('usage: send_build_cmd.py <json-payload>')
    sys.exit(2)
payload = sys.argv[1]

rclpy.init()
node = Node('vdb_cmd_sender')
pub = node.create_publisher(String, '/xw/visual_db/build', 10)

# Publish only once DDS has actually matched the subscribers. Sleeping a fixed
# 1.5 s and firing "worked" only because the domain happened to be small; with
# ~60 nodes the match takes longer and the message goes into the void — which
# then looks exactly like "the build node ignored the command".
t0 = time.time()
while pub.get_subscription_count() == 0 and time.time() - t0 < 20.0:
    rclpy.spin_once(node, timeout_sec=0.2)
matched = pub.get_subscription_count()
print(f'matched {matched} subscriber(s) after {time.time() - t0:.1f}s')
if matched == 0:
    print('NO SUBSCRIBERS MATCHED — refusing to publish into the void')
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(1)

msg = String()
msg.data = payload
# ONE publish by default. A duplicate `start` hits the build_busy branch in
# start_build, which calls status_dict() while already holding the non-reentrant
# self._lock — a permanent self-deadlock that wedges the whole node (observed
# 2026-09-11 01:53, evidence in log/incident_20260911/). Matching + RELIABLE
# already guarantees delivery, so repeating buys nothing and costs everything.
N = int(sys.argv[2]) if len(sys.argv) > 2 else 1
for _ in range(N):
    pub.publish(msg)
    time.sleep(0.3)
print(f'published {N} time(s): {payload}')
time.sleep(0.5)
node.destroy_node()
rclpy.shutdown()
