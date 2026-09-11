#!/usr/bin/env python3
"""Print the FULL latched build status dict once, and exit.

The status topic is the node's live truth; the session JSON only lands at round
boundaries, so during round 1 the JSON is stale by definition. Read-only.

`--watch N` reprints every N seconds (default: once).
"""
import json
import sys
import time
from collections import Counter

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy,
)
from std_msgs.msg import String

PERIOD = float(sys.argv[sys.argv.index('--watch') + 1]) if '--watch' in sys.argv else 0.0
QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,  # the topic is latched
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)

rclpy.init()
node = Node('vdb_status_once')
got = []


def cb(msg):
    got.append((time.time(), msg.data))


node.create_subscription(String, '/xw/visual_db/build_status', cb, QOS)

t0 = time.time()
while not got and time.time() - t0 < 10.0:
    rclpy.spin_once(node, timeout_sec=0.2)

if not got:
    print('NO LATCHED STATUS — is xw_visual_db_build running?')
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(1)


COMPACT = '--compact' in sys.argv


def show(ts, raw):
    print(f'--- {time.strftime("%F %T", time.localtime(ts))} ---')
    try:
        d = json.loads(raw)
    except Exception:  # noqa: BLE001
        print(raw)
        return
    if not COMPACT:
        print(json.dumps(d, indent=2, sort_keys=True, ensure_ascii=False))
        return
    # cell_status is ~30 KB of per-cell noise that swamps everything else. Fold
    # it into a histogram; the interesting keys are the ratios derived from it.
    # Done in-script on purpose: piping this through ssh+docker requires nested
    # quoting that has already mangled payloads more than once.
    comp = d.pop('completion', None) or {}
    cells = comp.pop('cell_status', {}) if isinstance(comp, dict) else {}
    print(json.dumps(d, indent=2, sort_keys=True, ensure_ascii=False))
    print(f'--- completion (cell_status folded away, {len(cells)} cells) ---')
    print(json.dumps(comp, indent=2, sort_keys=True, ensure_ascii=False))
    if cells:
        hist = Counter(cells.values())
        print('--- cell_status histogram ---')
        print(json.dumps(dict(sorted(hist.items())), indent=2))


show(*got[-1])
if PERIOD > 0:
    last = got[-1][1]
    while True:
        time.sleep(PERIOD)
        rclpy.spin_once(node, timeout_sec=0.1)
        if got and got[-1][1] != last:
            last = got[-1][1]
            show(*got[-1])

node.destroy_node()
rclpy.shutdown()
