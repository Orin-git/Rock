#!/usr/bin/env python3
"""Is /camera/front_up/color/image_raw actually delivering FRESH frames?

Bounded 6 s BEST_EFFORT sample -- the same QoS the capture node uses. Not a
continuous echo. Reports arrival count and header age, which is what the
capture node's `rgb=` gate actually tests.
"""
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

DUR = 6.0
TOPIC = '/camera/front_up/color/image_raw'


class P(Node):
    def __init__(self):
        super().__init__('probe_rgb')
        self.t = []
        self.last = None
        self.create_subscription(Image, TOPIC, self.cb, qos_profile_sensor_data)

    def cb(self, m):
        self.t.append(time.time())
        self.last = m


rclpy.init(); n = P()
t0 = time.time()
while time.time() - t0 < DUR:
    rclpy.spin_once(n, timeout_sec=0.1)

now = time.time()
print(f'{TOPIC}')
print(f'  frames in {DUR:.0f} s = {len(n.t)}   ({len(n.t)/DUR:.1f} Hz)')
if n.t:
    print(f'  last frame age = {now - n.t[-1]:.2f} s')
if n.last is not None:
    m = n.last
    hdr = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
    print(f'  header stamp age (vs wall) = {now - hdr:.2f} s')
    print(f'  frame_id={m.header.frame_id}  {m.width}x{m.height}  '
          f'enc={m.encoding}  bytes={len(m.data)}')
    print('  VERDICT: FRESH' if (now - n.t[-1]) < 1.0 and len(n.t) > 5
          else '  VERDICT: STALE or ABSENT')
else:
    print('  NO FRAME RECEIVED — camera is not delivering')
rclpy.shutdown()
