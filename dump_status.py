"""One-shot: print the latched /xw/visual_db/build_status payload verbatim."""
import sys, time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.qos import (QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy)

rclpy.init()
n = Node('status_dump')
got = []
q = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
               durability=DurabilityPolicy.TRANSIENT_LOCAL,
               history=HistoryPolicy.KEEP_LAST, depth=1)
n.create_subscription(String, '/xw/visual_db/build_status', lambda m: got.append(m.data), q)
t0 = time.time()
while time.time() - t0 < 8.0 and not got:
    rclpy.spin_once(n, timeout_sec=0.2)
if got:
    sys.stdout.write(got[-1])
else:
    sys.stdout.write('{"_error":"no_status_message"}')
