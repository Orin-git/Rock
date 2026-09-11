"""Read-only: proper-QoS TF tree + long observation of loc data flow."""
import time, math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import Int8

LATCH = QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                   reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST)
rclpy.init()
n = Node('diag_tf2')
edges, counts = {}, {}
def on_tf(m):
    for t in m.transforms:
        edges[(t.header.frame_id, t.child_frame_id)] = t
        counts[(t.header.frame_id, t.child_frame_id)] = counts.get((t.header.frame_id, t.child_frame_id), 0) + 1
n.create_subscription(TFMessage, '/tf', on_tf, 100)
n.create_subscription(TFMessage, '/tf_static', on_tf, LATCH)
stat = {k: [0, None] for k in ('amcl', 'odom', 'imu', 'locst')}
def mk(k, typ, topic, qos=10):
    def cb(m):
        stat[k][0] += 1; stat[k][1] = time.monotonic()
    n.create_subscription(typ, topic, cb, qos)
mk('amcl', PoseWithCovarianceStamped, '/amcl_pose')
mk('odom', Odometry, '/odom')
mk('imu', Imu, '/imu/data')
mk('locst', Int8, '/xw/localization_status', LATCH)

t0 = time.monotonic()
while time.monotonic() - t0 < 40.0:
    rclpy.spin_once(n, timeout_sec=0.2)
rclpy.shutdown()

print(f'--- TF edges after 40 s ({len(edges)} distinct) ---')
for (p, c), cnt in sorted(counts.items()):
    print(f'    {p:26s} -> {c:26s}  {cnt} msgs')
print(f'  frames: {sorted({f for e in edges for f in e})}')
print('--- data flow over 40 s ---')
for k, (cnt, last) in stat.items():
    age = 'never' if last is None else f'{time.monotonic()-last:.2f}s ago'
    print(f'  {k:6s}: {cnt:4d} msgs   last={age}')
