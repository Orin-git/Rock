"""Read-only: enumerate the live TF tree and frame publishers."""
import time
import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import PoseWithCovarianceStamped

rclpy.init()
n = Node('diag_frames')
frames, edges = {}, {}
def on_tf(m, static=False):
    for t in m.transforms:
        p, c = t.header.frame_id, t.child_frame_id
        edges[(p, c)] = edges.get((p, c), 0) + 1
        frames[p] = frames.get(p, 0) + 1
        frames[c] = frames.get(c, 0) + 1
n.create_subscription(TFMessage, '/tf', lambda m: on_tf(m), 100)
n.create_subscription(TFMessage, '/tf_static', lambda m: on_tf(m, True), 100)
amcl = {'t': None, 'msg': None}
def on_amcl(m):
    amcl['t'] = time.monotonic(); amcl['msg'] = m
n.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', on_amcl, 10)

t0 = time.monotonic()
while time.monotonic() - t0 < 12.0:
    rclpy.spin_once(n, timeout_sec=0.2)
rclpy.shutdown()

print(f'--- /tf + /tf_static over 12 s ---')
if not edges:
    print('  NO TF MESSAGES AT ALL')
else:
    print(f'  distinct edges: {len(edges)}')
    for (p, c), cnt in sorted(edges.items()):
        print(f'    {p:22s} -> {c:22s}  ({cnt} msgs)')
    print(f'  distinct frames: {sorted(frames)}')
print('--- /amcl_pose ---')
if amcl['t'] is None:
    print('  NEVER received in 12 s  -> AMCL is not publishing (or QoS mismatch)')
else:
    print(f'  received {time.monotonic()-amcl["t"]:.2f}s ago on a VOLATILE subscription')
    print(f'  frame_id = {amcl["msg"].header.frame_id!r}')
