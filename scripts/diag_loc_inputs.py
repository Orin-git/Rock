"""Read-only: replicate the three inputs of localization_health._raw_code()."""
import math, time
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener
import tf2_py  # noqa

rclpy.init()
n = Node('diag_loc_inputs')
buf = Buffer()
listener = TransformListener(buf, n)

state = {'amcl': None, 'amcl_mono': None, 'scan_mono': None, 'amcl_cov': None}

def on_amcl(m):
    state['amcl'] = m
    state['amcl_mono'] = time.monotonic()
    state['amcl_cov'] = (round(m.pose.covariance[0], 4), round(m.pose.covariance[7], 4),
                         round(m.pose.covariance[35], 4))

n.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', on_amcl, 10)
def on_scan(m):
    state['scan_mono'] = time.monotonic()
n.create_subscription(LaserScan, '/scan', on_scan, 10)

t0 = time.monotonic()
while time.monotonic() - t0 < 8.0:
    rclpy.spin_once(n, timeout_sec=0.2)

now_m = time.monotonic()
print('--- AMCL ---')
if state['amcl'] is None:
    print('  /amcl_pose        : NEVER RECEIVED in 8s   -> _amcl is None? no, latched? check QoS')
else:
    m = state['amcl']
    hdr_age = (n.get_clock().now() - Time.from_msg(m.header.stamp)).nanoseconds / 1e9
    print(f'  /amcl_pose        : arrived {now_m - state["amcl_mono"]:.2f}s ago, '
          f'header age {hdr_age:.2f}s, frame_id={m.header.frame_id!r}')
    print(f'  covariance xy/yaw : {state["amcl_cov"]}')
    p = m.pose.pose
    print(f'  pose              : x={p.position.x:.3f} y={p.position.y:.3f}')

print('--- SCAN ---')
_s = state["scan_mono"]
print(f'  /scan             : ' + ('never' if _s is None else f'{now_m - _s:.2f}s ago'))

print('--- TF ---')
for parent, child in (('map', 'odom'), ('map', 'base_link'), ('odom', 'base_link'),
                      ('base_link', 'laser'), ('base_link', 'lidar')):
    try:
        t = buf.lookup_transform(parent, child, Time())
        tr = t.transform.translation
        print(f'  {parent:9s} -> {child:10s}: OK  x={tr.x:.3f} y={tr.y:.3f}')
    except Exception as e:
        print(f'  {parent:9s} -> {child:10s}: FAIL  {type(e).__name__}: {str(e)[:70]}')

print('--- odom static since amcl? ---')
try:
    if state['amcl'] is not None:
        t_ac = buf.lookup_transform('odom', 'base_link', Time.from_msg(state['amcl'].header.stamp))
        t_now = buf.lookup_transform('odom', 'base_link', Time())
        dx = t_now.transform.translation.x - t_ac.transform.translation.x
        dy = t_now.transform.translation.y - t_ac.transform.translation.y
        qa, qn = t_ac.transform.rotation, t_now.transform.rotation
        ya = math.atan2(2*(qa.w*qa.z), 1-2*qa.z**2); yn = math.atan2(2*(qn.w*qn.z), 1-2*qn.z**2)
        dyaw = abs((yn - ya + math.pi) % (2*math.pi) - math.pi)
        print(f'  odom moved since last amcl: dxy={math.hypot(dx,dy):.3f} m (thresh 0.12), '
              f'dyaw={math.degrees(dyaw):.1f} deg (thresh 6.9)')
        print(f'  => _odom_nearly_static_since_amcl = {math.hypot(dx,dy) < 0.12 and dyaw < 0.12}')
except Exception as e:
    print(f'  could not compare: {type(e).__name__}: {str(e)[:80]}')

rclpy.shutdown()
