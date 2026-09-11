import math, time, json
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import String

rclpy.init()
n = Node('recon_loc')
out = {}
def mk(topic, typ, cb):
    q = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                   reliability=ReliabilityPolicy.RELIABLE)
    n.create_subscription(typ, topic, cb, q)

def on_pose(m):
    p = m.pose.pose
    yaw = math.degrees(math.atan2(2*(p.orientation.w*p.orientation.z), 1-2*p.orientation.z**2))
    out['amcl_pose'] = (round(p.position.x,3), round(p.position.y,3), round(yaw,1))
    out['cov_xx'] = round(m.pose.covariance[0],4); out['cov_yy'] = round(m.pose.covariance[7],4)
mk('/amcl_pose', PoseWithCovarianceStamped, on_pose)

def on_st(m):
    out.setdefault('loc_status', []).append(str(m.data)[:120])
for t in ('/xw/localization/status','/xw/localization_state','/xw/phase2c/status'):
    try: mk(t, String, on_st)
    except Exception: pass

t0=time.time()
while time.time()-t0 < 6.0:
    rclpy.spin_once(n, timeout_sec=0.3)
rclpy.shutdown()
print(json.dumps(out, ensure_ascii=False, indent=1))
