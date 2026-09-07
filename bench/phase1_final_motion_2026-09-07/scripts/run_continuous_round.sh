#!/usr/bin/env bash
# One Continuous Follow round: record + follow ON for DUR_FOLLOW + exit jump + nav to charger.
# Usage: run_continuous_round.sh <round_dir> <follow_sec>
set -eo pipefail
set +u
source /ros2_ws/scripts/ros_env.sh
set -e
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-99}"
OUT="${1:?round dir}"
FOLLOW_SEC="${2:-180}"
mkdir -p "$OUT"
echo "ROUND_START $(date -Iseconds) follow_sec=$FOLLOW_SEC" | tee "$OUT/meta.txt"

# Confirm continuous + no beamskip
ros2 param set /xw_follow_session follow_localization_mode continuous
ros2 param get /xw_follow_session follow_localization_mode | tee "$OUT/flags.txt"
timeout 5 ros2 param get /amcl laser_model_type >> "$OUT/flags.txt" 2>&1 || true
timeout 5 ros2 param get /amcl do_beamskip >> "$OUT/flags.txt" 2>&1 || true

# Start pose
python3 - <<PY | tee "$OUT/pose_start.txt"
import time, rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseWithCovarianceStamped
class N(Node):
    def __init__(self):
        super().__init__("ps"); self.p=None
        qos=QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL, history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(PoseWithCovarianceStamped, "/amcl_pose", lambda m: setattr(self,"p",m), qos)
rclpy.init(); n=N(); t0=time.time()
while time.time()-t0<5 and n.p is None: rclpy.spin_once(n, timeout_sec=0.05)
if n.p is None: print("NO_POSE"); 
else:
    p=n.p.pose.pose.position; c=n.p.pose.covariance
    print(f"x={p.x:.4f} y={p.y:.4f} cov_xx={c[0]:.6g} cov_yy={c[7]:.6g} cov_yaw={c[35]:.6g}")
n.destroy_node(); rclpy.shutdown()
PY

# Recorder for follow window + buffer
REC=$((FOLLOW_SEC + 30))
bash /ros2_ws/bench/phase1_final_motion_2026-09-07/scripts/record_motion_window.sh "$OUT" "$REC" "T3" \
  > "$OUT/record_log.txt" 2>&1 &
REC_PID=$!
sleep 2

ros2 service call /xw/supervisor/set_follow std_srvs/srv/SetBool "{data: true}" | tee "$OUT/follow_on.txt"
echo "FOLLOW_ON $(date -Iseconds)" | tee -a "$OUT/meta.txt"
echo "OPERATOR: walk ≥30m (straight+turn+brief occlusion) NOW"

# Light motion / pose log every 15s
python3 - <<PY | tee "$OUT/during_follow.txt"
import time, math, rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from std_msgs.msg import Int8

class N(Node):
    def __init__(self):
        super().__init__("dur")
        self.pose=None; self.loc=-1; self.cmd=(0.0,0.0); self.max_v=0.0
        qos=QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL, history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(PoseWithCovarianceStamped, "/amcl_pose", self.on_p, qos)
        self.create_subscription(Int8, "/xw/localization_status", lambda m: setattr(self,"loc",int(m.data)), 10)
        self.create_subscription(Twist, "/cmd_vel", self.on_c, 10)
    def on_p(self,m): self.pose=m
    def on_c(self,m):
        self.cmd=(m.linear.x, m.angular.z)
        self.max_v=max(self.max_v, abs(m.linear.x))

rclpy.init(); n=N()
t0=time.time(); dur=float("$FOLLOW_SEC")
x0=y0=None
print("t,loc,x,y,cov_xx,cov_yy,vx,wz")
while time.time()-t0 < dur:
    rclpy.spin_once(n, timeout_sec=0.05)
    if int(time.time()-t0) % 15 == 0:
        # debounce print once per second slot
        pass
    now=time.time()
    if not hasattr(n,"_next") or now>=n._next:
        n._next=now+15.0
        if n.pose is not None:
            p=n.pose.pose.pose.position; c=n.pose.pose.covariance
            if x0 is None: x0,y0=p.x,p.y
            d=math.hypot(p.x-x0, p.y-y0) if x0 is not None else 0.0
            print(f"{now-t0:.0f},{n.loc},{p.x:.3f},{p.y:.3f},{c[0]:.4g},{c[7]:.4g},{n.cmd[0]:.3f},{n.cmd[1]:.3f},disp={d:.2f}")
        else:
            print(f"{now-t0:.0f},{n.loc},,,,,,,max_v={n.max_v:.3f}")
# end pose
if n.pose is not None and x0 is not None:
    p=n.pose.pose.pose.position
    d=math.hypot(p.x-x0,p.y-y0)
    print(f"END_DISP_m={d:.3f} max_vx={n.max_v:.3f} end_xy=({p.x:.3f},{p.y:.3f})")
    open("$OUT/displacement.txt","w").write(f"map_displacement_m={d:.3f} max_vx={n.max_v:.3f} end_xy=({p.x},{p.y}) start_xy=({x0},{y0})\n")
n.destroy_node(); rclpy.shutdown()
PY

ros2 service call /xw/supervisor/set_follow std_srvs/srv/SetBool "{data: false}" | tee "$OUT/follow_off.txt"
echo "FOLLOW_OFF $(date -Iseconds)" | tee -a "$OUT/meta.txt"

# Immediate exit jump
python3 /ros2_ws/bench/phase1_final_motion_2026-09-07/scripts/exit_pose_jump_tl.py "$OUT/exit_jump.txt" | tee "$OUT/exit_jump_run.txt"

# Follow → Nav to charger
python3 /ros2_ws/bench/phase1_final_motion_2026-09-07/scripts/follow_to_nav.py charger "$OUT/follow_to_nav.txt" | tee "$OUT/follow_to_nav_run.txt"

wait $REC_PID || true
echo "ROUND_DONE $(date -Iseconds)" | tee -a "$OUT/meta.txt"
cat "$OUT/displacement.txt" 2>/dev/null || true
cat "$OUT/exit_jump.txt" 2>/dev/null | tail -3
cat "$OUT/follow_to_nav.txt" 2>/dev/null || true
