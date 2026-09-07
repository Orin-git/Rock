#!/usr/bin/env bash
# Phase1 runtime helpers — run inside container after ros_env.
set -eo pipefail
set +u
source /ros2_ws/scripts/ros_env.sh
set -e
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-99}"
OUT="${1:?out dir}"
mkdir -p "$OUT"
DUR="${2:-65}"

cpu_sample() {
  local secs="$1" dest="$2"
  python3 - <<PY
import time, collections
secs=float("$secs")
cores=None
samples=[]
t0=time.time()
while time.time()-t0 < secs:
    idle=[]; total=[]
    with open("/proc/stat") as f:
        for line in f:
            if not line.startswith("cpu"):
                break
            p=line.split()
            if p[0]=="cpu":
                vals=list(map(int,p[1:]))
                idle.append(vals[3]+(vals[4] if len(vals)>4 else 0)); total.append(sum(vals))
            elif p[0].startswith("cpu") and p[0][3:].isdigit():
                vals=list(map(int,p[1:]))
                idle.append(vals[3]+(vals[4] if len(vals)>4 else 0)); total.append(sum(vals))
    samples.append((idle,total)); time.sleep(0.5)
# compute deltas
import statistics as st
all_u=[]; per=collections.defaultdict(list)
for i in range(1,len(samples)):
    ia,ta=samples[i-1]; ib,tb=samples[i]
    du=100.0*(1.0-(ib[0]-ia[0])/max(1,(tb[0]-ta[0])))
    all_u.append(du)
    for c in range(1,min(len(ia),len(ib))):
        per[c-1].append(100.0*(1.0-(ib[c]-ia[c])/max(1,(tb[c]-ta[c]))))
def pct(xs,p):
    xs=sorted(xs); 
    if not xs: return float("nan")
    k=(len(xs)-1)*p/100.0; f=int(k); c=min(f+1,len(xs)-1); return xs[f]*(c-k)+xs[c]*(k-f) if f!=c else xs[f]
with open("$dest","w") as o:
    o.write(f"all: avg={sum(all_u)/len(all_u):.1f} p50={pct(all_u,50):.1f} p95={pct(all_u,95):.1f} max={max(all_u):.1f}\n")
    for c,xs in sorted(per.items()):
        o.write(f"cpu{c}: avg={sum(xs)/len(xs):.1f} p50={pct(xs,50):.1f} p95={pct(xs,95):.1f} max={max(xs):.1f}\n")
    la=open("/proc/loadavg").read().split()
    o.write(f"loadavg: {la[0]} {la[1]} {la[2]}\n")
    try:
        npu=open("/sys/class/devfreq/fdab0000.npu/load").read().strip()
    except Exception:
        npu="NA"
    o.write(f"npu={npu}\n")
print(open("$dest").read())
PY
}

hz_once() {
  local topic="$1" secs="${2:-8}"
  timeout "$secs" ros2 topic hz "$topic" 2>&1 | grep -E "average rate|does not appear|WARNING" | tail -3 || true
}

echo "OUT=$OUT DUR=$DUR"
date -Iseconds | tee "$OUT/ts.txt"

# system
{
  echo "loadavg=$(cat /proc/loadavg)"
  echo -n "npu="; cat /sys/class/devfreq/fdab0000.npu/load 2>/dev/null || echo NA
  free -h | head -3
} | tee "$OUT/sys.txt"

cpu_sample "$DUR" "$OUT/cpu.txt"

# top ros
timeout 5 top -b -n 1 -o %CPU 2>/dev/null | head -5 > "$OUT/top_head.txt" || true
ps -eo pid,pcpu,pmem,rss,comm,args --sort=-pcpu 2>/dev/null | grep -E 'ros2|/ros2_ws/install|/opt/ros' | head -35 | tee "$OUT/top_ros.txt" || true

# rates
{
  echo "===== /scan ====="; hz_once /scan 10
  echo "===== /odom ====="; hz_once /odom 8
  echo "===== /amcl_pose ====="; hz_once /amcl_pose 8
  echo "===== /camera/front_up/depth/image_raw ====="; hz_once /camera/front_up/depth/image_raw 12
  echo "===== /camera/front_up/color/image_raw ====="; hz_once /camera/front_up/color/image_raw 12
  echo "===== /camera/front_down/depth/image_raw ====="; hz_once /camera/front_down/depth/image_raw 12
  echo "===== /camera/front_down/color/image_raw ====="; hz_once /camera/front_down/color/image_raw 12
  echo "===== /xw/localization_status ====="; hz_once /xw/localization_status 6
} | tee "$OUT/hz.txt"

# state / loc / flags
timeout 5 ros2 topic echo /xw/robot_state --once 2>&1 | tee "$OUT/robot_state.txt" || true
timeout 5 ros2 topic echo /xw/localization_status --once 2>&1 | tee "$OUT/loc_status.txt" || true
timeout 5 ros2 topic echo /xw/perception/profile --once 2>&1 | tee "$OUT/perception_profile.txt" || true
timeout 5 ros2 topic echo /xw/fall/enable --once 2>&1 | tee "$OUT/fall_enable.txt" || true
timeout 3 ros2 topic echo /amcl_pose --once 2>&1 | tee "$OUT/amcl_pose.txt" || true

# params
{
  ros2 param get /xw_follow_session follow_localization_mode || true
  ros2 param get /xw_depth_topic_bridge relay_depth || true
  ros2 param get /xw_depth_topic_bridge lazy_mjpeg || true
  ros2 param get /xw_depth_topic_bridge lazy_rgb_info || true
  ros2 param get /amcl laser_model_type || true
  ros2 param get /amcl do_beamskip || true
} 2>&1 | tee "$OUT/flags.txt"

# cmd_vel presence
timeout 3 ros2 topic echo /cmd_vel --once 2>&1 | tee "$OUT/cmd_vel_once.txt" || echo "NO_CMD_VEL_MSG" | tee "$OUT/cmd_vel_once.txt"

echo DONE
