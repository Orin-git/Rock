#!/usr/bin/env python3
import math, sys, time
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import Int8

def yaw_of(msg):
    q = msg.pose.pose.orientation
    return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))

def ang_diff(a,b):
    d=a-b
    while d>math.pi: d-=2*math.pi
    while d<-math.pi: d+=2*math.pi
    return d

class Cap(Node):
    def __init__(self):
        super().__init__("ej2")
        self.pose=None
        self.loc=-1
        qos=QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(PoseWithCovarianceStamped, "/amcl_pose", self._p, qos)
        self.create_subscription(Int8, "/xw/localization_status", self._l, 10)
    def _p(self,m): self.pose=m
    def _l(self,m): self.loc=int(m.data)

def main():
    out=sys.argv[1]
    rclpy.init(); n=Cap()
    # wait first pose up to 10s
    t0=time.time()
    while time.time()-t0<10 and n.pose is None:
        rclpy.spin_once(n, timeout_sec=0.1)
    if n.pose is None:
        open(out,"w").write("ERROR no amcl_pose\n"); print("ERROR no amcl_pose"); 
        n.destroy_node(); rclpy.shutdown(); return
    times=[0,1,2,5,10]
    t_start=time.time()
    rows=["t_s,loc,x,y,yaw,cov_xx,cov_yy,cov_yaw,dpos_m,dyaw_deg"]
    p0=n.pose; x0=p0.pose.pose.position.x; y0=p0.pose.pose.position.y; yaw0=yaw_of(p0)
    for t in times:
        while time.time()-t_start < t:
            rclpy.spin_once(n, timeout_sec=0.05)
        # refresh
        for _ in range(20):
            rclpy.spin_once(n, timeout_sec=0.05)
        p=n.pose
        x,y,yaw=p.pose.pose.position.x,p.pose.pose.position.y,yaw_of(p)
        c=p.pose.covariance
        dpos=math.hypot(x-x0,y-y0)
        dyaw=abs(ang_diff(yaw,yaw0))*180/math.pi
        line="%d,%d,%.4f,%.4f,%.4f,%.6g,%.6g,%.6g,%.4f,%.3f"%(
            t,n.loc,x,y,yaw,c[0],c[7],c[35],dpos,dyaw)
        rows.append(line); print(line)
    rows.append("# ExitPoseJump_vs_t0_at_10s position_m=%.4f yaw_deg=%.3f"%(dpos,dyaw))
    open(out,"w").write("\n".join(rows)+"\n")
    print("ExitPoseJump position=%.4fm yaw=%.3fdeg"%(dpos,dyaw))
    n.destroy_node(); rclpy.shutdown()

if __name__=="__main__":
    main()
