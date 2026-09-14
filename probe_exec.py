#!/usr/bin/env python3
"""决定性探针：执行器活性 + 监听器日志真相 + 完整 status payload"""
import json
import os
import time

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger

print("=== [1] watcher 日志 stat（两次，间隔 3s）===", flush=True)
WL = "/ros2_ws/log/vdb_cmd_watch.log"
for i in range(2):
    try:
        st = os.stat(WL)
        print(f"  t{i}: size={st.st_size} mtime={st.st_mtime}", flush=True)
    except Exception as e:
        print(f"  t{i}: ERR {e}", flush=True)
    time.sleep(3)

print("=== [2] 执行器活性：Trigger 服务（挂 _cb 组）===", flush=True)
rclpy.init()
n = Node("probe_exec_liveness")
cli = n.create_client(Trigger, "/xw/visual_db/build_status_svc")
ok = cli.wait_for_service(timeout_sec=15.0)
print(f"  wait_for_service -> {ok}", flush=True)
if ok:
    fut = cli.call_async(Trigger.Request())
    t0 = time.time()
    while not fut.done() and time.time() - t0 < 15.0:
        rclpy.spin_once(n, timeout_sec=0.2)
    print(f"  done={fut.done()} elapsed={time.time()-t0:.2f}s", flush=True)
    if fut.done() and fut.result() is not None:
        r = fut.result()
        print(f"  success={r.success} msg_len={len(r.message)}", flush=True)
        d = json.loads(r.message)
        print("  --- 关键字段 ---", flush=True)
        for k in ("state", "message", "stop_reason", "loc_status", "phase2c_state",
                  "goals_blocked", "current_goal"):
            v = d.get(k)
            if k == "current_goal" and isinstance(v, dict):
                v = {kk: v.get(kk) for kk in ("spatial_cell", "yaw_bin")}
            print(f"    {k} = {v!r}", flush=True)
        print(f"    coverage_after keys = {len(d.get('coverage_after') or {})}", flush=True)
    else:
        print("  *** 服务超时 —— 执行器未处理回调 ***", flush=True)
else:
    print("  *** 服务不存在 —— 端点未注册 ***", flush=True)

n.destroy_node()
rclpy.shutdown()
