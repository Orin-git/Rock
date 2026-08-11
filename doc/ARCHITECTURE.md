# 小维二代 (Rock 5T) 架构说明

> 工作空间：宿主机 `/home/radxa/ros2_ws` ≡ 容器内 `/ros2_ws`  
> 运行环境：Docker `ros2_humble_dev`（`--net=host --privileged -v /dev:/dev -v ros2_ws:/ros2_ws`）

## 1. 分层一览

| 层 | 包 | 职责 |
|----|-----|------|
| 契约 | `xw_interfaces` | msg/srv 唯一源 |
| 驱动 | `xw_chassis`, `xw_sensors`, `xw_description` | 底盘 / 传感器 / TF |
| 安全运动 | `xw_cmd_arbiter`, `xw_safety_gate`, `xw_motion` | 仲裁 → 安全门 → 点动 |
| 应用 | `xw_supervisor` + `*_session` + `xw_map_manager` | 模式机与会话 |
| 感知 | `xw_perception` | 人体轨迹 / 跌倒（stub→算法） |
| 对外 | `xw_web`, `foxglove_bridge` | SPA + WS |
| 入口 | `xw_bringup`, `xw_health` | launch / 健康 |

## 2. 新功能放哪里？

| 要加的内容 | 放到 |
|------------|------|
| 新 msg/srv | `xw_interfaces` |
| 新传感器驱动 | `xw_sensors` 适配节点 |
| 新运动源（回充等） | 发布 `/xw/cmd/<name>` + 改 `xw_cmd_arbiter` 优先级 |
| 新业务模式 | 新 `xw_*_session` + `supervisor` 注册 mode |
| 页面 | `xw_web/public/pages/` + `api.js` 一条 API |
| 启动组合 | 只改 `xw_bringup/launch/robot.launch.py` |

**禁止**：业务直接写 `/cmd_vel`；HTML 里拼 ad-hoc topic；pgrep 当会话状态真理。

## 3. 速度链路（红线）

```
/xw/cmd/teleop | motion | nav | follow | recharge
        → xw_cmd_arbiter
        → xw_safety_gate   (/scan + ultrasonic [+ depth later])
        → /cmd_vel
        → xw_chassis
```

优先级：`estop > teleop > motion > nav > follow > recharge`

## 4. 模式 FSM（supervisor）

| mode | 常量 | 会话 |
|------|------|------|
| IDLE | 0 | — |
| MAPPING | 1 | slam_session |
| NAVIGATING | 2 | nav_session |
| FOLLOWING | 3 | follow_session |
| FALL_DETECT | 4 | fall_session |

默认互斥；`SetMode` 的 `mode=0` 为取消回 IDLE。

## 5. 统一契约

| 方向 | 名字 | 说明 |
|------|------|------|
| Snap | `/xw/robot_state` | latched 全局态 |
| Progress | `/xw/task/progress` | 过程 |
| Result | `/xw/task/result` | **唯一终态** |
| Event | `/xw/event` | 急停/安全/定位 |
| Power | `/xw/power` | 电量 |
| Srv | `/xw/supervisor/set_mode` | 改模式 |
| Srv | `/xw/supervisor/get_state` | 拉快照 |
| Srv | `/xw/map/manage` | 地图 CRUD |
| Srv | `/xw/map/waypoint` | 航点 |
| Srv | `/xw/motion/command` | 定距点动 |
| In | `/xw/cmd/teleop` | 遥控白名单入口 |
| In | `/xw/goal_pose` | 导航目标（预留） |

## 6. Docker 日常

```bash
# 进入容器
docker exec -it ros2_humble_dev bash

# 环境 + 编译
source /ros2_ws/scripts/ros_env.sh
cd /ros2_ws && colcon build --symlink-install
source /ros2_ws/install/setup.bash

# 启动 mock 基座
ros2 launch xw_bringup robot.launch.py

# 可选：只读挂一代参考（宿主机重建容器时加）
# -v $HOME/vs_ws1:/vs_ws1:ro
```

环境变量：`XW_WS` `XW_MAPS` `XW_LOG`（默认 `/ros2_ws` 下）。

端口：Web `9000`，Foxglove `8765`（host 网络 = 板子 IP）。

## 7. Phase 路线

- **P0（当前）**：mock 栈 + FSM + Web SPA  
- **P1**：真底盘 / 雷达 / 超声  
- **P2**：手推建图已落地（`slam_toolbox` + `/map` 画布 + `{map}_pointList/charger`）；Nav2 仍待接入  
- **P3**：双深度相机 + 跟随/跌倒算法  
- **P4**：回充 / 压测 / 可选 Gateway  

### 手推建图要点

- `set_mode(1)` → `xw_slam_session` 自管启停 `async_slam_toolbox_node`，抓取 `map→base_link` 起点  
- 保存：`/xw/map/manage` op=1 → `map_saver_cli` → upsert `waypoints/{name}_pointList.yaml` 的 `charger`（yaw = tf_yaw+π）  
- 未保存停止 → `autosave_YYYYMMDD_HHMMSS`  
- Web：`/pages/mapping.html`（Foxglove `/map`+`/scan`），`/pages/maps.html`（批量 CRUD，级联 pointList）  
- HTTP：`POST /api/map`、`POST /api/waypoint`  

## 8. 验收（容器内）

1. `colcon build` 通过  
2. `robot.launch.py` mock 可起  
3. teleop → `/cmd_vel` → mock odom  
4. 浏览器连 9000/8765，`set_mode` 互斥可见  

## 9. Supervisor 与 Session 协作

- `SetMode` 只改模式并在 `/xw/slam|nav|follow|fall/enable`（latched Bool）上发命令。
- Session 既可订阅 enable，也可暴露 `/xw/session/*/control` 供调试直连。
- **禁止** Supervisor 在 service 回调里 `spin_until_future_complete` 再调 session service（会死锁）。
