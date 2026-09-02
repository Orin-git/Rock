# 小维二代 (Rock 5T) 架构说明

> 工作空间：宿主机 `/home/radxa/ros2_ws` ≡ 容器内 `/ros2_ws`  
> 运行环境：Docker `ros2_humble_dev`（`--net=host --privileged -v /dev:/dev -v ros2_ws:/ros2_ws`）

## 1. 分层一览

| 层 | 包 | 职责 |
|----|-----|------|
| 契约 | `xw_interfaces` | msg/srv 唯一源 |
| 驱动 | `xw_chassis`, `xw_sensors`, `xw_description`, `third_party/ascamera` | 底盘 / 传感器 / TF / 深度相机 |
| 安全运动 | `xw_cmd_arbiter`, `xw_safety_gate`, `xw_motion` | 仲裁 → 安全门 → 点动 |
| 应用 | `xw_supervisor` + `*_session` + `xw_map_manager` | 模式机与会话 |
| 感知 | `xw_perception` | 人体轨迹 / 跌倒（YOLOv8n-pose RKNN） |
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
        → xw_cmd_arbiter  (+ /xw/cmd/active_source)
        → xw_safety_gate  (teleop: 扇区OA；nav: 硬停+后方安全后退)
        → /cmd_vel
        → xw_chassis
```

导航支路：`Nav2 controller → velocity_smoother(cmd_vel_smoothed) → collision_monitor → /xw/cmd/nav`

优先级：`teleop > motion > nav > follow > recharge`（MCU 失能见 `/xw/chassis/motor_disabled`，不进速度仲裁）

深度相机（前视 HP60C）：驱动随 `robot.launch.py` **常开**；Web 彩色预览仅在打开 `/pages/camera.html` 时订 compressed 推流。

## 4. 模式 FSM（supervisor）

| mode | 常量 | 会话 |
|------|------|------|
| IDLE | 0 | — |
| MAPPING | 1 | slam_session |
| NAVIGATING | 2 | nav_session（Nav2 能力） |
| FOLLOWING | 3 | **nav 保持** + `/xw/follow/enable` 任务 |
| FALL_DETECT | 4 | fall latch（正交） |

- 建图 ↔ 导航：运动栈互斥。
- **跟随是导航上的正交任务**：`/xw/supervisor/set_follow` / 设置页开关；开跟随只取消点位/巡航，**不 `_stop_nav2`**；关跟随后 Nav2 仍在。
- **回充是导航上的正交任务**：`/xw/supervisor/set_recharge` → `/xw/recharge/enable`；与跟随互斥；远场 Nav2 接近点，近场 `/xw/cmd/recharge`。
- `set_mode(3)` 兼容入口：确保 `/xw/nav/enable=true` + follow on。
- 跌倒仍走 `/xw/supervisor/set_fall`，与模式切换独立。

## 5. 统一契约

| 方向 | 名字 | 说明 |
|------|------|------|
| Snap | `/xw/robot_state` | latched 全局态（含 `localization_status` 0–3） |
| Progress | `/xw/task/progress` | 过程 |
| Result | `/xw/task/result` | **唯一终态** |
| Event | `/xw/event` | 急停/安全/定位 |
| Loc | `/xw/localization_status` | Int8 0–3 定位健康 |
| Power | `/xw/power` | 电量、`charging`/`docked`/`charging_current`（MCU `0x7C` 回充帧） |
| In | `/xw/recharge/enable` | 回充正交使能（latched） |
| Out | `/xw/recharge/status` | 回充阶段 JSON（网页状态条） |
| In | `/xw/cmd/recharge` | 近场贴桩速度 → 仲裁 |
| In | `/xw/chassis/charge_mode` | 底盘 TX[1] 回充模式闩锁 |
| Srv | `/xw/supervisor/set_mode` | 改模式 |
| Srv | `/xw/supervisor/set_follow` | 跟随任务开关（不拆 Nav2） |
| Srv | `/xw/supervisor/set_recharge` | 自动回充开关（不拆 Nav2） |
| Srv | `/xw/supervisor/set_fall` | 跌倒开关 |
| Srv | `/xw/supervisor/get_state` | 拉快照 |
| Srv | `/xw/map/manage` | 地图 CRUD |
| Srv | `/xw/map/waypoint` | 航点 |
| Srv | `/xw/motion/command` | 定距点动 |
| In | `/xw/cmd/teleop` | 遥控白名单入口 |
| In | `/xw/goal_pose` | 单点导航目标（跟随/回充中拒绝） |
| In | `/xw/nav/patrol_cmd` | 多点巡航 JSON（跟随/回充中拒绝） |
| In | `/xw/nav/cancel` | 软取消当前导航（不关 Nav2） |
| In | `/goal_update` | 动态跟随目标（Nav2 GoalUpdater） |
| In | `/xw/nav/map_name` | 导航地图名（latched） |
| In | `/initialpose` | AMCL 初始位姿 |

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

- **P0**：mock 栈 + FSM + Web SPA  
- **P1**：真底盘（`xw_chassis` 串口 `/dev/chassis` 或回退 `/dev/ttyACM0` @115200，一代 0x7B 协议；udev：`scripts/install_robot_udev.sh`）/ 雷达 / 超声（超声仍待）  
- **P2**：手推建图已落地；**Nav2/AMCL 已接入**（`xw_nav_session`）；单点/多点/初位姿 Web API 已通  
- **P3（部分）**：双前视深度 + 安全门深度 ROI；导航 local costmap 融合激光+双深度点云；**感知真节点**（YOLOv8n-pose RKNN）  
- P4（部分）：**WT901C485 + EKF 已默认启用**（`/odom/wheel` + `/imu/data` → `/odom`）；**自动回充 Laser-Lock Dock 已接入**（`xw_recharge` + MCU `0x7C` 电流确认）  

### USB 接口分配（Rock 5T）— HP60C 为 USB2.0 设备

Nuwa-HP60C 官方为 **USB2.0 接口**（不会出现 `speed=5000`，属正常）。  
两个蓝色口在 USB2 层共用 **同一条 VIA HS 总线（Bus1，480Mbps）**；两台深度相机都插蓝口会挤爆总线 → 掉线/libusb 崩溃。

**当前接线（供电/信号分线后，2026-08 实测）：**

| 板载口 / 拓扑 | 设备 | 实测 Bus/path |
|---------------|------|----------------|
| EHCI 根口（黑口一侧） | 深度 front_up | Bus **5** path `1` → `KERNEL 5-1` |
| 另一 EHCI + hub | 深度 front_down | Bus **3** path `1.2` |
| 蓝色 USB3 → VIA Hub | 拓展坞 → 底盘+IMU | Bus1 path `1.1.x`（`/dev/chassis` `/dev/imu`） |
| 蓝色 USB3 → VIA Hub | 激光雷达 CP210x | Bus1 path `1.2` → `/dev/radar` |

旧直插约定（仅作对照）：蓝口 cam1=`1/1.2`、黑口 cam2=`3/1.2`、黑口雷达、蓝口拓展坞。  
分线后 **cam1 必须改配 `usb_bus_no/usb_path`**，否则会误绑到雷达所在的 `1/1.2`。

**供电/信号分线注意：** CP210x 能枚举 ≠ 雷达有数据。雷达机身 5V、USB 地与 D+/D− 必须同时连通；否则 `rplidar` 会 `SL_RESULT_OPERATION_TIMEOUT`，网页显示激光无数据。

默认 `USE_DEPTH_CAM_2=true`（两路已分主机控制器）。

注意：HP60C SDK **不支持 fps=5**；深度用 **10**。已加 udev 解绑 `uvcvideo`（接口名 `*:1.0`/`*:1.1`），避免与 ascamera 抢接口。

### 传感器命名契约（Gen2）

| 设备 | 节点 | Frame | 公共话题 |
|------|------|-------|----------|
| 激光雷达 | `rplidar_node` | `lidar_link` | `/scan`（建图/AMCL/避障） |
| 前上深度 | `ascamera_hp60c/...` + bridge | `camera_front_up_link` | `/camera/front_up/{color,depth,...}` |
| 前下深度 | `ascamera_hp60c_2/...` + bridge | `camera_front_down_link` | `/camera/front_down/...` |
| 独立 IMU（WT901C485） | `xw_wt901_imu` | `imu_link` | `/imu/data`（Modbus RTU @9600，slave `0x50`） |
| 底盘轮式里程计 | `xw_chassis` | `base_link` | `/odom/wheel`（默认；EKF 关时 `/odom`） |
| EKF 融合（默认开） | `ekf_filter_node` | `odom→base_link` | `/odom` |

厂商私有话题 `/ascamera_hp60c{,_2}/...` 仅 bridge 订阅，业务节点只用 `/camera/...`。

**激光前向验收**：正前方挡板 → `/scan` 在 angle≈0 距离变短；否则只改 URDF `lidar_joint` yaw=`π` **或** 驱动 `inverted`（二选一）。

### 传感器职责

| 能力 | 激光 | 深度1 | 深度2 | IMU |
|------|------|-------|-------|-----|
| 建图 SLAM | 是 | 否 | 否 | 间接（稳 odom） |
| AMCL | 是 | 否 | 否 | 间接 |
| Local 避障 | 是 | points_nav | points_nav | 否 |
| 安全门 | 是 | ROI | 后续 | 否 |
| 人体跟随 | 否 | **是** | 否 | 否 |

### 深度相机要点

- 包：`third_party/ascamera` + `xw_sensors/launch/depth_camera.launch.py`  
- 进导航：`/xw/camera/set_pointcloud_nav` 开 #1；#2 镜像 `/xw/camera/pointcloud_enabled`  
- **建图不参与深度**：slam_toolbox 只用 `/scan`  
- raw RGB（#1）：仅在 `fall_en || follow_en` 时转发  

### EKF 融合（打滑友好，默认开）

- 默认 `use_ekf:=true` / `USE_EKF=true`：底盘 `/odom/wheel`（不发 TF）+ IMU → EKF → `/odom` + TF  
- 回退 `use_ekf:=false`：`chassis_odom_topic:=odom`、`chassis_publish_odom_tf:=true`  
- 融合：`odom0` 只融 **vx**；`imu0` 只融 **vyaw**（绝对 yaw / aX 待标定后再开）  
- **不用**底盘 MCU IMU 字节；只用中置 `imu_link` → `/imu/data`  
- 标定参考：[imu_utils](https://github.com/gaowenliang/imu_utils)、[Nav2 camera calibration](https://docs.nav2.org/tutorials/docs/camera_calibration.html)  

### 感知 / 跌倒 / 跟随

- **感知**：底部相机 `/camera/front_down/{color,depth}` → YOLOv8n-pose RKNN ~6Hz → `/xw/perception/tracks`
  - `is_primary`：最近/候选；`is_target`：跟随锁定（上升沿锁定 + Kalman/IoU 关联 + 遮挡 coast）
- **跟随（实时视觉伺服，默认）**：`xw_follow_session` 对 `is_target` 做 bearing+distance → `/xw/cmd/follow`（一代式 P 控制，检测帧率）
  - 摄像头：`camera_front_down_link`（底部 HP60C）
  - 可选 `use_nav2_follow:=true`：投影到 `map` + `NavigateToPose`（`follow_point.xml`）+ `/goal_update`（旧“打点”路径）
  - 丢失：TRACKING → COAST → SEARCH（旋转）→ LOST；不跟路人
  - 前置：已进入导航（有地图）；开关跟随 **不拆 Nav2**（视觉伺服不依赖规划器）
- **跌倒**：正交开关与 `/xw/perception/fall` 契约不变

### 导航要点（Gen2 重设计，非一代 MPPI 移植）

| 层 | 选型 |
|----|------|
| 全局 | **SmacPlanner2D** |
| 局部 | **RotationShim → Regulated Pure Pursuit**（禁止以 MPPI 作主控） |
| 动态绕障 | local costmap 路径失效 → BT 重规划 |
| 硬安全 | **Collision Monitor**（smoother 后）+ 模式感知 `xw_safety_gate` |
| 前向 | 正常跟线 `allow_reversing: false`；后退仅 recovery + 后方扇区安全 |

- 参数：`xw_nav_session/config/nav2_params.yaml`（多边形 footprint 0.45×0.35，`transform_tolerance` ≈0.3）
- BT：`behavior_trees/navigate_to_pose_gen2.xml`；跟随仍用 `follow_point.xml`
- Launch：`nav2.launch.py`：localization + navigation + collision_monitor → `/xw/cmd/nav`
- 点云：`xw_pc_nav_filter` 将双深度 raw → `.../points_nav`（Crop+Voxel+SOR+Radius ≤5 Hz，参数见 `pc_nav_filter.yaml`）；local costmap 订 `*_points_nav`
- 仲裁：`/xw/cmd/active_source`；安全门 teleop=一代扇区避障，nav=硬停+后方安全后退
- 定位健康：`xw_localization_health` → `/xw/localization_status`（0 正常 / 1 未就绪 / 2 漂移自愈 / 3 需重定位）；写入 `RobotState.localization_status`
- 会话：`set_mode(2,{map_name})` → `/xw/nav/map_name` + `/xw/nav/enable` → 起 Nav2
- 单点 / 多点 / 初位姿 / 取消 API 不变
- Local costmap：`/scan` + `/camera/front_{up,down}/depth/points_nav`；Global：static + scan only

### CPU（Rock 5T）

- 导航稳态：过滤点云、costmap 不全量 publish、RPP@20 Hz、关可视化
- `xw_health` 记录 TF fail rate、cmd_vel overdue、`localization_code`、`points_nav_*` 存活

### 指令优先级（跟随场景）

1. teleop / motion（调试遥控仍最高）
2. 跟随任务与点位/巡航互斥：开跟随 → 软取消点位；跟随时点位被拒绝
3. 跟随运动（默认）= 视觉伺服 → `/xw/cmd/follow`（优先级高于残余 `/xw/cmd/nav`）
4. 可选 Nav2 打点跟随：`use_nav2_follow:=true` → `/xw/cmd/nav`

### 手推建图要点

- `set_mode(1)` → slam_toolbox；保存仍写 charger 到 pointList  
- HTTP：`POST /api/map`、`POST /api/waypoint`、`POST /api/goal`、`POST /api/initialpose`、`POST /api/nav/patrol`、`POST /api/follow`、`POST /api/recharge`、`POST /api/explore`、`GET /api/sensors`  

### 自主建图要点

- 正交于 **MAPPING**：`/xw/supervisor/set_explore` → `/xw/explore/enable`
- `xw_explore`：启无 AMCL 的探索 Nav2（`static_layer` 跟 SLAM `/map`）+ frontier 节点；速度经 `/xw/cmd/nav`
- 完成发 `/xw/explore/finished` → 会话自动保存地图名并清 latch
- 建图页「开始自主建图」走 `/api/explore`；与手推同屏画布
## 8. 验收（容器内）

1. `colcon build` 通过  
2. `robot.launch.py` mock 可起  
3. teleop → `/cmd_vel` → mock odom  
4. 浏览器连 9000/8765，`set_mode` 互斥可见  
5. 选地图进入导航 → Nav2 起；`/api/initialpose` + `/api/goal` 有 TaskProgress/Result  
6. 设置页：跌倒开关；**跟随 toggle 不关 Nav2**；导航进/出双相机点云自动开/关  
7. 有 `yolov8n-pose.rknn` 时：跌倒可见 `/xw/perception/fall`；跟随可见 `is_target` 锁定  

## 9. Supervisor 与 Session 协作

- `SetMode` 改运动模式并在 `/xw/slam|nav/enable` 上发命令；**跟随**走 `/xw/supervisor/set_follow` / `/xw/follow/enable`；**回充**走 `/xw/supervisor/set_recharge` / `/xw/recharge/enable`；**自主建图**走 `/xw/supervisor/set_explore` / `/xw/explore/enable`（需建图模式）；**跌倒**走 `/xw/supervisor/set_fall` / `/xw/fall/enable`。
- Session 既可订阅 enable，也可暴露 `/xw/session/*/control` 供调试直连。
- **禁止** Supervisor 在 service 回调里 `spin_until_future_complete` 再调 session service（会死锁）。
- 进/出 NAVIGATING/FOLLOWING 时 supervisor 异步调 `/xw/camera/set_pointcloud_nav`（不写 persist）。
- `/xw/nav/cancel` 与跟随抢占只 cancel goal，**禁止**在跟随路径调用 `_stop_nav2`。