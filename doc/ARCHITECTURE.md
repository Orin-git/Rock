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
        → xw_cmd_arbiter
        → xw_safety_gate   (/scan + ultrasonic + depth ROI)
        → /cmd_vel
        → xw_chassis
```

优先级：`teleop > motion > nav > follow > recharge`（MCU 失能见 `/xw/chassis/motor_disabled`，不进速度仲裁）

深度相机（前视 HP60C）：驱动随 `robot.launch.py` **常开**；Web 彩色预览仅在打开 `/pages/camera.html` 时订 compressed 推流。

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
| In | `/xw/goal_pose` | 单点导航目标 |
| In | `/xw/nav/patrol_cmd` | 多点巡航 JSON |
| In | `/xw/nav/cancel` | 取消当前导航 |
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
- **P4（部分）**：**WT901C485 独立 IMU 已接入**（`/imu/data`）；EKF 真融合默认关（`use_ekf:=true` 启用）；回充 / 压测 / 精标定仍待  

### USB 接口分配（Rock 5T）— HP60C 为 USB2.0 设备

Nuwa-HP60C 官方为 **USB2.0 接口**（不会出现 `speed=5000`，属正常）。  
两个蓝色口在 USB2 层共用 **同一条 VIA HS 总线（Bus1，480Mbps）**；两台深度相机都插蓝口会挤爆总线 → 掉线/libusb 崩溃。

**推荐接线（双相机同时开的关键：相机分属不同 USB 主机控制器）：**

| 板载口 | 设备 | 为何 |
|--------|------|------|
| 蓝色 USB3 A | 深度相机 front_up | 走 xhci 的 USB2 伴生（Bus1） |
| 蓝色 USB3 B | 拓展坞 → 底盘+IMU | 串口流量极小，可与 cam1 共享 Bus1 |
| 黑色 USB2 A | 深度相机 front_down | **独占**另一路 EHCI（Bus3 或 Bus5） |
| 黑色 USB2 B | 激光雷达 | **独占**供电/电流 |

插完后执行 `lsusb -t`：两台 `3482:6723` 必须出现在**不同 Bus**（例如一台 Bus1、一台 Bus5），再开双相机。

```bash
# 确认分总线后打开第二路
USE_DEPTH_CAM_2=true sudo systemctl restart xw-robot
```

注意：HP60C SDK **不支持 fps=5**；深度用 **10**。已加 udev 解绑 `uvcvideo`，避免与 ascamera 抢接口。

### 传感器命名契约（Gen2）

| 设备 | 节点 | Frame | 公共话题 |
|------|------|-------|----------|
| 激光雷达 | `rplidar_node` | `lidar_link` | `/scan`（建图/AMCL/避障） |
| 前上深度 | `ascamera_hp60c/...` + bridge | `camera_front_up_link` | `/camera/front_up/{color,depth,...}` |
| 前下深度 | `ascamera_hp60c_2/...` + bridge | `camera_front_down_link` | `/camera/front_down/...` |
| 独立 IMU（WT901C485） | `xw_wt901_imu` | `imu_link` | `/imu/data`（Modbus RTU @9600，slave `0x50`） |
| 底盘轮式里程计 | `xw_chassis` | `base_link` | 默认 `/odom`；EKF 时 `/odom/wheel` |
| EKF 融合（可选） | `ekf_filter_node` | `odom→base_link` | `/odom` |

厂商私有话题 `/ascamera_hp60c{,_2}/...` 仅 bridge 订阅，业务节点只用 `/camera/...`。

**激光前向验收**：正前方挡板 → `/scan` 在 angle≈0 距离变短；否则只改 URDF `lidar_joint` yaw=`π` **或** 驱动 `inverted`（二选一）。

### 传感器职责

| 能力 | 激光 | 深度1 | 深度2 | IMU |
|------|------|-------|-------|-----|
| 建图 SLAM | 是 | 否 | 否 | 间接（稳 odom） |
| AMCL | 是 | 否 | 否 | 间接 |
| Local 避障 | 是 | 点云 | 点云 | 否 |
| 安全门 | 是 | ROI | 后续 | 否 |
| 人体跟随 | 否 | **是** | 否 | 否 |

### 深度相机要点

- 包：`third_party/ascamera` + `xw_sensors/launch/depth_camera.launch.py`  
- 进导航：`/xw/camera/set_pointcloud_nav` 开 #1；#2 镜像 `/xw/camera/pointcloud_enabled`  
- **建图不参与深度**：slam_toolbox 只用 `/scan`  
- raw RGB（#1）：仅在 `fall_en || follow_en` 时转发  

### EKF 空槽（打滑友好）

- 默认 `use_ekf:=false`：底盘直接发 `/odom` + TF  
- `use_ekf:=true` 时：`chassis_odom_topic:=odom/wheel`、`chassis_publish_odom_tf:=false`，加载 `xw_sensors/config/ekf.yaml`  
- 融合：`odom0` 只融 **vx**；`imu0` 只融 **vyaw**（绝对 yaw / aX 待标定后再开）  
- **不用**底盘 MCU IMU 字节；只用中置 `imu_link` → `/imu/data`  
- 标定参考：[imu_utils](https://github.com/gaowenliang/imu_utils)、[Nav2 camera calibration](https://docs.nav2.org/tutorials/docs/camera_calibration.html)  

### 感知 / 跌倒 / 跟随

- 跟随仍用相机1：`/camera/front_up/{color,depth}` → `/xw/cmd/follow`  
- 跌倒正交开关与输出契约不变（`/xw/perception/tracks`、`/xw/perception/fall`）  

### 导航要点

- 参数：`xw_nav_session/config/nav2_params.yaml`（一代 MPPI 移植，`base_link`，`robot_radius: 0.23`）  
- Launch：`xw_nav_session/launch/nav2.launch.py`（localization + navigation；`cmd_vel`→`/xw/cmd/nav`）  
- 会话：`set_mode(2,{map_name})` → `/xw/nav/map_name` + `/xw/nav/enable` → 起 Nav2  
- 单点：`POST /api/goal` → `/xw/goal_pose` → `NavigateToPose`  
- 多点：`POST /api/nav/patrol` → `/xw/nav/patrol_cmd`（读 `*_pointList.yaml`）  
- 初位姿：`POST /api/initialpose` → `/initialpose`  
- 取消：`POST /api/nav/cancel` → `/xw/nav/cancel`  
- Local costmap：`/scan` + `/camera/front_up/depth/points` + `/camera/front_down/depth/points`  

### 手推建图要点

- `set_mode(1)` → slam_toolbox；保存仍写 charger 到 pointList  
- HTTP：`POST /api/map`、`POST /api/waypoint`、`POST /api/goal`、`POST /api/initialpose`、`POST /api/nav/patrol`、`GET /api/sensors`  

## 8. 验收（容器内）

1. `colcon build` 通过  
2. `robot.launch.py` mock 可起  
3. teleop → `/cmd_vel` → mock odom  
4. 浏览器连 9000/8765，`set_mode` 互斥可见  
5. 选地图进入导航 → Nav2 起；`/api/initialpose` + `/api/goal` 有 TaskProgress/Result  
6. 设置页：跌倒开关；跟随；导航进/出双相机点云自动开/关  
7. 有 `yolov8n-pose.rknn` 时：跌倒可见 `/xw/perception/fall`  

## 9. Supervisor 与 Session 协作

- `SetMode` 改运动模式并在 `/xw/slam|nav|follow/enable` 上发命令；**跌倒**走独立 `/xw/supervisor/set_fall` / `/xw/fall/enable`（模式切换不清除）。
- Session 既可订阅 enable，也可暴露 `/xw/session/*/control` 供调试直连。
- **禁止** Supervisor 在 service 回调里 `spin_until_future_complete` 再调 session service（会死锁）。
- 进/出 NAVIGATING 时 supervisor 异步调 `/xw/camera/set_pointcloud_nav`（不写 persist）。