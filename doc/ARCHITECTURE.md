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

- **P0**：mock 栈 + FSM + Web SPA  
- **P1**：真底盘（`xw_chassis` 串口 `/dev/chassis` 或回退 `/dev/ttyACM0` @115200，一代 0x7B 协议；udev：`scripts/install_chassis_udev.sh`）/ 雷达 / 超声（超声仍待）  
- **P2**：手推建图已落地；**导航 Web 壳 + `/xw/goal_pose` 链路已通**，Nav2/AMCL 仍待接入  
- **P3（部分）**：前视深度驱动 + `/camera/front/...` + 安全门深度 ROI；**感知已换真节点**（YOLOv8n-pose RKNN → tracks/fall）  
- **P4**：回充 / 压测 / 可选 Gateway  

### 传感器命名契约（Gen2）

| 设备 | 节点 | Frame | 公共话题 |
|------|------|-------|----------|
| 激光雷达 | `rplidar_node` | `lidar_link` | `/scan` |
| 前视深度 #1 | `ascamera_hp60c/camera_publisher` + `xw_depth_topic_bridge` | `camera_front_link` | `/camera/front/{color,depth}/...` |
| 前视深度 #2 | `ascamera_hp60c_2/camera_publisher` + `xw_depth_topic_bridge_front_2` | `camera_front_2_link` | `/camera/front_2/{color,depth}/...` |
| 底盘 | `xw_chassis` | `base_link`（odom→base） | `/odom` `/cmd_vel` `/xw/power` |

厂商私有话题 `/ascamera_hp60c{,_2}/...` 仅 bridge 订阅，业务节点只用 `/camera/...`。

### 深度相机要点

- 包：`third_party/ascamera` + `xw_sensors/launch/depth_camera.launch.py`（`config:=depth_camera.yaml` / `depth_camera_front_2.yaml`）  
- `use_depth_cam:=true` / `use_depth_cam_2:=true`；USB 用 `usb_bus_no` + `usb_path` 钉死（换口需改 yaml）  
- 预览：Foxglove Image 订 `/camera/front/.../compressed` 或 `/camera/front_2/.../compressed`  
- 深度 #1：`/camera/front/depth/image_raw`（安全门 ROI + 感知）；**点云默认关**  
- 深度 #2：bring-up 完成；感知/安全门仍用 #1（导航避障 / 人脸后续）  
- 点云（仅 #1）：进导航自动开（`/xw/camera/set_pointcloud_nav`）；设置页手动 persist  
- 服务：`/xw/camera/set_pointcloud`；`/xw/camera/set_pointcloud_nav`；状态：`/xw/camera/pointcloud_enabled`  
- **建图不参与深度**：slam_toolbox 仍只用 `/scan`  
- raw RGB（#1）：仅在 `fall_en || follow_en` 时由 bridge 转发  

### 感知 / 跌倒 / 跟随

- 节点：`xw_perception` → `person_perception_node`（替换 stub）  
- 模型：`xw_perception/models/yolov8n-pose.rknn`（见 `models/README.md` / `convert_rknn.sh`）  
- 容器依赖：`scripts/install_perception_deps.sh`（rknnlite cp310 + librknnrt + opencv）  
- 推理：约 **5–6 FPS**；跌倒几何去抖约 **9 帧（≤2s）**  
- 输出契约不变：`/xw/perception/tracks`、`/xw/perception/fall`  
- **跌倒为正交开关**：`POST /api/fall` → `/xw/supervisor/set_fall` → `/xw/fall/enable`（可与 IDLE/导航并存；`set_mode(4)` 仍兼容）  
- **跟随**：`set_mode(3)` → `/xw/follow/enable`（与建图/导航互斥）；距离用深度中位数  
- 订阅：`/camera/front/{color,depth}/image_raw`（frame `camera_front_link`）  

### 导航 Web 要点（壳 + 链路）

- 页：`/pages/navigation.html`（对照一代控制台布局）  
- 画布：复用 `map_canvas.js` → Foxglove `/map` + `/scan` + TF；可 `mapManage(5)` 静态预览  
- 会话：`set_mode(2, {map_name})` → `/xw/nav/enable`；结束 `set_mode(0)`  
- 目标：`POST /api/goal` → 发布 `/xw/goal_pose` → `xw_nav_session`（现 stub 回 TaskProgress/Result）  
- 传感器面板：`GET /api/sensors`（激光/深度在线探测；超声/IMU/底盘/架位为 URDF 占位，后续只填契约）  
- **未做**：真实 Nav2、AMCL 初始位姿、多点巡航执行、自动回充  

### 手推建图要点

- `set_mode(1)` → `xw_slam_session` 自管启停 `async_slam_toolbox_node`，抓取 `map→base_link` 起点  
- 保存：`/xw/map/manage` op=1 → `map_saver_cli` → upsert `waypoints/{name}_pointList.yaml` 的 `charger`（yaw = tf_yaw+π）  
- 未保存停止 → `autosave_YYYYMMDD_HHMMSS`  
- Web：`/pages/mapping.html`（Foxglove `/map`+`/scan`），`/pages/maps.html`（批量 CRUD，级联 pointList）  
- 导航 Web：`/pages/navigation.html` · `POST /api/goal` · `GET /api/sensors`  
- HTTP：`POST /api/map`、`POST /api/waypoint`、`POST /api/goal`、`GET /api/sensors`  

## 8. 验收（容器内）

1. `colcon build` 通过  
2. `robot.launch.py` mock 可起  
3. teleop → `/cmd_vel` → mock odom  
4. 浏览器连 9000/8765，`set_mode` 互斥可见  
5. 设置页：跌倒开关 → `/xw/fall/enable`；跟随 `set_mode(3)`；导航进/出点云自动开/关  
6. 有 `yolov8n-pose.rknn` 时：开启跌倒后 `ros2 topic echo /xw/perception/fall` 可见几何去抖结果  

## 9. Supervisor 与 Session 协作

- `SetMode` 改运动模式并在 `/xw/slam|nav|follow/enable` 上发命令；**跌倒**走独立 `/xw/supervisor/set_fall` / `/xw/fall/enable`（模式切换不清除）。
- Session 既可订阅 enable，也可暴露 `/xw/session/*/control` 供调试直连。
- **禁止** Supervisor 在 service 回调里 `spin_until_future_complete` 再调 session service（会死锁）。
- 进/出 NAVIGATING 时 supervisor 异步调 `/xw/camera/set_pointcloud_nav`（不写 persist）。