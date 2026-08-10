# Rock 5T / Xiaowei Gen2 workspace

## Layout
- `src/xw_*` — layered ROS 2 packages (see `doc/ARCHITECTURE.md`)
- `scripts/` — `ros_env.sh`, `start_robot.sh`, preflight/watchdog
- `maps/`, `log/` — runtime data
- `docker/` — optional Dockerfile / compose notes
- `vs_ws1` (host, not in workspace) — Gen1 reference only

## Docker (default)

```bash
docker exec -it ros2_humble_dev bash
source /ros2_ws/scripts/ros_env.sh
cd /ros2_ws && colcon build --symlink-install
source install/setup.bash
ros2 launch xw_bringup robot.launch.py
```

- Web: `http://<board-ip>:9000`
- Foxglove: `ws://<board-ip>:8765` (if `foxglove_bridge` installed)

## Quick checks

```bash
ros2 topic echo /xw/robot_state --once
ros2 service call /xw/supervisor/set_mode xw_interfaces/srv/SetMode \
  "{mode: 1, payload_json: '{}', command_id: 'cli'}"
ros2 topic pub --rate 10 /xw/cmd/teleop geometry_msgs/msg/Twist \
  "{linear: {x: 0.1}, angular: {z: 0.0}}"
```


## Docker 日常用法（必看）

容器精简镜像默认没有 `htop`，且 **未 source 环境时 `ros2` 会找不到命令**。

```bash
# 进入容器
docker exec -it ros2_humble_dev bash

# 每次新开 shell 必须加载环境（Domain 默认 99）
source /ros2_ws/scripts/ros_env.sh

# 可选：装进程查看
apt-get update && apt-get install -y htop

# 启动基座
ros2 launch xw_bringup robot.launch.py use_foxglove:=false

# 另一终端调试（同样要 source）
docker exec -it ros2_humble_dev bash
source /ros2_ws/scripts/ros_env.sh
ros2 topic list
ros2 topic echo /xw/robot_state
```

网页打开 `http://<板子IP>:9000`，右上角应显示 **ROS 桥在线 · D99**（不再依赖 Foxglove 8765）。

## 开机自启（Web + 全栈）

宿主机执行一次：

```bash
bash /home/radxa/ros2_ws/scripts/install_autostart.sh
```

之后每次开机：Docker 容器 + `xw-robot.service` 自动 `ros2 launch`，网页 `http://<IP>:9000`。

```bash
systemctl status xw-robot
journalctl -u xw-robot -f
```
