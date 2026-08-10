# Docker 开发约定

现用容器名：`ros2_humble_dev`（`ros:humble`）

```text
--net=host --privileged -v /dev:/dev -v $HOME/ros2_ws:/ros2_ws
# 可选一代只读： -v $HOME/vs_ws1:/vs_ws1:ro
```

容器内路径：`/ros2_ws`（= 宿主机 `~/ros2_ws`）

```bash
docker exec -it ros2_humble_dev bash
source /ros2_ws/scripts/ros_env.sh
cd /ros2_ws && colcon build --symlink-install
ros2 launch xw_bringup robot.launch.py
```

端口（host 网络）：Web `9000`，Foxglove `8765`（需安装 `ros-humble-foxglove-bridge`）。
