#!/usr/bin/env bash
# Wire auto source of ros_env into running ros2_humble_dev container.
set -e
CONTAINER="${1:-ros2_humble_dev}"
MARKER="# >>> xiaowei ros_env >>>"

if ! docker ps --format '{{.Names}}' | grep -qx "$CONTAINER"; then
  echo "Container $CONTAINER is not running"
  exit 1
fi

docker exec -u root "$CONTAINER" bash -c "
set -e
MARKER='$MARKER'
# root interactive bashrc
for f in /root/.bashrc /home/*/.bashrc; do
  [ -f \"\$f\" ] || continue
  if grep -qF \"\$MARKER\" \"\$f\" 2>/dev/null; then
    echo \"already installed in \$f\"
    continue
  fi
  cat >> \"\$f\" << 'SNIP'

$MARKER
# Auto-load Xiaowei Gen2 ROS env when entering interactive shell
if [ -f /ros2_ws/scripts/ros_env.sh ]; then
  # shellcheck disable=SC1091
  source /ros2_ws/scripts/ros_env.sh
fi
# <<< xiaowei ros_env <<<
SNIP
  echo \"installed into \$f\"
done
# also bash global for any user (optional safety)
if [ -f /etc/bash.bashrc ] && ! grep -qF \"\$MARKER\" /etc/bash.bashrc; then
  cat >> /etc/bash.bashrc << 'SNIP'

$MARKER
if [ -n \"\${PS1:-}\" ] && [ -f /ros2_ws/scripts/ros_env.sh ]; then
  # shellcheck disable=SC1091
  source /ros2_ws/scripts/ros_env.sh
fi
# <<< xiaowei ros_env <<<
SNIP
  echo 'installed into /etc/bash.bashrc'
fi
"
echo "Done. New shells in $CONTAINER will auto-load ROS env."
