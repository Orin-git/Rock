#!/usr/bin/env bash
# Install boot autostart for Gen2 web/robot stack (host systemd).
# Run on host: bash /home/radxa/ros2_ws/scripts/install_autostart.sh
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
UNIT_SRC="${ROOT}/deploy/xw-robot.service"
UNIT_DST="/etc/systemd/system/xw-robot.service"

chmod +x "${ROOT}/scripts/start_robot_host.sh" "${ROOT}/scripts/start_robot.sh"

if [[ ! -f "$UNIT_SRC" ]]; then
  echo "missing $UNIT_SRC"
  exit 1
fi

echo "Installing $UNIT_DST"
sudo cp "$UNIT_SRC" "$UNIT_DST"
sudo systemctl daemon-reload
sudo systemctl enable xw-robot.service
echo "Starting xw-robot.service ..."
sudo systemctl restart xw-robot.service
sleep 2
sudo systemctl --no-pager --full status xw-robot.service || true
echo
echo "Boot autostart enabled. Check:"
echo "  systemctl status xw-robot"
echo "  journalctl -u xw-robot -f"
echo "  curl http://127.0.0.1:9000/api/health"
echo "Web: http://<board-ip>:9000"
