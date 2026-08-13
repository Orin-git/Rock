#!/usr/bin/env bash
# Backward-compatible wrapper → install_robot_udev.sh
exec "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/install_robot_udev.sh"
