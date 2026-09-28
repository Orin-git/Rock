#!/bin/bash
set -u
export DEBIAN_FRONTEND=noninteractive
MARK=/ros2_ws/mon_20260928/LOAD_MARKER.log
LOG=/tmp/rtabmap_install.log
exec > >(tee -a "$LOG") 2>&1

echo "########## 0. 身份与前置 ##########"
id
df -h /ros2_ws | tail -1
echo "--- 装前已装? ---"; dpkg -l | grep -c rtabmap || true

echo
echo "$(date -u +%FT%TZ) LOAD_WINDOW_START rtabmap 装包（apt-get update + install，不 upgrade）" >> "$MARK"

echo "########## 1. apt-get update ##########"
apt-get update
echo "update EXIT=$?"

echo
echo "########## 2. 装前快照：可升级包数（只记录，不动） ##########"
apt list --upgradable 2>/dev/null | wc -l

echo
echo "########## 3. 安装 ros-humble-rtabmap-ros（【只此一个】，不 upgrade） ##########"
apt-get install -y --no-install-recommends ros-humble-rtabmap-ros
echo "install EXIT=$?"

echo
echo "$(date -u +%FT%TZ) LOAD_WINDOW_END rtabmap 装包结束" >> "$MARK"

echo "########## 4. 装后核对 ##########"
dpkg -l | grep -i rtabmap | awk '{print $1, $2, $3}'
echo "--- 确认没有顺手升级任何已有包 ---"
apt list --upgradable 2>/dev/null | wc -l
