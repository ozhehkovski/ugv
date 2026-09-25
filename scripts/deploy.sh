#!/usr/bin/env bash
# Sync ros2_ws/src to the robot and build it there.
# PYTHONNOUSERSITE: a newer setuptools in ~/.local breaks ament_python builds on the Jetson.
#   UGV_HOST=luki@192.168.1.58 ./scripts/deploy.sh [colcon args...]
set -euo pipefail

HOST="${UGV_HOST:-luki@192.168.1.58}"
REMOTE_WS="${UGV_WS:-ugv_ws}"
ROOT="$(cd "$(dirname "$0")/.." && pwd)"

${UGV_SSH:-ssh} "$HOST" "mkdir -p $REMOTE_WS/src"
rsync -e "${UGV_SSH:-ssh}" -az --delete --exclude '__pycache__' --exclude '.pytest_cache' \
  "$ROOT/ros2_ws/src/" "$HOST:$REMOTE_WS/src/"
rsync -e "${UGV_SSH:-ssh}" -az --delete "$ROOT/scripts/" "$HOST:$REMOTE_WS/scripts/"

${UGV_SSH:-ssh} "$HOST" "bash -lc 'set -e; source /opt/ros/humble/setup.bash; cd ~/$REMOTE_WS; \
  PYTHONNOUSERSITE=1 colcon build --symlink-install $* 2>&1 | tail -20'"
