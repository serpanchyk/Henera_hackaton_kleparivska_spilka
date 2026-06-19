#!/bin/bash
set -e

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "[start] Killing old processes..."
pkill -9 -f "px4" 2>/dev/null || true
pkill -9 -f "gz sim" 2>/dev/null || true
pkill -9 -f "gazebo" 2>/dev/null || true
pkill -9 -f "follower.py" 2>/dev/null || true
pkill -9 -f "mission_launch.py" 2>/dev/null || true
sleep 2
echo "[start] Done killing."

echo "[start] Setting up display..."
export DISPLAY=:0
export WAYLAND_DISPLAY=wayland-0
export XDG_RUNTIME_DIR=/mnt/wslg/runtime-dir
export MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA
export GZ_RENDER_ENGINE=ogre2

echo "[start] Fixing line endings..."
sed -i 's/\r//' "$REPO/resources/scripts/px4_gz_setup.sh"

echo "[start] Loading PX4 + Gazebo environment..."
source "$REPO/resources/scripts/px4_gz_setup.sh"

echo "[start] Loading ROS2..."
source /opt/ros/humble/setup.bash

echo "[start] Launching simulation..."
ros2 launch "$REPO/resources/scripts/solution_launch.py"
