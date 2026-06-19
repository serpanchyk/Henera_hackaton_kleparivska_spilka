#!/bin/bash
# DEBUG / EVAL ONLY — ground-truth follower test (not for submission).
set -e

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "[start-debug] Killing old processes..."
pkill -9 -f "px4" 2>/dev/null || true
pkill -9 -f "gz sim" 2>/dev/null || true
pkill -9 -f "gazebo" 2>/dev/null || true
pkill -9 -f "debug_swarm_test.py" 2>/dev/null || true
pkill -9 -f "follower.py" 2>/dev/null || true
pkill -9 -f "mission_launch.py" 2>/dev/null || true
sleep 2
echo "[start-debug] Done killing."

echo "[start-debug] Setting up display..."
export DISPLAY=:0
export WAYLAND_DISPLAY=wayland-0
export XDG_RUNTIME_DIR=/mnt/wslg/runtime-dir
export MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA
export GZ_RENDER_ENGINE=ogre2

echo "[start-debug] Fixing line endings..."
sed -i 's/\r//' "$REPO/resources/scripts/px4_gz_setup.sh"

echo "[start-debug] Loading PX4 + Gazebo environment..."
source "$REPO/resources/scripts/px4_gz_setup.sh"

echo "[start-debug] Loading ROS2..."
source /opt/ros/humble/setup.bash

# Pre-start Gazebo server+GUI so the heavy baylands world is fully loaded BEFORE
# any PX4 instance. Otherwise instance 0 races its own gz_bridge spawn against
# Gazebo startup and dies with "gz_bridge Service call timed out".
WORLD="$PX4_DIR/Tools/simulation/gz/worlds/baylands_custom.sdf"
echo "[start-debug] Starting Gazebo first: $WORLD"
gz sim -v 2 -r "$WORLD" &
GZ_PID=$!
trap 'kill $GZ_PID 2>/dev/null || true' EXIT

echo "[start-debug] Waiting for Gazebo to load the world..."
READY=0
for i in $(seq 1 60); do
    if gz topic -l 2>/dev/null | grep -q "baylands_custom"; then
        READY=1
        echo "[start-debug] Gazebo ready after ${i}s."
        break
    fi
    sleep 1
done
if [ "$READY" -eq 0 ]; then
    echo "[start-debug] WARN: world not detected after 60s, proceeding anyway."
fi
sleep 3

echo "[start-debug] Launching DEBUG simulation (ground-truth follower test)..."
ros2 launch "$REPO/resources/scripts/debug_launch.py"
