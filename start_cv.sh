#!/bin/bash
# Final CV end-to-end test (legal Path B) — real camera + LED optical channel.
# PREREQUISITE: run `bash project_setup.sh` once after the 2-lens model change.
set -e

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Mirror everything to run.log so it can be inspected after the run (survives Ctrl-C).
LOG="$REPO/run.log"
exec > >(tee "$LOG") 2>&1
echo "[start-cv] Logging to $LOG"

echo "[start-cv] Killing old processes..."
pkill -9 -f "px4" 2>/dev/null || true
pkill -9 -f "gz sim" 2>/dev/null || true
pkill -9 -f "gazebo" 2>/dev/null || true
pkill -9 -f "cv_swarm_test.py" 2>/dev/null || true
pkill -9 -f "debug_swarm_test.py" 2>/dev/null || true
pkill -9 -f "follower.py" 2>/dev/null || true
pkill -9 -f "mission_launch.py" 2>/dev/null || true
pkill -9 -f "parameter_bridge" 2>/dev/null || true
pkill -9 -f "image_bridge" 2>/dev/null || true
sleep 2
echo "[start-cv] Done killing."

echo "[start-cv] Setting up display..."
export DISPLAY=:0
export WAYLAND_DISPLAY=wayland-0
export XDG_RUNTIME_DIR=/mnt/wslg/runtime-dir
export MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA
export GZ_RENDER_ENGINE=ogre2
export PYTHONUNBUFFERED=1   # show python logs live (ros2 launch buffers otherwise)

echo "[start-cv] Fixing line endings..."
sed -i 's/\r//' "$REPO/resources/scripts/px4_gz_setup.sh"

echo "[start-cv] Loading PX4 + Gazebo environment..."
source "$REPO/resources/scripts/px4_gz_setup.sh"

echo "[start-cv] Loading ROS2..."
source /opt/ros/humble/setup.bash

# Pre-start Gazebo so the heavy baylands world is loaded BEFORE any PX4 instance,
# avoiding the leader gz_bridge spawn race.
WORLD="$PX4_DIR/Tools/simulation/gz/worlds/baylands_custom.sdf"
echo "[start-cv] Starting Gazebo first: $WORLD"
gz sim -v 2 -r "$WORLD" &
GZ_PID=$!
trap 'kill $GZ_PID 2>/dev/null || true' EXIT

echo "[start-cv] Waiting for Gazebo to load the world..."
READY=0
for i in $(seq 1 60); do
    if gz topic -l 2>/dev/null | grep -q "baylands_custom"; then
        READY=1
        echo "[start-cv] Gazebo ready after ${i}s."
        break
    fi
    sleep 1
done
if [ "$READY" -eq 0 ]; then
    echo "[start-cv] WARN: world not detected after 60s, proceeding anyway."
fi
sleep 3

echo "[start-cv] Launching CV end-to-end test..."
echo "[start-cv] CV route: straight train-direction flight only; no turn pattern."
ros2 launch "$REPO/resources/scripts/cv_launch.py"
