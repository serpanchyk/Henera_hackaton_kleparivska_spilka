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

# ── Leader mission selection ────────────────────────────────────────────────
# Fly a converted mission JSON (resources/scripts/Mission/<file>) instead of the
# built-in turn pattern. Change the mission without editing Python:
#   bash start_cv.sh                       # default: mission_01.json
#   bash start_cv.sh mission_02.json       # pick another file in Mission/
#   MISSION_FILE=/abs/path.json bash start_cv.sh   # or an absolute path
MISSION_DIR="$REPO/resources/scripts/Mission"
MISSION="${1:-${MISSION:-mission_01.json}}"
# Allow either a bare filename in Mission/ or an absolute/explicit path.
if [ -f "$MISSION" ]; then
    MISSION_FILE="$MISSION"
else
    MISSION_FILE="$MISSION_DIR/$MISSION"
fi
if [ ! -f "$MISSION_FILE" ]; then
    echo "[start-cv] ERROR: mission file not found: $MISSION_FILE"
    echo "[start-cv] Available missions in $MISSION_DIR:"
    ls -1 "$MISSION_DIR"/*.json 2>/dev/null || echo "  (none)"
    exit 1
fi

export LEADER_MISSION_FILE="$MISSION_FILE"
export WATCHDOG_S="${WATCHDOG_S:-900}"        # long routes need more than the 300s default
export LEADER_YAW_MODE="${LEADER_YAW_MODE:-file}"  # file = JSON yaw, path = face leg, current = spawn yaw

echo "[start-cv] Launching CV end-to-end test..."
echo "[start-cv] CV route: mission file -> $LEADER_MISSION_FILE (yaw_mode=$LEADER_YAW_MODE, watchdog=${WATCHDOG_S}s)"
ros2 launch "$REPO/resources/scripts/cv_launch.py"
