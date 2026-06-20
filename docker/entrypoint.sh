#!/usr/bin/env bash
set -euo pipefail

if [[ -f /opt/ros/humble/setup.bash ]]; then
    source /opt/ros/humble/setup.bash
fi

cd "${FALCON_GAZE_DIR:-/opt/falcon-gaze}"
exec "$@"
