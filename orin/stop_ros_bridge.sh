#!/bin/bash
# Stops the background ROS2 phone video bridge started by start_ros_bridge.sh

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

PID_FILE=".ros_bridge.pid"
LOG_FILE="ros_bridge.log"
LOCAL_PORT="${ROS_BRIDGE_LOCAL_PORT:-9100}"

if [ ! -f "$PID_FILE" ]; then
    echo "ROS bridge PID file not found; attempting best-effort cleanup"
else
    BRIDGE_PID=$(cat "$PID_FILE")
    if ps -p "$BRIDGE_PID" >/dev/null 2>&1; then
        echo "Stopping ROS bridge (PID: $BRIDGE_PID)..."
        kill "$BRIDGE_PID" 2>/dev/null || true
        sleep 0.5
        if ps -p "$BRIDGE_PID" >/dev/null 2>&1; then
            kill -9 "$BRIDGE_PID" 2>/dev/null || true
        fi
        echo "✓ ROS bridge stopped"
    else
        echo "ROS bridge PID $BRIDGE_PID not running"
    fi
    rm -f "$PID_FILE"
fi

if command -v adb >/dev/null 2>&1; then
    adb forward --remove "tcp:${LOCAL_PORT}" >/dev/null 2>&1 || true
fi

echo "Logs: $LOG_FILE"
exit 0
