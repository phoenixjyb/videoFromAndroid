#!/bin/bash
# Background launcher for the ROS2 phone video bridge
# Starts ros2_camcontrol.ws_to_image via ADB port forwarding and records PID/logs

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

LOG_FILE="ros_bridge.log"
PID_FILE=".ros_bridge.pid"
LOCAL_PORT="${ROS_BRIDGE_LOCAL_PORT:-9100}"
REMOTE_PORT="${ROS_BRIDGE_REMOTE_PORT:-9090}"
BRIDGE_RATE="${ROS_BRIDGE_FRAME_RATE:-10}"
BRIDGE_CODEC="${ROS_BRIDGE_CODEC:-h265}"
BRIDGE_TOPIC="${ROS_BRIDGE_TOPIC:-/recomo/rgb}"
TIMING_INTERVAL="${WS_TIMING_SAMPLE_INTERVAL:-60}"

log() {
    local level="$1"; shift
    echo "$(date '+%Y-%m-%d %H:%M:%S') [$level] $*" | tee -a "$LOG_FILE"
}

if [ -f "$PID_FILE" ]; then
    EXISTING_PID=$(cat "$PID_FILE")
    if ps -p "$EXISTING_PID" >/dev/null 2>&1; then
        log "INFO" "ROS bridge already running (PID: $EXISTING_PID)"
        exit 0
    else
        rm -f "$PID_FILE"
    fi
fi

log "INFO" "Starting ROS bridge (topic: $BRIDGE_TOPIC, codec: $BRIDGE_CODEC, rate: ${BRIDGE_RATE}Hz)"

if ! command -v adb >/dev/null 2>&1; then
    log "ERROR" "adb command not found"
    exit 1
fi

if ! adb devices | grep -q "device$"; then
    log "ERROR" "No ADB device connected"
    exit 1
fi

if ! adb shell "ps -A" | grep -q "com.example.camcontrol"; then
    log "ERROR" "CamControl app is not running on the connected device"
    exit 1
fi

adb forward "tcp:${LOCAL_PORT}" "tcp:${REMOTE_PORT}" >/dev/null

if ! adb forward --list | grep -q "tcp:${LOCAL_PORT}.*tcp:${REMOTE_PORT}"; then
    log "ERROR" "Failed to establish ADB port forwarding ${LOCAL_PORT}->${REMOTE_PORT}"
    exit 1
fi

if ! timeout 3 bash -c "</dev/tcp/127.0.0.1/${LOCAL_PORT}" >/dev/null 2>&1; then
    log "ERROR" "Unable to reach forwarded port 127.0.0.1:${LOCAL_PORT}"
    exit 1
fi

if [ -f "/opt/ros/humble/setup.bash" ]; then
    set +u
    # shellcheck disable=SC1091
    source /opt/ros/humble/setup.bash
    set -u
elif [ -f "/opt/ros/foxy/setup.bash" ]; then
    set +u
    # shellcheck disable=SC1091
    source /opt/ros/foxy/setup.bash
    set -u
else
    log "ERROR" "ROS2 environment not found (install Humble or source manually)"
    exit 1
fi

if [ -f "$SCRIPT_DIR/ros2_camcontrol/install/setup.bash" ]; then
    set +u
    # shellcheck disable=SC1091
    source "$SCRIPT_DIR/ros2_camcontrol/install/setup.bash"
    set -u
fi

export PYTHONPATH="$SCRIPT_DIR/ros2_camcontrol:$SCRIPT_DIR/ros2_camcontrol/build/ros2_camcontrol:${PYTHONPATH:-}"

COMMAND=(python3 -m ros2_camcontrol.ws_to_image \
    --host 127.0.0.1 \
    --port "$LOCAL_PORT" \
    --topic "$BRIDGE_TOPIC" \
    --rate "$BRIDGE_RATE" \
    --codec "$BRIDGE_CODEC" \
    --timing-sample-interval "$TIMING_INTERVAL")

log "INFO" "Launching bridge process..."

env -u http_proxy -u https_proxy -u HTTP_PROXY -u HTTPS_PROXY -u ftp_proxy -u FTP_PROXY \
    nohup "${COMMAND[@]}" >> "$LOG_FILE" 2>&1 &
BRIDGE_PID=$!
echo "$BRIDGE_PID" > "$PID_FILE"

log "INFO" "ROS bridge started (PID: $BRIDGE_PID)"
log "INFO" "Logs: $LOG_FILE"
exit 0
