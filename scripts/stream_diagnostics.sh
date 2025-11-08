#!/bin/bash
set -euo pipefail

# Simple helper to restart the quick_start pipeline, capture basic diagnostics,
# and clean everything up automatically.

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
LOG_FILE="/tmp/quick_start.log"
PID_FILE="/tmp/quick_start.pid"
TOPIC="/recomo/rgb"
SAMPLE_INTERVAL="${WS_TIMING_SAMPLE_INTERVAL:-10}"
ROS_WS_DIR="$ROOT_DIR/orin/ros2_camcontrol"

cleanup() {
    if [[ -f "${PID_FILE}" ]]; then
        if kill -0 "$(<"${PID_FILE}")" 2>/dev/null; then
            kill "$(<"${PID_FILE}")" 2>/dev/null || true
            wait "$(<"${PID_FILE}")" 2>/dev/null || true
        fi
        rm -f "${PID_FILE}"
    fi
}
trap cleanup EXIT

# Ensure previous run is stopped
cleanup

echo "[1/5] Starting quick_start.sh with timing interval ${SAMPLE_INTERVAL}"
WS_TIMING_SAMPLE_INTERVAL="${SAMPLE_INTERVAL}" \
    bash "${ROOT_DIR}/quick_start.sh" "$@" >"${LOG_FILE}" 2>&1 &
printf '%s' "$!" > "${PID_FILE}"

sleep 5
PID="$(<"${PID_FILE}")"
echo "[2/5] quick_start.sh running with PID ${PID}"

if ! kill -0 "${PID}" 2>/dev/null; then
    echo "Process exited unexpectedly; recent log:" >&2
    tail -n 60 "${LOG_FILE}" >&2 || true
    exit 1
fi

echo "[3/5] Process stats"
ps -p "${PID}" -o pid,ppid,%cpu,%mem,cmd

echo "[4/5] Measuring topic rate (${TOPIC}) for ~15s"
if [[ -f "${ROS_WS_DIR}/install/setup.bash" ]]; then
    # Temporarily disable nounset so ROS setup scripts can reference
    # optional environment variables without tripping set -u.
    set +u
    source /opt/ros/humble/setup.bash
    source "${ROS_WS_DIR}/install/setup.bash"
    set -u
    timeout 15 ros2 topic hz "${TOPIC}" --window 50 || true
else
    echo "ROS2 workspace not built; skipping ros2 topic hz" >&2
fi

echo "[5/5] Recent log excerpt ($LOG_FILE)"
tail -n 200 "${LOG_FILE}" || true

echo "Diagnostics complete; stopping quick_start"
cleanup
