#!/bin/bash
# Unified start script for all Orin API services
# Starts Target API (port 8082) and Media API (port 8081) in the background
#
# Usage:
#   ./start_all_services.sh              # Use default network (zerotier)
#   NETWORK_CONFIG=t8space ./start_all_services.sh  # Use T8Space network

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

# Load network configuration
if [ -f "../config/load_network_config.sh" ]; then
    source ../config/load_network_config.sh
    echo "Network: $NETWORK_NAME"
    echo "  Phone IP: $PHONE_IP"
    echo "  Orin IP: $ORIN_IP"
    echo
else
    echo "Warning: Network config not found, using defaults"
    PHONE_IP="${PHONE_IP:-192.168.100.156}"
fi

echo "=========================================="
echo "Orin API Services - Unified Startup"
echo "=========================================="
echo

# Install dependencies if needed
echo "Installing/updating dependencies..."
python3 -m pip install --user -q --upgrade pip
python3 -m pip install --user -q -r requirements.txt
echo "✓ Dependencies installed"
echo

# Check if ROS2 is available for Target API
ROS2_FLAG="--no-ros2"
if [ -f "/opt/ros/humble/setup.bash" ]; then
    echo "✓ ROS2 Humble detected - enabling for Target API"
    source /opt/ros/humble/setup.bash
    ROS2_FLAG="--ros2"
elif [ -f "/opt/ros/foxy/setup.bash" ]; then
    echo "✓ ROS2 Foxy detected - enabling for Target API"
    source /opt/ros/foxy/setup.bash
    ROS2_FLAG="--ros2"
else
    echo "⚠ ROS2 not found - Target API running in test mode"
fi
echo

# Start Target API in background
echo "=========================================="
echo "Starting Target API Server..."
echo "=========================================="
echo "  URL: http://0.0.0.0:${ORIN_TARGET_PORT:-8082}"
echo "  ROS2: $ROS2_FLAG"
echo "  Logs: target_api.log"
nohup python3 target_api.py $ROS2_FLAG --port ${ORIN_TARGET_PORT:-8082} > target_api.log 2>&1 &
TARGET_PID=$!
echo "  PID: $TARGET_PID"
echo "✓ Target API started"
echo

# Wait a moment for Target API to start
sleep 2

# Start Media API in background
echo "=========================================="
echo "Starting Media API Server..."
echo "=========================================="
echo "  URL: http://0.0.0.0:${ORIN_MEDIA_PORT:-8081}"
echo "  Media dir: ./media/"
echo "  Logs: media_api.log"
nohup python3 media_api.py --port ${ORIN_MEDIA_PORT:-8081} > media_api.log 2>&1 &
MEDIA_PID=$!
echo "  PID: $MEDIA_PID"
echo "✓ Media API started"
echo

# Save PIDs for easy stopping
echo "$TARGET_PID" > .target_api.pid
echo "$MEDIA_PID" > .media_api.pid

# Start Camera Relay in background (ROS2 to WebSocket bridge)
if [ "$ROS2_FLAG" = "--ros2" ]; then
    echo "=========================================="
    echo "Starting Camera Relay..."
    echo "=========================================="
    echo "  Phone: ${PHONE_IP}:9090/control"
    echo "  ROS2 topics: /camera/*"
    echo "  Logs: camera_relay.log"
    env -u http_proxy -u https_proxy -u HTTP_PROXY -u HTTPS_PROXY -u ftp_proxy -u FTP_PROXY \
        nohup python3 camera_control_relay.py --phone-host "${PHONE_IP}" --phone-port 9090 > camera_relay.log 2>&1 &
    RELAY_PID=$!
    echo "  PID: $RELAY_PID"
    echo "✓ Camera Relay started"
    echo "$RELAY_PID" > .camera_relay.pid
    echo
else
    echo "⚠ Skipping Camera Relay (ROS2 not available)"
    echo
fi

echo "=========================================="
echo "✓ All services started successfully!"
echo "=========================================="
echo
echo "Services running:"
echo "  Target API: http://$(hostname -I | awk '{print $1}'):8082"
echo "  Media API:  http://$(hostname -I | awk '{print $1}'):8081"
if [ "$ROS2_FLAG" = "--ros2" ]; then
    echo "  Camera Relay: ROS2 /camera/* ↔ ${PHONE_IP}:9090"
fi
echo
echo ""
echo "To stop all services, run:"
echo "  ./stop_all_services.sh"
echo ""
echo
echo "To view logs:"
echo "  tail -f target_api.log"
echo "  tail -f media_api.log"
echo
echo "To check status:"
echo "  ps aux | grep -E 'target_api|media_api' | grep -v grep"
echo "=========================================="
