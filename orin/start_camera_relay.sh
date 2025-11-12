#!/bin/bash
# Start camera control relay
# Usage: ./start_camera_relay.sh [phone_ip] [phone_port]

PHONE_IP=${1:-172.16.30.28}
PHONE_PORT=${2:-9090}

# Get the directory where this script is located
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

cd "$SCRIPT_DIR"

# Source ROS2 if not already sourced
if [ -z "$ROS_DISTRO" ]; then
    echo "Sourcing ROS2..."
    source /opt/ros/humble/setup.bash
fi

# Start the relay
echo "🚀 Starting camera control relay..."
echo "📡 Phone: $PHONE_IP:$PHONE_PORT/control"
echo "🔍 ROS2 topics: /camera/*"
echo ""
echo "Note: Starting without proxy environment variables to allow WebSocket connections"

# Start relay without proxy variables (they interfere with WebSocket connections)
env -u http_proxy -u https_proxy -u HTTP_PROXY -u HTTPS_PROXY -u ftp_proxy -u FTP_PROXY \
    python3 camera_control_relay.py --phone-host "$PHONE_IP" --phone-port "$PHONE_PORT"
