#!/bin/bash
# Start camera control relay with virtual environment
# Usage: ./start_camera_relay.sh [phone_ip]

PHONE_IP=${1:-172.16.30.28}

# Get the directory where this script is located
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

cd "$SCRIPT_DIR"

# Activate virtual environment if it exists
if [ -d ".venv" ]; then
    source .venv/bin/activate
    echo "✅ Virtual environment activated"
else
    echo "⚠️  Virtual environment not found. Run setup_camera_relay.sh first."
    exit 1
fi

# Start the relay
echo "🚀 Starting camera control relay..."
echo "📡 Phone: $PHONE_IP:8080/control"
echo "🔍 ROS2 topics: /camera/*"
echo ""
python3 camera_control_relay.py --phone-host "$PHONE_IP"
