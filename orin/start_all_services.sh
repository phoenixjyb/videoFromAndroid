#!/bin/bash
# Unified start script for all Orin API services
# Starts Target API (port 8080) and Media API (port 8081) in the background

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

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
echo "  URL: http://0.0.0.0:8080"
echo "  ROS2: $ROS2_FLAG"
echo "  Logs: target_api.log"
nohup python3 target_api.py $ROS2_FLAG --port 8080 > target_api.log 2>&1 &
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
echo "  URL: http://0.0.0.0:8081"
echo "  Media dir: ./media/"
echo "  Logs: media_api.log"
nohup python3 media_api.py > media_api.log 2>&1 &
MEDIA_PID=$!
echo "  PID: $MEDIA_PID"
echo "✓ Media API started"
echo

# Save PIDs for easy stopping
echo "$TARGET_PID" > .target_api.pid
echo "$MEDIA_PID" > .media_api.pid

echo "=========================================="
echo "✓ All services started successfully!"
echo "=========================================="
echo
echo "Services running:"
echo "  Target API: http://$(hostname -I | awk '{print $1}'):8080"
echo "  Media API:  http://$(hostname -I | awk '{print $1}'):8081"
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
