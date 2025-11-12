#!/bin/bash
# Quick start script for Orin Target API

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

echo "=================================="
echo "Orin Target API - Quick Start"
echo "=================================="
echo

# Install dependencies if needed
echo "Installing/updating dependencies..."
python3 -m pip install --user -q --upgrade pip
python3 -m pip install --user -q -r requirements.txt
echo "✓ Dependencies installed"

# Check if ROS2 is available
echo
echo "Checking ROS2 availability..."
if [ -f "/opt/ros/humble/setup.bash" ]; then
    echo "✓ ROS2 Humble detected"
    source /opt/ros/humble/setup.bash
    ROS2_FLAG="--ros2"
elif [ -f "/opt/ros/foxy/setup.bash" ]; then
    echo "✓ ROS2 Foxy detected"
    source /opt/ros/foxy/setup.bash
    ROS2_FLAG="--ros2"
else
    echo "⚠ ROS2 not found - running in test mode"
    ROS2_FLAG="--no-ros2"
fi

echo
echo "=================================="
echo "Starting Target API Server"
echo "=================================="
echo "Listening on: http://0.0.0.0:8080"
echo "ROS2 mode: $ROS2_FLAG"
echo
echo "Test from CamViewer app or use:"
echo "  python3 test_target_api.py"
echo
echo "Press Ctrl+C to stop"
echo "=================================="
echo

# Start the server
python3 target_api.py $ROS2_FLAG --port 8080
