#!/bin/bash
# Quick setup script for camera control relay on Orin
# Run this after syncing the repo to Orin

set -e

echo "🚀 Camera Control Relay Setup"
echo "==============================="
echo ""

# Check if we're in the right directory
if [ ! -f "camera_control_relay.py" ]; then
    echo "❌ Error: camera_control_relay.py not found"
    echo "Please run this script from the orin/ directory"
    exit 1
fi

# Check Python
echo "📋 Checking Python..."
if ! command -v python3 &> /dev/null; then
    echo "❌ Python 3 not found. Please install Python 3.8+"
    exit 1
fi
PYTHON_VERSION=$(python3 --version)
echo "✅ $PYTHON_VERSION"

# Note: Using pip --user approach (no virtual environment)
# This matches the approach used in other scripts (start_target_api.sh, start_media_api.sh)

# Check ROS2
echo ""
echo "📋 Checking ROS2..."
if [ -z "$ROS_DISTRO" ]; then
    echo "⚠️  ROS2 not sourced. Trying to source..."
    if [ -f "/opt/ros/humble/setup.bash" ]; then
        source /opt/ros/humble/setup.bash
        echo "✅ Sourced ROS2 Humble"
    elif [ -f "/opt/ros/foxy/setup.bash" ]; then
        source /opt/ros/foxy/setup.bash
        echo "✅ Sourced ROS2 Foxy"
    else
        echo "❌ Could not find ROS2. Please install ROS2 and source setup.bash"
        exit 1
    fi
else
    echo "✅ ROS2 $ROS_DISTRO"
fi

# Install Python dependencies
echo ""
echo "📦 Installing Python dependencies..."
pip install --user -r requirements.txt
echo "✅ Dependencies installed (including websockets>=12.0)"

# Check phone connectivity
echo ""
echo "📡 Checking phone connectivity..."
read -p "Enter phone IP address (default: 172.16.30.28): " PHONE_IP
PHONE_IP=${PHONE_IP:-172.16.30.28}

if ping -c 1 -W 2 $PHONE_IP &> /dev/null; then
    echo "✅ Phone is reachable at $PHONE_IP"
else
    echo "⚠️  Warning: Cannot ping phone at $PHONE_IP"
    echo "   Make sure the phone is on the network"
fi

# Test WebSocket port
echo ""
echo "🔌 Testing WebSocket port 9090..."
if timeout 2 bash -c "cat < /dev/null > /dev/tcp/$PHONE_IP/9090" 2>/dev/null; then
    echo "✅ Port 9090 is open"
else
    echo "⚠️  Warning: Cannot connect to port 9090"
    echo "   Make sure camControl app is running on the phone"
fi

# Done
echo ""
echo "✅ Setup complete!"
echo ""
echo "To start the camera control relay:"
echo "  python3 camera_control_relay.py --phone-host $PHONE_IP"
echo ""
echo "To test camera control:"
echo "  ros2 topic pub --once /camera/zoom std_msgs/Float32 \"data: 2.5\""
echo ""
echo "See SETUP_GUIDE.md for detailed instructions."
