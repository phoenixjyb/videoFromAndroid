#!/bin/bash
set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo -e "${GREEN}=== CamControl Quick Start ===${NC}"

# Ensure adb is available
if ! command -v adb >/dev/null 2>&1; then
    echo -e "${RED}ERROR: adb command not found${NC}"
    echo "Install Android platform tools (e.g. sudo apt install android-tools-adb) and retry"
    exit 1
fi

# Disable proxies for localhost connections
export no_proxy='localhost,127.0.0.1,*'
export NO_PROXY='localhost,127.0.0.1,*'
unset http_proxy https_proxy HTTP_PROXY HTTPS_PROXY all_proxy ALL_PROXY

# Check if ADB device is connected
echo -e "\n${YELLOW}Checking ADB connection...${NC}"
if ! adb devices | grep -q "device$"; then
    echo -e "${RED}ERROR: No ADB device connected${NC}"
    echo "Please connect your Android device via USB"
    exit 1
fi
echo -e "${GREEN}✓ ADB device connected${NC}"

# Check if app is running
echo -e "\n${YELLOW}Checking if CamControl app is running...${NC}"
if ! adb shell "ps -A | grep -q com.example.camcontrol"; then
    echo -e "${RED}ERROR: CamControl app is not running${NC}"
    echo "Please start the CamControl app on your Android device"
    exit 1
fi
echo -e "${GREEN}✓ CamControl app is running${NC}"

# Set up port forwarding
echo -e "\n${YELLOW}Setting up ADB port forwarding...${NC}"
adb forward tcp:9100 tcp:9090
if adb forward --list | grep -q "9100.*9090"; then
    echo -e "${GREEN}✓ Port forwarding active: localhost:9100 -> device:9090${NC}"
else
    echo -e "${RED}ERROR: Port forwarding failed${NC}"
    exit 1
fi

# Test connection
echo -e "\n${YELLOW}Testing connection to Android app...${NC}"
if timeout 3 bash -c "echo > /dev/tcp/127.0.0.1/9100" 2>/dev/null; then
    echo -e "${GREEN}✓ Connection successful${NC}"
else
    echo -e "${RED}ERROR: Cannot connect to localhost:9100${NC}"
    echo "Make sure the app is streaming video"
    exit 1
fi

# Source ROS2 environment
echo -e "\n${YELLOW}Setting up ROS2 environment...${NC}"
source /opt/ros/humble/setup.bash
cd /home/nvidia/videoFromAndroid/orin/ros2_camcontrol
source install/setup.bash
echo -e "${GREEN}✓ ROS2 environment ready${NC}"

# Start the ROS2 node
echo -e "\n${GREEN}Starting ROS2 image publisher node...${NC}"
echo -e "${YELLOW}Publishing to topics:${NC}"
echo "  - /recomo/rgb (sensor_msgs/Image)"
echo "  - /recomo/camera_info (sensor_msgs/CameraInfo)"
echo -e "\n${YELLOW}Press Ctrl+C to stop${NC}\n"

TIMING_INTERVAL=${WS_TIMING_SAMPLE_INTERVAL:-60}
WS_TIMING_SAMPLE_INTERVAL=${TIMING_INTERVAL} \
PYTHONPATH=/home/nvidia/videoFromAndroid/orin/ros2_camcontrol:/home/nvidia/videoFromAndroid/orin/ros2_camcontrol/build/ros2_camcontrol:$PYTHONPATH \
exec python3 -m ros2_camcontrol.ws_to_image \
    --host 127.0.0.1 \
    --port 9100 \
    --topic /recomo/rgb \
    --rate 10 \
    --codec h265 \
    --timing-sample-interval "${TIMING_INTERVAL}" \
    "$@"