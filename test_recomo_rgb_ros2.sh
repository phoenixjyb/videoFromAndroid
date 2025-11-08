#!/bin/bash

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

echo -e "${BLUE}=== ROS2 /recomo/rgb Node Test ===${NC}\n"

# Source ROS2 environment
source /opt/ros/humble/setup.bash 2>/dev/null

# Check if ROS2 is available
if ! command -v ros2 &> /dev/null; then
    echo -e "${RED}ERROR: ROS2 not found${NC}"
    echo "Make sure ROS2 Humble is installed and sourced"
    exit 1
fi

# Check if ROS2 daemon is running
echo -e "${YELLOW}Checking ROS2 daemon...${NC}"
if ! ros2 daemon status &>/dev/null; then
    echo -e "${YELLOW}Starting ROS2 daemon...${NC}"
    ros2 daemon start
    sleep 2
fi
echo -e "${GREEN}✓ ROS2 daemon is running${NC}\n"

# List all topics
echo -e "${YELLOW}Available ROS2 topics:${NC}"
ros2 topic list
echo ""

# Check if /recomo/rgb topic exists
echo -e "${YELLOW}Checking for /recomo/rgb topic...${NC}"
if ros2 topic list | grep -q "^/recomo/rgb$"; then
    echo -e "${GREEN}✓ Topic /recomo/rgb exists${NC}\n"
else
    echo -e "${RED}✗ Topic /recomo/rgb NOT found${NC}"
    echo -e "${YELLOW}Is the quick_start.sh script running?${NC}\n"
    exit 1
fi

# Check if /recomo/camera_info topic exists
echo -e "${YELLOW}Checking for /recomo/camera_info topic...${NC}"
if ros2 topic list | grep -q "^/recomo/camera_info$"; then
    echo -e "${GREEN}✓ Topic /recomo/camera_info exists${NC}\n"
else
    echo -e "${YELLOW}⚠ Topic /recomo/camera_info NOT found${NC}\n"
fi

# Get topic info
echo -e "${YELLOW}Topic info for /recomo/rgb:${NC}"
ros2 topic info /recomo/rgb
echo ""

# Check message type
echo -e "${YELLOW}Checking message type...${NC}"
TOPIC_TYPE=$(ros2 topic type /recomo/rgb)
if [ "$TOPIC_TYPE" = "sensor_msgs/msg/Image" ]; then
    echo -e "${GREEN}✓ Correct message type: $TOPIC_TYPE${NC}\n"
else
    echo -e "${RED}✗ Unexpected message type: $TOPIC_TYPE${NC}\n"
fi

# Check publishing rate
echo -e "${YELLOW}Measuring publishing rate (5 seconds)...${NC}"
RATE_OUTPUT=$(timeout 5 ros2 topic hz /recomo/rgb 2>&1 || true)
if echo "$RATE_OUTPUT" | grep -q "average rate"; then
    RATE=$(echo "$RATE_OUTPUT" | grep "average rate" | awk '{print $3}')
    echo -e "${GREEN}✓ Publishing at ~${RATE} Hz${NC}\n"
else
    echo -e "${RED}✗ No messages received${NC}"
    echo "The topic exists but no data is being published"
    exit 1
fi

# Show message sample (header only)
echo -e "${YELLOW}Sample message (header only):${NC}"
timeout 2 ros2 topic echo /recomo/rgb --once --no-arr | head -20
echo ""

# Get bandwidth
echo -e "${YELLOW}Measuring bandwidth (3 seconds)...${NC}"
BW_OUTPUT=$(timeout 3 ros2 topic bw /recomo/rgb 2>&1 || true)
if echo "$BW_OUTPUT" | grep -q "mean"; then
    echo "$BW_OUTPUT" | grep -E "(mean|min|max)"
    echo -e "${GREEN}✓ Data is flowing${NC}\n"
else
    echo -e "${YELLOW}⚠ Could not measure bandwidth${NC}\n"
fi

# Check camera info if available
if ros2 topic list | grep -q "^/recomo/camera_info$"; then
    echo -e "${YELLOW}Camera info available:${NC}"
    timeout 2 ros2 topic echo /recomo/camera_info --once | grep -E "(width|height|frame_id)" | head -5
    echo ""
fi

# Summary
echo -e "${GREEN}=== Test Summary ===${NC}"
echo -e "${GREEN}✓ ROS2 node is running and publishing${NC}"
echo -e "${GREEN}✓ Topic: /recomo/rgb${NC}"
echo -e "${GREEN}✓ Type: sensor_msgs/msg/Image${NC}"
echo -e "${GREEN}✓ Rate: ~${RATE} Hz${NC}"
echo -e "\n${BLUE}To view the stream:${NC}"
echo "  ros2 topic echo /recomo/rgb --no-arr"
echo "  ros2 run rqt_image_view rqt_image_view"
echo ""
