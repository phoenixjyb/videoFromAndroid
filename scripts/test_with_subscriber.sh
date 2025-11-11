#!/bin/bash
#
# Test script: Run publisher and subscriber together to measure actual throughput
#

set -e

# Configuration
MAX_IMAGES=${1:-100}  # Default: save 100 images
OUTPUT_DIR="${2:-/home/nvidia/videoFromAndroid/saved_images}"
TOPIC="/recomo/rgb"

echo "=========================================="
echo "Publisher + Subscriber Test"
echo "=========================================="
echo "Max images: $MAX_IMAGES"
echo "Output directory: $OUTPUT_DIR"
echo "Topic: $TOPIC"
echo ""

# Check ADB connection
echo "Checking ADB connection..."
if ! adb devices | grep -q "device$"; then
    echo "ERROR: No ADB device connected"
    exit 1
fi
echo "✓ ADB device connected"

# Check if CamControl app is running
echo "Checking if CamControl app is running..."
APP_RUNNING=$(adb shell "dumpsys activity activities | grep -i 'camcontrol' | grep -i 'topResumedActivity' || true")
if [ -z "$APP_RUNNING" ]; then
    echo "ERROR: CamControl app is not running"
    echo "Please start the CamControl app on your Android device"
    exit 1
fi
echo "✓ CamControl app is running"

# Setup port forwarding
echo "Setting up ADB port forwarding..."
adb forward tcp:9100 tcp:9090 > /dev/null
echo "✓ Port forwarding active: localhost:9100 -> device:9090"

# Source ROS2 environment
echo "Setting up ROS2 environment..."
set +u  # Disable unbound variable check for ROS sourcing
if [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
fi
if [ -f /home/nvidia/videoFromAndroid/orin/ros2_camcontrol/install/setup.bash ]; then
    source /home/nvidia/videoFromAndroid/orin/ros2_camcontrol/install/setup.bash
fi
set -u
echo "✓ ROS2 environment ready"

# Start publisher in background
echo ""
echo "Starting publisher node..."
python3 -m ros2_camcontrol.ws_to_image \
    --host 127.0.0.1 \
    --port 9100 \
    --topic $TOPIC \
    --rate 10 \
    --codec h265 \
    > /tmp/publisher.log 2>&1 &

PUBLISHER_PID=$!
echo "✓ Publisher started (PID: $PUBLISHER_PID)"

# Wait for publisher to initialize
echo "Waiting for publisher to initialize..."
sleep 3

# Check if publisher is still running
if ! kill -0 $PUBLISHER_PID 2>/dev/null; then
    echo "ERROR: Publisher exited unexpectedly"
    echo "Recent log:"
    tail -20 /tmp/publisher.log
    exit 1
fi

# Start subscriber (will run in foreground and stop after max_images)
echo ""
echo "Starting subscriber node (saving up to $MAX_IMAGES images)..."
echo "=========================================="
python3 -m ros2_camcontrol.image_saver \
    --topic $TOPIC \
    --output-dir "$OUTPUT_DIR" \
    --max-images $MAX_IMAGES \
    --format png

# Subscriber has finished, kill publisher
echo ""
echo "Stopping publisher..."
kill $PUBLISHER_PID 2>/dev/null || true
wait $PUBLISHER_PID 2>/dev/null || true
echo "✓ Publisher stopped"

# Show saved images info
echo ""
echo "=========================================="
echo "Checking saved images..."
LATEST_RUN=$(ls -td "$OUTPUT_DIR"/run_* 2>/dev/null | head -1)
if [ -n "$LATEST_RUN" ]; then
    IMAGE_COUNT=$(ls "$LATEST_RUN"/*.png 2>/dev/null | wc -l)
    echo "Images saved in: $LATEST_RUN"
    echo "Total images: $IMAGE_COUNT"
    if [ -f "$LATEST_RUN/stats.txt" ]; then
        echo ""
        echo "Statistics:"
        cat "$LATEST_RUN/stats.txt"
    fi
else
    echo "Warning: No output directory found"
fi

echo ""
echo "Test complete!"
