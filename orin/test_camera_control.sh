#!/bin/bash
# Test camera control relay functionality
# Make sure camera_control_relay is running before using this script

set -e

echo "🧪 Testing Camera Control via ROS2"
echo "=================================="
echo ""

# Check if relay is running
if ! pgrep -f camera_control_relay > /dev/null; then
    echo "⚠️  Warning: camera_control_relay doesn't appear to be running"
    echo "Start it with: ./start_camera_relay.sh"
    echo ""
    read -p "Continue anyway? (y/n) " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        exit 1
    fi
fi

echo "Test 1: Zoom control"
echo "--------------------"
echo "Setting zoom to 2.0x..."
ros2 topic pub --once /camera/zoom std_msgs/Float32 "data: 2.0"
sleep 2

echo ""
echo "Setting zoom to 5.0x..."
ros2 topic pub --once /camera/zoom std_msgs/Float32 "data: 5.0"
sleep 2

echo ""
echo "Setting zoom to 1.0x (default)..."
ros2 topic pub --once /camera/zoom std_msgs/Float32 "data: 1.0"
sleep 2

echo ""
echo "Test 2: Auto Exposure Lock"
echo "---------------------------"
echo "Locking AE..."
ros2 topic pub --once /camera/ae_lock std_msgs/Bool "data: true"
sleep 2

echo ""
echo "Unlocking AE..."
ros2 topic pub --once /camera/ae_lock std_msgs/Bool "data: false"
sleep 2

echo ""
echo "Test 3: Auto White Balance Lock"
echo "--------------------------------"
echo "Locking AWB..."
ros2 topic pub --once /camera/awb_lock std_msgs/Bool "data: true"
sleep 2

echo ""
echo "Unlocking AWB..."
ros2 topic pub --once /camera/awb_lock std_msgs/Bool "data: false"
sleep 2

echo ""
echo "Test 4: Key Frame Request"
echo "-------------------------"
echo "Requesting key frame..."
ros2 topic pub --once /camera/key_frame std_msgs/Empty
sleep 2

echo ""
echo "Test 5: Bitrate Control"
echo "-----------------------"
echo "Setting bitrate to 5 Mbps..."
ros2 topic pub --once /camera/bitrate std_msgs/Int32 "data: 5000000"
sleep 2

echo ""
echo "Setting bitrate to 10 Mbps..."
ros2 topic pub --once /camera/bitrate std_msgs/Int32 "data: 10000000"
sleep 2

echo ""
echo "Test 6: Codec Selection"
echo "-----------------------"
echo "Switching to H.265..."
ros2 topic pub --once /camera/codec std_msgs/String "data: 'h265'"
sleep 2

echo ""
echo "Switching to H.264..."
ros2 topic pub --once /camera/codec std_msgs/String "data: 'h264'"
sleep 2

echo ""
echo "✅ All tests complete!"
echo ""
echo "Note: Check the phone's camControl app to verify that"
echo "the camera responded to these commands."
