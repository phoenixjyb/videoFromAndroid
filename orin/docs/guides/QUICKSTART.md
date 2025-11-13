# Quick Start - Camera Control Relay on Orin

## One-Time Setup

```bash
cd /path/to/camControl/orin
./setup_camera_relay.sh
```

This will:
- ✅ Create Python virtual environment (`.venv`)
- ✅ Install dependencies (including `websockets`)
- ✅ Test phone connectivity
- ✅ Show you next steps

## Daily Usage

### Start the Relay

```bash
cd /path/to/camControl/orin
./start_camera_relay.sh 172.16.30.28
```

Replace `172.16.30.28` with your phone's IP address.

### Test It Works

```bash
# In another terminal
ros2 topic pub --once /camera/zoom std_msgs/Float32 "data: 2.5"
```

You should see:
- Relay logs: `🔍 Received zoom command: 2.5` → `✅ Command sent successfully`
- Camera zooms to 2.5x on phone

### Available ROS2 Topics

```bash
/camera/zoom        # Float32 (1.0 - 10.0)
/camera/ae_lock     # Bool
/camera/awb_lock    # Bool
/camera/switch      # String ("back" or "front")
/camera/bitrate     # Int32 (in bps, e.g., 15000000 = 15Mbps)
/camera/codec       # String ("h264" or "h265")
/camera/key_frame   # Empty
```

### Test Commands

```bash
# Zoom to 3x
ros2 topic pub --once /camera/zoom std_msgs/Float32 "data: 3.0"

# Lock auto-exposure
ros2 topic pub --once /camera/ae_lock std_msgs/Bool "data: true"

# Switch to front camera
ros2 topic pub --once /camera/switch std_msgs/String "data: 'front'"

# Set 15Mbps bitrate
ros2 topic pub --once /camera/bitrate std_msgs/Int32 "data: 15000000"

# Switch to H.265
ros2 topic pub --once /camera/codec std_msgs/String "data: 'h265'"
```

## Troubleshooting

### "Connection refused"
- Check phone's camControl app is running
- Verify phone IP: `ping 172.16.30.28`
- Check port: `telnet 172.16.30.28 8080`

### "ModuleNotFoundError: websockets"
```bash
source .venv/bin/activate
pip install -r requirements.txt
```

### "No module named 'rclpy'"
```bash
source /opt/ros/humble/setup.bash  # or your ROS2 distro
```

### Check if relay is running
```bash
ps aux | grep camera_control_relay
```

### View relay logs
The relay prints colorful logs with emojis:
- 🔍 = Received ROS2 message
- 📤 = Sending to phone
- ✅ = Success
- ❌ = Error

## Integration with Your ROS2 Code

```python
from std_msgs.msg import Float32, Bool, String, Int32

# Create publishers
zoom_pub = node.create_publisher(Float32, '/camera/zoom', 10)
ae_lock_pub = node.create_publisher(Bool, '/camera/ae_lock', 10)

# Send commands
zoom_msg = Float32()
zoom_msg.data = 2.5
zoom_pub.publish(zoom_msg)
```

## File Locations

- Relay script: `camera_control_relay.py`
- Setup: `setup_camera_relay.sh`
- Start: `start_camera_relay.sh`
- Full guide: `SETUP_GUIDE.md`
- Architecture: `../docs/THREE_WAY_CAMERA_CONTROL.md`

## Need Help?

See detailed documentation:
- `SETUP_GUIDE.md` - Complete setup and troubleshooting
- `CAMERA_CONTROL_RELAY_README.md` - API reference
- `../docs/THREE_WAY_CAMERA_CONTROL.md` - System architecture
