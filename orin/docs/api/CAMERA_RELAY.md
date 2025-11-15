# Camera Control Relay

ROS2 node that enables camera control from downstream ROS2 nodes by forwarding commands to the phone's camControl app via WebSocket.

## Overview

This relay bridges ROS2 topics to the phone's WebSocket camera control API, enabling three ways to control the camera:

1. **WebUI** (browser) → Phone WebSocket → Camera
2. **CamViewer Developer Mode** (tablet) → Phone WebSocket → Camera  
3. **ROS2 Topics** (Orin) → **Camera Control Relay** → Phone WebSocket → Camera

## Installation

```bash
# Install Python dependencies
pip install -r requirements.txt

# Ensure ROS2 is sourced
source /opt/ros/humble/setup.bash  # or your ROS2 distribution
```

## Usage

Start the camera control relay with the phone's IP address:

```bash
python3 camera_control_relay.py --phone-host 172.16.30.28
```

The relay will subscribe to camera control topics and forward commands to the phone.

## ROS2 Topics

The relay subscribes to the following topics:

| Topic | Message Type | Description | Example |
|-------|--------------|-------------|---------|  
| `/recomo/film/zoom` | `std_msgs/Float32` | Zoom ratio (1.0 - 10.0) | `1.5` |
| `/recomo/film/ae_lock` | `std_msgs/Bool` | Auto Exposure lock | `true` |
| `/recomo/film/awb_lock` | `std_msgs/Bool` | Auto White Balance lock | `false` |
| `/recomo/film/switch` | `std_msgs/String` | Camera facing | `"back"` or `"front"` |
| `/recomo/film/bitrate` | `std_msgs/Int32` | Bitrate in bps | `10000000` (10Mbps) |
| `/recomo/film/codec` | `std_msgs/String` | Video codec | `"h264"` or `"h265"` |
| `/recomo/film/key_frame` | `std_msgs/Empty` | Request key frame | (empty) |## Example Commands

### Set zoom to 2.5x
```bash
ros2 topic pub --once /recomo/film/zoom std_msgs/Float32 "data: 2.5"
```

### Enable AE lock
```bash
ros2 topic pub --once /recomo/film/ae_lock std_msgs/Bool "data: true"
```

### Switch to front camera
```bash
ros2 topic pub --once /recomo/film/switch std_msgs/String "data: 'front'"
```

### Set bitrate to 15Mbps
```bash
ros2 topic pub --once /recomo/film/bitrate std_msgs/Int32 "data: 15000000"
```

### Change codec to H.265
```bash
ros2 topic pub --once /recomo/film/codec std_msgs/String "data: 'h265'"
```

### Request key frame
```bash
ros2 topic pub --once /recomo/film/key_frame std_msgs/Empty
```

## Integration Example

Python node that controls camera:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Bool

class CameraController(Node):
    def __init__(self):
        super().__init__('camera_controller')
        self.zoom_pub = self.create_publisher(Float32, '/recomo/film/zoom', 10)
        self.ae_lock_pub = self.create_publisher(Bool, '/recomo/film/ae_lock', 10)
        
    def zoom_in(self, ratio: float):
        msg = Float32()
        msg.data = ratio
        self.zoom_pub.publish(msg)
        
    def lock_exposure(self, locked: bool):
        msg = Bool()
        msg.data = locked
        self.ae_lock_pub.publish(msg)
```

## Architecture

```
┌─────────────────┐
│  Downstream     │
│  ROS2 Node      │
│  (tracking,     │
│   detection,    │
│   etc.)         │
└────────┬────────┘
         │ Publish to
         │ /camera/* topics
         ▼
┌─────────────────────────┐
│ Camera Control Relay    │
│ (this script)           │
│ - Subscribe to topics   │
│ - Forward via WebSocket │
└────────┬────────────────┘
         │ WebSocket
         │ ws://phone:8080
         ▼
┌─────────────────────────┐
│ camControl App (Phone)  │
│ - ControlServer         │
│ - Camera2Controller     │
└────────┬────────────────┘
         │ Camera2 API
         ▼
┌─────────────────┐
│  Camera HW      │
└─────────────────┘
```

## Troubleshooting

### Connection Refused
- Ensure phone is on the same network
- Verify phone's camControl app is running
- Check phone's IP address is correct
- Test with: `telnet <phone-ip> 8080`

### Commands Not Working
- Check phone app logs for received commands
- Verify ROS2 topics are publishing: `ros2 topic echo /recomo/film/zoom`
- Ensure command values are in valid range (e.g., zoom 1.0-10.0)

### No Response from Relay
- Check relay is running: `ros2 node list | grep camera_control_relay`
- Verify topic subscriptions: `ros2 topic info /recomo/film/zoom`
- Check relay logs for errors

## See Also

- [TARGET_API_README.md](TARGET_API_README.md) - ROI target control via ROS2
- [MEDIA_API_README.md](MEDIA_API_README.md) - Media file management
- Phone app: `app/src/main/java/com/example/camcontrol/transport/ControlCommand.kt`
