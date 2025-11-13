# Camera Control Usage Guide

The ROS2 camera control system allows you to control the Android camera remotely via ROS2 topics.

## Starting the Node with Camera Control

Add the `--enable-control` flag when starting the node:

```bash
cd /home/nvidia/videoFromAndroid/orin/ros2_camcontrol
source install/setup.bash
python3 -m ros2_camcontrol.ws_to_image \
    --host 127.0.0.1 --port 9100 \
    --topic /recomo/rgb --rate 10 --codec h265 \
    --enable-control
```

## Available Command Topics

All commands use `std_msgs/String` messages.

### 1. Zoom Control
**Topic**: `/recomo/rgb/cmd/zoom`  
**Format**: Float value as string (e.g., "2.5")  
**Example**:
```bash
ros2 topic pub --once /recomo/rgb/cmd/zoom std_msgs/String "{data: '2.5'}"
```

### 2. Camera Switch
**Topic**: `/recomo/rgb/cmd/camera`  
**Format**: "front" or "back"  
**Example**:
```bash
ros2 topic pub --once /recomo/rgb/cmd/camera std_msgs/String "{data: 'front'}"
ros2 topic pub --once /recomo/rgb/cmd/camera std_msgs/String "{data: 'back'}"
```

### 3. Exposure/White Balance Locks
**Topic**: `/recomo/rgb/cmd/lock`  
**Format**: "ae:true", "ae:false", "awb:true", "awb:false"  
**Examples**:
```bash
# Lock auto-exposure
ros2 topic pub --once /recomo/rgb/cmd/lock std_msgs/String "{data: 'ae:true'}"

# Unlock auto-exposure
ros2 topic pub --once /recomo/rgb/cmd/lock std_msgs/String "{data: 'ae:false'}"

# Lock auto-white-balance
ros2 topic pub --once /recomo/rgb/cmd/lock std_msgs/String "{data: 'awb:true'}"
```

### 4. Recording Control
**Topic**: `/recomo/rgb/cmd/record`  
**Format**: "start" or "stop"  
**Examples**:
```bash
ros2 topic pub --once /recomo/rgb/cmd/record std_msgs/String "{data: 'start'}"
ros2 topic pub --once /recomo/rgb/cmd/record std_msgs/String "{data: 'stop'}"
```

### 5. Video Profile
**Topic**: `/recomo/rgb/cmd/profile`  
**Format**: "WIDTHxHEIGHT@FPS" (e.g., "1920x1080@30")  
**Example**:
```bash
ros2 topic pub --once /recomo/rgb/cmd/profile std_msgs/String "{data: '1920x1080@30'}"
```

### 6. Keyframe Request
**Topic**: `/recomo/rgb/cmd/keyframe`  
**Format**: Any string (content ignored)  
**Example**:
```bash
ros2 topic pub --once /recomo/rgb/cmd/keyframe std_msgs/String "{data: ''}"
```

## Telemetry Topic

**Topic**: `/recomo/rgb/telemetry`  
**Format**: JSON string with camera state  
**Example output**:
```json
{
  "af": "FOCUSED",
  "ae": "CONVERGED", 
  "iso": 100,
  "expNs": 33333333,
  "zoom": 2.5,
  "fps": 30.0,
  "camera": "back",
  "recording": false
}
```

**Monitor telemetry**:
```bash
ros2 topic echo /recomo/rgb/telemetry
```

## Using the Test Script

### Interactive Mode
```bash
cd /home/nvidia/videoFromAndroid/orin/ros2_camcontrol
source install/setup.bash
python3 -m ros2_camcontrol.camera_control_test
```

Available commands in interactive mode:
- `zoom 2.5` - Set zoom to 2.5x
- `camera front` - Switch to front camera
- `camera back` - Switch to back camera
- `ae on` - Lock auto-exposure
- `ae off` - Unlock auto-exposure
- `awb on` - Lock auto-white-balance
- `record start` - Start recording
- `record stop` - Stop recording
- `profile 1920x1080@30` - Change video profile
- `keyframe` - Request keyframe
- `demo` - Run automated test sequence
- `quit` - Exit

### Automated Demo Mode
```bash
python3 -m ros2_camcontrol.camera_control_test --demo
```

Runs a predefined sequence:
1. Zoom: 1.0x → 2.5x → 1.0x
2. Camera switch: front → back
3. AE lock: on → off
4. Keyframe request

## Programmatic Control Example

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MyCameraController(Node):
    def __init__(self):
        super().__init__('my_controller')
        self.zoom_pub = self.create_publisher(
            String, '/recomo/rgb/cmd/zoom', 10
        )
    
    def set_zoom(self, ratio):
        msg = String()
        msg.data = str(ratio)
        self.zoom_pub.publish(msg)
        self.get_logger().info(f'Zoom set to {ratio}')

def main():
    rclpy.init()
    controller = MyCameraController()
    controller.set_zoom(3.0)
    rclpy.shutdown()
```

## Performance Notes

- Camera control adds <50ms latency on average
- Image streaming continues at ~8.8 Hz during control operations
- Commands are queued and sent asynchronously (non-blocking)
- Telemetry updates at Android's native rate (~30Hz)

## Troubleshooting

**Commands not working?**
1. Verify `--enable-control` flag is used when starting the node
2. Check WebSocket connection is established (look for "Connected to ws://..." in logs)
3. Verify Android app is running and WebSocket server is active on port 9100

**Telemetry not appearing?**
1. Check that Android app is sending telemetry (WebUI should show values)
2. Verify topic name: `ros2 topic list | grep telemetry`
3. Check for JSON parsing errors in node logs
