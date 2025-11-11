# Orin Target API

REST API server for receiving target selection coordinates from the CamViewer Android app.

## Overview

The Target API allows the CamViewer app to send tap coordinates to the Orin device, which then:
1. Receives normalized coordinates (0.0-1.0) via HTTP POST
2. Validates the coordinates
3. Publishes to ROS2 topic `/target_roi` (if ROS2 enabled)
4. Returns success/failure response

## Quick Start

### 1. Install Dependencies

```bash
cd /home/nvidia/videoFromAndroid/orin

# Create virtual environment
python3 -m venv .venv-target-api
source .venv-target-api/bin/activate

# Install requirements
pip install -r requirements-target-api.txt
```

### 2. Start the Server

**Easy way (auto-detects ROS2):**
```bash
./start_target_api.sh
```

**Manual start with ROS2:**
```bash
source /opt/ros/humble/setup.bash
python3 target_api.py --ros2
```

**Manual start without ROS2 (testing):**
```bash
python3 target_api.py --no-ros2
```

### 3. Test the API

In a separate terminal:

```bash
# Test from localhost
python3 test_target_api.py

# Test from specific URL
python3 test_target_api.py http://172.16.30.234:8080
```

Or use curl:

```bash
# Health check
curl http://localhost:8080/health

# Send target coordinates
curl -X POST http://localhost:8080/target \
  -H "Content-Type: application/json" \
  -d '{"x": 0.5, "y": 0.5}'
```

## API Endpoints

### GET `/health`
Health check endpoint.

**Response:**
```json
{
  "status": "ok",
  "ros2_enabled": true,
  "ros2_available": true
}
```

### POST `/target`
Receive target coordinates from CamViewer.

**Request:**
```json
{
  "x": 0.5,
  "y": 0.5
}
```

**Parameters:**
- `x` (float): Normalized x-coordinate (0.0 = left, 1.0 = right)
- `y` (float): Normalized y-coordinate (0.0 = top, 1.0 = bottom)

**Response (Success):**
```json
{
  "status": "success",
  "x": 0.5,
  "y": 0.5,
  "message": "Target coordinates received"
}
```

**Response (Error):**
```json
{
  "detail": "Coordinate must be between 0.0 and 1.0"
}
```

## ROS2 Integration

When ROS2 is enabled, the API publishes target coordinates to:

**Topic:** `/target_roi`  
**Message Type:** `geometry_msgs/msg/Point`

```python
Point(
  x=0.5,  # normalized x
  y=0.5,  # normalized y
  z=0.0   # unused
)
```

### Monitor ROS2 Topic

```bash
# Echo messages
ros2 topic echo /target_roi

# Check publishing rate
ros2 topic hz /target_roi

# View topic info
ros2 topic info /target_roi
```

## CamViewer Configuration

In the CamViewer Android app:

1. Go to **Settings** tab
2. Set **Target API URL**: `http://172.16.30.234:8080`
   - Replace `172.16.30.234` with your Orin's IP address
3. Save settings
4. Go to **Video** tab
5. Connect to camera
6. Tap anywhere on video to send coordinates

## Network Setup

### Find Orin IP Address

```bash
# On Orin device
ip addr show | grep inet
# or
hostname -I
```

### Firewall Configuration

```bash
# Allow port 8080
sudo ufw allow 8080/tcp

# Check firewall status
sudo ufw status
```

### Test Connectivity

From your Android device or Mac:

```bash
# Test if Orin is reachable
ping 172.16.30.234

# Test if port is open
nc -zv 172.16.30.234 8080
```

## Command Line Options

```bash
python3 target_api.py --help
```

**Options:**
- `--port PORT` - Port to listen on (default: 8080)
- `--host HOST` - Host to bind to (default: 0.0.0.0)
- `--ros2` - Enable ROS2 publishing
- `--no-ros2` - Disable ROS2 publishing (test mode)

## Logs

The server logs all received coordinates:

```
2025-11-11 10:30:45 - target_api - INFO - Received target coordinates: x=0.5200, y=0.6800
2025-11-11 10:30:45 - target_api - INFO - Published target: (0.5200, 0.6800)
```

### View Logs

```bash
# Server logs are printed to console
# To save to file:
python3 target_api.py --ros2 2>&1 | tee target_api.log
```

## Troubleshooting

### Server won't start

**Error: "Address already in use"**
```bash
# Find process using port 8080
sudo netstat -tlnp | grep 8080

# Kill the process
sudo kill <PID>
```

### CamViewer can't connect

1. **Check server is running:**
   ```bash
   curl http://localhost:8080/health
   ```

2. **Check firewall:**
   ```bash
   sudo ufw status
   sudo ufw allow 8080/tcp
   ```

3. **Verify IP address:**
   ```bash
   hostname -I
   ```

4. **Test from another device:**
   ```bash
   ping <orin-ip>
   nc -zv <orin-ip> 8080
   ```

### ROS2 not working

1. **Check ROS2 is sourced:**
   ```bash
   echo $ROS_DISTRO
   # Should print: humble (or your distro)
   ```

2. **Verify ROS2 topic:**
   ```bash
   ros2 topic list | grep target_roi
   ```

3. **Check ROS2 node:**
   ```bash
   ros2 node list | grep target_publisher
   ```

## Performance

- **Latency:** ~10-20ms per request
- **Throughput:** 100+ requests/second
- **Memory:** ~50MB RAM usage

## Integration Example

### Python ROS2 Subscriber

```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point

class TargetSubscriber(Node):
    def __init__(self):
        super().__init__('target_subscriber')
        self.subscription = self.create_subscription(
            Point,
            '/target_roi',
            self.target_callback,
            10
        )
    
    def target_callback(self, msg):
        print(f'Received target: x={msg.x:.4f}, y={msg.y:.4f}')
        # Process target coordinates here
        # Convert to pixel coordinates if needed:
        # pixel_x = msg.x * image_width
        # pixel_y = msg.y * image_height

def main():
    rclpy.init()
    subscriber = TargetSubscriber()
    rclpy.spin(subscriber)

if __name__ == '__main__':
    main()
```

## See Also

- [ORIN_TARGET_API.md](../docs/ORIN_TARGET_API.md) - Full API specification
- [CamViewer README](../camviewer/README.md) - Android app documentation
- ROS2 documentation: https://docs.ros.org/en/humble/
