# Orin Target API Specification

## Overview
This document specifies the API that needs to be implemented on the Orin device to receive target selection coordinates from the CamViewer Android app.

## Server Requirements

### Base Configuration
- **Host**: `0.0.0.0` (listen on all interfaces)
- **Port**: `8080`
- **Protocol**: HTTP/REST
- **Content-Type**: `application/json`

### Recommended Implementation
- **Framework**: FastAPI (Python)
- **CORS**: Enable for mobile device access
- **Logging**: Request/response logging for debugging

---

## API Endpoints

### POST /target

Receives normalized tap coordinates from the CamViewer app when user taps on the video stream.

#### Request

**Headers:**
```
Content-Type: application/json
```

**Body:**
```json
{
  "x": 0.5,
  "y": 0.5
}
```

**Parameters:**
- `x` (float, required): Normalized x-coordinate (0.0 = left, 1.0 = right)
- `y` (float, required): Normalized y-coordinate (0.0 = top, 1.0 = bottom)

**Coordinate System:**
- Origin (0, 0) is at **top-left** corner
- Range: `0.0 <= x <= 1.0` and `0.0 <= y <= 1.0`
- Coordinates are resolution-independent (normalized)

#### Response

**Success (200 OK):**
```json
{
  "status": "success",
  "x": 0.5,
  "y": 0.5,
  "message": "Target coordinates received"
}
```

**Error (400 Bad Request):**
```json
{
  "status": "error",
  "message": "Invalid coordinates: x and y must be between 0.0 and 1.0"
}
```

**Error (500 Internal Server Error):**
```json
{
  "status": "error",
  "message": "Failed to publish to ROS2 topic"
}
```

---

## ROS2 Integration

### Topic Publishing
When target coordinates are received, publish them to ROS2:

**Topic Name:** `/target_roi`

**Message Type:** `geometry_msgs/msg/Point` or custom message type

**Expected Behavior:**
1. Receive normalized coordinates from HTTP POST
2. Optionally transform to pixel coordinates based on camera resolution
3. Publish to ROS2 topic for vision pipeline consumption
4. Return success response to client

### Example Message
```python
# Using geometry_msgs/Point
from geometry_msgs.msg import Point

point_msg = Point()
point_msg.x = x  # normalized or pixel x
point_msg.y = y  # normalized or pixel y
point_msg.z = 0.0  # unused

publisher.publish(point_msg)
```

---

## Example Implementation (Python FastAPI)

### Minimal Server

```python
from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel, field_validator
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
import uvicorn

app = FastAPI(title="Orin Target API")

# Enable CORS for mobile device access
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_methods=["*"],
    allow_headers=["*"],
)

class TargetCoordinates(BaseModel):
    x: float
    y: float
    
    @field_validator('x', 'y')
    def validate_range(cls, v):
        if not 0.0 <= v <= 1.0:
            raise ValueError('Coordinate must be between 0.0 and 1.0')
        return v

class ROS2Publisher(Node):
    def __init__(self):
        super().__init__('target_publisher')
        self.publisher_ = self.create_publisher(Point, '/target_roi', 10)
    
    def publish_target(self, x: float, y: float):
        msg = Point()
        msg.x = x
        msg.y = y
        msg.z = 0.0
        self.publisher_.publish(msg)
        self.get_logger().info(f'Published target: ({x}, {y})')

# Initialize ROS2
rclpy.init()
ros2_node = ROS2Publisher()

@app.post("/target")
async def receive_target(coords: TargetCoordinates):
    """
    Receive target coordinates from CamViewer app
    """
    try:
        # Publish to ROS2
        ros2_node.publish_target(coords.x, coords.y)
        
        return {
            "status": "success",
            "x": coords.x,
            "y": coords.y,
            "message": "Target coordinates received"
        }
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

@app.get("/health")
async def health_check():
    """
    Health check endpoint
    """
    return {"status": "ok"}

if __name__ == "__main__":
    uvicorn.run(app, host="0.0.0.0", port=8080)
```

### Installation

```bash
# Install dependencies
pip install fastapi uvicorn pydantic

# For ROS2 integration
# (Assume ROS2 is already installed on Orin)
source /opt/ros/humble/setup.bash  # or your ROS2 distro
```

### Running the Server

```bash
# Simple run
python3 target_api.py

# Or with auto-reload for development
uvicorn target_api:app --host 0.0.0.0 --port 8080 --reload
```

---

## Testing

### Using curl

```bash
# Test health endpoint
curl http://172.16.30.234:8080/health

# Send target coordinates
curl -X POST http://172.16.30.234:8080/target \
  -H "Content-Type: application/json" \
  -d '{"x": 0.5, "y": 0.5}'

# Test invalid coordinates (should return 422)
curl -X POST http://172.16.30.234:8080/target \
  -H "Content-Type: application/json" \
  -d '{"x": 1.5, "y": 0.5}'
```

### Using Python

```python
import requests

url = "http://172.16.30.234:8080/target"
data = {"x": 0.5, "y": 0.5}

response = requests.post(url, json=data)
print(response.json())
```

### From CamViewer App

1. Open CamViewer app
2. Go to Settings tab
3. Verify Target API URL: `http://172.16.30.234:8080`
4. Save settings
5. Go to Video tab
6. Connect to camera
7. Tap anywhere on the video stream
8. Check Orin logs for received coordinates

---

## Network Configuration

### Firewall
Ensure port 8080 is open on the Orin device:

```bash
# Check if port is listening
sudo netstat -tlnp | grep 8080

# If using ufw firewall
sudo ufw allow 8080/tcp
```

### Testing Connectivity

From the Mac or Android device:

```bash
# Test if Orin is reachable
ping 172.16.30.234

# Test if port 8080 is open
nc -zv 172.16.30.234 8080
# or
telnet 172.16.30.234 8080
```

---

## Logging and Debugging

### Server Logs
- Log all incoming requests with timestamps
- Log coordinate values for verification
- Log ROS2 publish success/failure
- Use structured logging (JSON format recommended)

### Example Log Output
```
2025-11-11 10:30:45 INFO     POST /target - x=0.52, y=0.68
2025-11-11 10:30:45 INFO     Published to /target_roi: Point(x=0.52, y=0.68, z=0.0)
2025-11-11 10:30:45 INFO     Response: 200 OK
```

### Monitoring ROS2 Topic

```bash
# Monitor the target topic
ros2 topic echo /target_roi

# Check topic info
ros2 topic info /target_roi

# Check publishing rate
ros2 topic hz /target_roi
```

---

## Error Handling

### Client-Side (CamViewer)
The Android app will:
- Validate coordinates before sending (0.0-1.0 range)
- Log success/failure of HTTP requests
- Display errors in logcat (tag: `OrinTargetClient`)

### Server-Side (Orin)
The API should:
- Validate coordinate ranges (return 400 if invalid)
- Handle ROS2 publish failures gracefully (return 500)
- Log all errors with stack traces
- Continue running even if ROS2 node fails

---

## Future Enhancements

### Potential Additions
1. **Coordinate transformation**: Convert to pixel coordinates based on camera resolution
2. **Bounding box**: Return suggested ROI box around target point
3. **Target confirmation**: Acknowledge when vision system locks onto target
4. **Multiple targets**: Support sending array of coordinates
5. **WebSocket upgrade**: Real-time bidirectional communication for tracking updates

### Media API (Port 8081)
See separate documentation for the Media API implementation (Phase 5).

---

## Quick Start Checklist

- [ ] Install FastAPI and uvicorn on Orin
- [ ] Create target_api.py with the example code
- [ ] Ensure ROS2 is sourced and available
- [ ] Run the server: `python3 target_api.py`
- [ ] Test health endpoint: `curl http://172.16.30.234:8080/health`
- [ ] Configure CamViewer app with Orin IP
- [ ] Test end-to-end by tapping on video in CamViewer
- [ ] Monitor ROS2 topic: `ros2 topic echo /target_roi`

---

## Contact & Support

For issues or questions:
- Check CamViewer logs: `adb logcat -s OrinTargetClient VideoViewModel`
- Check Orin server logs
- Verify network connectivity between devices
- Ensure all devices are on the same WiFi network (172.16.30.0/24)
