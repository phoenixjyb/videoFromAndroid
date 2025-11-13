# Target ROI Listener Guide

## Quick Start

### Start the Listener
```bash
cd /home/nvidia/videoFromAndroid/orin
source /opt/ros/humble/setup.bash
python3 listen_target_roi.py
```

### Stop the Listener
Press `Ctrl+C` or:
```bash
pkill -f listen_target_roi.py
```

## What You'll See

### Tap Points (from CamViewer taps)
```
[INFO] [timestamp] [target_roi_listener]: 📍 Tap #18: Center=(0.750, 0.250) [default 10% box]
```
- Shows the center point of the tap
- Server automatically creates 10% bounding box around it

### Full ROI (bounding boxes)
```
[INFO] [timestamp] [target_roi_listener]: 🔲 ROI #19: TopLeft=(0.300, 0.400) Size=(0.200×0.150) Center=(0.400, 0.475)
```
- Shows top-left corner, size, and calculated center
- Used when width/height are explicitly provided

## Coordinate System

All coordinates are normalized (0.0 - 1.0):
```
(0.0, 0.0) ──────────────► x (1.0, 0.0)
    │
    │    [ROI]
    │
    ▼
    y
(0.0, 1.0)              (1.0, 1.0)
```

## Testing

### Send Test Tap Point
```bash
curl -X POST http://localhost:8080/target \
  -H "Content-Type: application/json" \
  -d '{"x": 0.5, "y": 0.5}'
```

### Send Test ROI
```bash
curl -X POST http://localhost:8080/target \
  -H "Content-Type: application/json" \
  -d '{"x": 0.3, "y": 0.4, "width": 0.2, "height": 0.15}'
```

## Use with CamViewer

1. Start the listener
2. Open CamViewer app on phone
3. Tap anywhere on the video
4. Watch the listener show received coordinates in real-time

## ROS2 Commands

### Echo Topic (one message)
```bash
source /opt/ros/humble/setup.bash
ros2 topic echo /target_roi --once
```

### Monitor Topic Rate
```bash
ros2 topic hz /target_roi
```

### Topic Info
```bash
ros2 topic info /target_roi
```

### Topic Type
```bash
ros2 interface show sensor_msgs/msg/RegionOfInterest
```

## Integration with Your Code

Subscribe to `/target_roi` in your ROS2 node:

```python
from sensor_msgs.msg import RegionOfInterest

class YourNode(Node):
    def __init__(self):
        super().__init__('your_node')
        self.subscription = self.create_subscription(
            RegionOfInterest,
            '/target_roi',
            self.roi_callback,
            10
        )
    
    def roi_callback(self, msg: RegionOfInterest):
        # Convert to normalized coords
        x_norm = msg.x_offset / 10000.0
        y_norm = msg.y_offset / 10000.0
        width_norm = msg.width / 10000.0
        height_norm = msg.height / 10000.0
        
        # Process the ROI...
```

## Troubleshooting

### No messages received
1. Check Target API is running: `ps aux | grep target_api.py`
2. Check ROS2 topic exists: `ros2 topic list | grep target_roi`
3. Check CamViewer settings have correct Orin URL

### Old messages on startup
The listener shows historical messages briefly - this is normal ROS2 behavior.

### Listener stops responding
Restart both the Target API and listener:
```bash
pkill -f target_api.py
pkill -f listen_target_roi.py
cd /home/nvidia/videoFromAndroid/orin
source /opt/ros/humble/setup.bash
nohup python3 target_api.py --port 8080 > target_api.log 2>&1 &
python3 listen_target_roi.py
```
