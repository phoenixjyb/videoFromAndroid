# Orin Setup Guide for Camera Control Relay

## Prerequisites

Ensure you have:
- ROS2 installed and sourced
- Python 3.8+
- Network connectivity to phone (172.16.30.28)

## Installation Steps

### 1. Run the Setup Script (Recommended)

The easiest way to set everything up:

```bash
cd /path/to/camControl/orin
chmod +x setup_camera_relay.sh
./setup_camera_relay.sh
```

This script will:
- Check Python and ROS2 installation
- Create/activate virtual environment (.venv)
- Install all Python dependencies from requirements.txt
- Test phone connectivity
- Show you how to start the relay

### 2. Manual Installation (Alternative)

If you prefer to set up manually:

```bash
cd /path/to/camControl/orin

# Create and activate virtual environment
python3 -m venv .venv
source .venv/bin/activate

# Install dependencies
pip install -r requirements.txt
```

The requirements.txt includes:
- `websockets>=12.0` - for WebSocket client to phone
- `fastapi>=0.104.0` - for existing Target/Media APIs
- `uvicorn[standard]>=0.24.0` - ASGI server
- Other existing dependencies

### 2. Verify Phone Connectivity

Test that you can reach the phone's WebSocket server:

```bash
# Test network connectivity
ping 172.16.30.28

# Test WebSocket port (should connect, then Ctrl+C to exit)
telnet 172.16.30.28 8080
```

If `telnet` is not available:
```bash
nc -zv 172.16.30.28 8080
```

### 3. Start the Camera Control Relay

**Option A: Using the start script (recommended)**

```bash
cd /path/to/camControl/orin
chmod +x start_camera_relay.sh

# Start with default phone IP (172.16.30.28)
./start_camera_relay.sh

# Or specify a different phone IP
./start_camera_relay.sh 192.168.1.100
```

**Option B: Manual start**

```bash
cd /path/to/camControl/orin

# Activate virtual environment
source .venv/bin/activate

# Start relay
python3 camera_control_relay.py --phone-host 172.16.30.28

# With custom port (if needed)
python3 camera_control_relay.py --phone-host 172.16.30.28 --phone-port 8080
```

You should see:
```
[INFO] Camera Control Relay initialized
[INFO] Phone WebSocket: ws://172.16.30.28:8080/control
[INFO] Subscribed to /camera/zoom
[INFO] Subscribed to /camera/ae_lock
[INFO] Subscribed to /camera/awb_lock
[INFO] Subscribed to /camera/switch
[INFO] Subscribed to /camera/bitrate
[INFO] Subscribed to /camera/codec
[INFO] Subscribed to /camera/key_frame
```

### 4. Verify ROS2 Topics

In another terminal, check that topics are available:

```bash
# List all camera control topics
ros2 topic list | grep camera

# You should see:
# /camera/zoom
# /camera/ae_lock
# /camera/awb_lock
# /camera/switch
# /camera/bitrate
# /camera/codec
# /camera/key_frame
```

### 5. Test Camera Control

#### Test Zoom Control
```bash
ros2 topic pub --once /camera/zoom std_msgs/Float32 "data: 2.5"
```

**Expected relay output:**
```
🔍 Received zoom command: 2.5
📤 Sending command to phone: {'type': 'setZoomRatio', 'value': 2.5}
✅ Command sent successfully
```

**Expected phone output** (check phone logs):
```
📤 Command received: {"type":"setZoomRatio","value":2.5}
🔍 Zoom command received: 2.5, forwarding to MainActivity
```

#### Test Auto-Exposure Lock
```bash
ros2 topic pub --once /camera/ae_lock std_msgs/Bool "data: true"
```

**Expected relay output:**
```
☀️ Received AE lock command: True
📤 Sending command to phone: {'type': 'setAeLock', 'value': True}
✅ Command sent successfully
```

#### Test Camera Switch
```bash
# Switch to front camera
ros2 topic pub --once /camera/switch std_msgs/String "data: 'front'"

# Switch back to rear camera
ros2 topic pub --once /camera/switch std_msgs/String "data: 'back'"
```

**Expected relay output:**
```
📷 Received camera switch command: front
📤 Sending command to phone: {'type': 'switchCamera', 'facing': 'front'}
✅ Command sent successfully
```

#### Test Bitrate Change
```bash
# Set bitrate to 15 Mbps
ros2 topic pub --once /camera/bitrate std_msgs/Int32 "data: 15000000"
```

**Expected relay output:**
```
📊 Received bitrate command: 15000000
📤 Sending command to phone: {'type': 'setBitrate', 'bitrate': 15000000}
✅ Command sent successfully
```

#### Test Codec Change
```bash
# Switch to H.265
ros2 topic pub --once /camera/codec std_msgs/String "data: 'h265'"

# Switch to H.264
ros2 topic pub --once /camera/codec std_msgs/String "data: 'h264'"
```

#### Test Key Frame Request
```bash
ros2 topic pub --once /camera/key_frame std_msgs/Empty
```

### 6. Integration with Existing Services

The camera control relay runs independently from other Orin services. All services use the same virtual environment.

```bash
cd /path/to/camControl/orin
source .venv/bin/activate

# Terminal 1: Target API (ROI control)
./start_target_api.sh

# Terminal 2: Media API (file management)
./start_media_api.sh

# Terminal 3: Camera Control Relay (NEW)
./start_camera_relay.sh

# Or start all services (if using tmux or screen)
```

### 7. Create systemd Service (Optional)

For automatic startup, create `/etc/systemd/system/camera-control-relay.service`:

```ini
[Unit]
Description=Camera Control Relay for ROS2
After=network.target

[Service]
Type=simple
User=your-username
WorkingDirectory=/path/to/camControl/orin
ExecStart=/usr/bin/python3 camera_control_relay.py --phone-host 172.16.30.28
Restart=on-failure
RestartSec=5s
Environment="ROS_DOMAIN_ID=0"
# Add any other ROS2 environment variables needed

[Install]
WantedBy=multi-user.target
```

Enable and start:
```bash
sudo systemctl daemon-reload
sudo systemctl enable camera-control-relay.service
sudo systemctl start camera-control-relay.service
sudo systemctl status camera-control-relay.service
```

## Troubleshooting

### Issue: "ModuleNotFoundError: No module named 'websockets'"
**Solution:**
```bash
# Make sure virtual environment is activated
source .venv/bin/activate

# Install dependencies
pip install -r requirements.txt

# Or install websockets directly
pip install websockets
```

### Issue: "Connection refused" when connecting to phone
**Possible causes:**
1. Phone's camControl app is not running
2. Phone IP address has changed (check network settings)
3. Firewall blocking port 8080
4. Phone and Orin on different networks

**Debug steps:**
```bash
# Check phone is reachable
ping 172.16.30.28

# Check port is open
telnet 172.16.30.28 8080

# Try connecting with a test WebSocket client
python3 -c "
import asyncio
import websockets

async def test():
    uri = 'ws://172.16.30.28:8080/control'
    async with websockets.connect(uri) as ws:
        print('Connected successfully!')
        
asyncio.run(test())
"
```

### Issue: "No module named 'rclpy'"
**Solution:**
ROS2 environment not sourced. Add to your `~/.bashrc`:
```bash
source /opt/ros/humble/setup.bash  # or your ROS2 distro
```

Then reload:
```bash
source ~/.bashrc
```

### Issue: Commands sent but camera doesn't respond
**Check:**
1. Phone logs: `adb logcat | grep "ControlServer\|CamControlService"`
2. Verify command format matches what phone expects
3. Ensure phone camera is initialized and streaming
4. Check relay logs for errors

### Issue: Relay crashes or hangs
**Common causes:**
1. Network interruption - relay will log error and continue
2. Phone WebSocket server restarted - relay needs restart
3. Invalid ROS2 message data (e.g., zoom value outside 1.0-10.0 range)

**Solution:** 
The relay handles most errors gracefully. If it crashes, check logs and restart.

## Monitoring

### View relay logs in real-time
```bash
# If running with start script
./start_camera_relay.sh

# If running manually
source .venv/bin/activate
python3 camera_control_relay.py --phone-host 172.16.30.28

# If running as systemd service
sudo journalctl -u camera-control-relay.service -f
```

### Check phone logs (from development machine)
```bash
adb logcat | grep "ControlServer\|CamControlService\|Camera2Controller"
```

### Monitor ROS2 topics
```bash
# See all messages on zoom topic
ros2 topic echo /camera/zoom

# Check topic info
ros2 topic info /camera/zoom

# Check topic frequency
ros2 topic hz /camera/zoom
```

## Example Integration Script

Create a ROS2 node that automatically controls the camera:

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Bool, String, Int32

class AutoCameraController(Node):
    def __init__(self):
        super().__init__('auto_camera_controller')
        
        # Publishers for camera control
        self.zoom_pub = self.create_publisher(Float32, '/camera/zoom', 10)
        self.ae_lock_pub = self.create_publisher(Bool, '/camera/ae_lock', 10)
        self.bitrate_pub = self.create_publisher(Int32, '/camera/bitrate', 10)
        
        # Timer to adjust zoom based on some logic
        self.timer = self.create_timer(5.0, self.control_callback)
        self.zoom_level = 1.0
        
    def control_callback(self):
        # Example: cycle through zoom levels
        self.zoom_level = (self.zoom_level % 5.0) + 1.0
        
        zoom_msg = Float32()
        zoom_msg.data = self.zoom_level
        self.zoom_pub.publish(zoom_msg)
        
        self.get_logger().info(f'Set zoom to {self.zoom_level}x')

def main():
    rclpy.init()
    controller = AutoCameraController()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

Save as `auto_camera_controller.py` and run:
```bash
python3 auto_camera_controller.py
```

## Testing Checklist

- [ ] Python dependencies installed (`pip install -r requirements.txt`)
- [ ] Phone connectivity verified (`ping 172.16.30.28`)
- [ ] Phone WebSocket port accessible (`telnet 172.16.30.28 8080`)
- [ ] Camera control relay starts without errors
- [ ] ROS2 topics visible (`ros2 topic list | grep camera`)
- [ ] Zoom control works (`ros2 topic pub /camera/zoom ...`)
- [ ] AE lock works (`ros2 topic pub /camera/ae_lock ...`)
- [ ] AWB lock works (`ros2 topic pub /camera/awb_lock ...`)
- [ ] Camera switch works (`ros2 topic pub /camera/switch ...`)
- [ ] Bitrate change works (`ros2 topic pub /camera/bitrate ...`)
- [ ] Codec change works (`ros2 topic pub /camera/codec ...`)
- [ ] Key frame request works (`ros2 topic pub /camera/key_frame ...`)
- [ ] Phone logs show commands being received
- [ ] Camera actually responds to commands (visible in video stream)

## Next Steps

Once the relay is working:

1. **Integrate with your ROS2 application** - publish to `/camera/*` topics from your nodes
2. **Create launch files** - automate starting all services together
3. **Add monitoring** - create a node that monitors camera state
4. **Implement feedback** - consider adding a ROS2 service for command acknowledgment
5. **Document your use case** - add examples specific to your application

## Reference

- Full documentation: `CAMERA_CONTROL_RELAY_README.md`
- Architecture overview: `../docs/THREE_WAY_CAMERA_CONTROL.md`
- Deployment checklist: `../docs/DEPLOYMENT_CHECKLIST.md`
