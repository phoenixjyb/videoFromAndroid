# Orin Camera Control System

Control and process Android phone camera streams on Jetson Orin via WebSocket, ROS2, and REST APIs.

## Quick Start

**Start all services:**
```bash
./start_all_services.sh
```

**Stop all services:**
```bash
./stop_all_services.sh
```

See [Quick Start Guide](docs/guides/QUICKSTART.md) for detailed setup instructions.

---

## System Overview

The Orin system provides three main components:

1. **Target API** (port 8082) - Receives target detection data and publishes ROI to ROS2
2. **Media API** (port 8081) - Streams video from phone to desktop clients
3. **Camera Control Relay** - Relays camera commands from ROS2 to phone

### Architecture

```
Phone (Android)           Orin (Jetson)              Desktop/ROS2
  CamControl    ←─────→   Camera Relay   ←─────→   ROS2 Topics
     ↓                         ↓
  WebSocket              Target API
  :9090/video            :8082
     ↓                         ↓
  H.264 Stream          Detection Results
     ↓                         ↓
  Media API              ROS2 Publisher
  :8081                  /target_roi
```

See [Architecture Reference](docs/reference/ARCHITECTURE.md) for detailed design.

---

## 📚 Documentation

### Getting Started

- **[Quick Start](docs/guides/QUICKSTART.md)** - Get up and running in 5 minutes
- **[Setup Guide](docs/guides/SETUP.md)** - Detailed installation and configuration
- **[Camera Control Usage](docs/guides/CAMERA_CONTROL.md)** - How to control the camera

### API Reference

- **[Target API](docs/api/TARGET_API.md)** - Target detection endpoint (port 8082)
- **[Media API](docs/api/MEDIA_API.md)** - Video streaming endpoint (port 8081)
- **[Camera Relay](docs/api/CAMERA_RELAY.md)** - ROS2 to WebSocket camera control

### Guides

- **[ROI Updates](docs/guides/ROI_UPDATE.md)** - Working with region of interest data
- **[Listener Guide](docs/guides/LISTENER.md)** - Subscribing to ROS2 topics

### Reference

- **[Architecture](docs/reference/ARCHITECTURE.md)** - System design and components
- **[WebSocket Format](docs/reference/WS_FORMAT.md)** - Message formats and protocols
- **[Directory Structure](docs/reference/DIRECTORY.md)** - File organization
- **[Test Results](docs/reference/TEST_RESULTS.md)** - Camera control testing

---

## Key Scripts

| Script | Purpose |
|--------|---------|
| `start_all_services.sh` | Start Target API, Media API, and Camera Relay |
| `stop_all_services.sh` | Stop all services |
| `start_target_api.sh` | Start only Target API (port 8082) |
| `start_media_api.sh` | Start only Media API (port 8081) |
| `start_camera_relay.sh` | Start only Camera Control Relay |
| `test_target_api.py` | Test Target API with sample data |
| `test_ws_connection.py` | Test WebSocket connection to phone |
| `listen_target_roi.py` | Listen to /target_roi ROS2 topic |

---

## Network Configuration

The system supports two network presets:

**ZeroTier VPN** (default):
- Orin: `192.168.100.150`
- Phone: `192.168.100.156`

**T8Space WiFi**:
- Orin: `172.16.30.234`
- Phone: `172.16.30.28`

To switch networks:
```bash
export NETWORK_CONFIG=zerotier  # or t8space
./start_all_services.sh
```

See project root `config/` directory for network configuration details.

---

## ROS2 Topics

### Published by Orin

- `/target_roi` - Target detection bounding boxes
  - Type: `std_msgs/String`
  - Format: JSON with x, y, width, height

### Subscribed by Orin

- `/phone_camera/zoom` - Zoom level (Float32)
- `/phone_camera/switch` - Camera ID (Int32: 0=ultra, 1=wide, 2=tele)
- `/phone_camera/ae_lock` - Auto-exposure lock (Bool)
- `/phone_camera/awb_lock` - Auto white balance lock (Bool)
- `/phone_camera/bitrate` - Video bitrate in Mbps (Float32)

---

## Python Requirements

```bash
pip install -r requirements.txt
```

Key dependencies:
- `flask` - REST APIs
- `flask-cors` - Cross-origin support
- `websockets` - WebSocket client
- `aiohttp` - Async HTTP
- ROS2 Humble (system package)

---

## Troubleshooting

**Services not starting?**
```bash
# Check if ports are already in use
netstat -tuln | grep -E '8081|8082|9090'

# View service logs
tail -f *.log
```

**ROS2 topics not working?**
```bash
# Check ROS2 environment
source /opt/ros/humble/setup.bash
ros2 topic list

# Test topic subscription
ros2 topic echo /target_roi
```

**Phone connection issues?**
```bash
# Test WebSocket connection
python test_ws_connection.py --host 192.168.100.156

# Check phone app is running
adb shell "ps -A | grep camcontrol"
```

---

## Development

**Project Structure:**
```
orin/
├── docs/                    # Documentation
│   ├── api/                # API references
│   ├── guides/             # User guides
│   └── reference/          # Technical references
├── ros2_camcontrol/        # ROS2 package
├── target_api.py           # Target detection API
├── media_api.py            # Video streaming API
├── camera_control_relay.py # ROS2 to WebSocket relay
└── start_*.sh              # Service startup scripts
```

**Testing:**
```bash
# Test Target API
python test_target_api.py

# Test camera control
./test_camera_control.sh

# Listen to ROI updates
python listen_target_roi.py
```

---

## License

Part of the CamControl project - three-way camera control system (WebUI, CamViewer, ROS2).
