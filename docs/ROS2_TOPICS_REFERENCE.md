# ROS2 Topics Reference

Complete reference for all ROS2 topics used in the CamControl system.

## Overview

The system uses ROS2 topics for three main purposes:
1. **Camera Control** - Remote control of phone camera parameters
2. **Video Streaming** - Publishing phone video to ROS2 ecosystem
3. **Target Tracking** - ROI selection and bounding box information

---

## Camera Control Topics

### Publisher: `camera_control_relay.py`
### Subscribers: Phone camera (via WebSocket relay)

These topics control the phone camera remotely from ROS2 nodes.

### `/camera/zoom`
- **Type:** `std_msgs/Float32`
- **Purpose:** Control camera zoom level
- **Range:** 1.0 (no zoom) to 10.0 (maximum zoom)
- **Example:**
  ```bash
  ros2 topic pub --once /camera/zoom std_msgs/Float32 "data: 2.5"
  ```

### `/camera/ae_lock`
- **Type:** `std_msgs/Bool`
- **Purpose:** Lock/unlock auto-exposure
- **Values:** `true` (locked), `false` (unlocked)
- **Example:**
  ```bash
  ros2 topic pub --once /camera/ae_lock std_msgs/Bool "data: true"
  ```

### `/camera/awb_lock`
- **Type:** `std_msgs/Bool`
- **Purpose:** Lock/unlock auto white balance
- **Values:** `true` (locked), `false` (unlocked)
- **Example:**
  ```bash
  ros2 topic pub --once /camera/awb_lock std_msgs/Bool "data: false"
  ```

### `/camera/switch`
- **Type:** `std_msgs/String`
- **Purpose:** Switch between front/back cameras
- **Values:** `"back"`, `"front"`
- **Example:**
  ```bash
  ros2 topic pub --once /camera/switch std_msgs/String "data: 'front'"
  ```

### `/camera/bitrate`
- **Type:** `std_msgs/Int32`
- **Purpose:** Set video encoder bitrate
- **Range:** 2,000,000 to 50,000,000 bits/second (2-50 Mbps)
- **Default:** ~8,200,000 (8.2 Mbps for 1080p H.265)
- **Example:**
  ```bash
  ros2 topic pub --once /camera/bitrate std_msgs/Int32 "data: 15000000"
  ```

### `/camera/codec`
- **Type:** `std_msgs/String`
- **Purpose:** Switch video codec
- **Values:** `"h264"`, `"h265"`
- **Default:** `"h265"`
- **Example:**
  ```bash
  ros2 topic pub --once /camera/codec std_msgs/String "data: 'h264'"
  ```

### `/camera/key_frame`
- **Type:** `std_msgs/Empty`
- **Purpose:** Request immediate keyframe (I-frame) generation
- **Use Case:** Reduce latency for new video subscribers
- **Example:**
  ```bash
  ros2 topic pub --once /camera/key_frame std_msgs/Empty
  ```

---

## Video Streaming Topics

### Publisher: `ws_to_image` node (phone-to-ROS2 bridge)
### Subscribers: Vision processing nodes, recording nodes

These topics provide phone video as ROS2 Image messages.

### `/recomo/rgb`
- **Type:** `sensor_msgs/Image`
- **Purpose:** Main RGB video stream from phone
- **Format:** RGB8 encoding
- **Resolution:** Configurable (default 640×480 for efficiency)
- **Frame Rate:** ~10 Hz (configurable)
- **QoS:** BEST_EFFORT, VOLATILE, depth=1
- **Source:** Phone camera via WebSocket → H.265 decode → RGB conversion
- **Example:**
  ```bash
  # Monitor rate
  ros2 topic hz /recomo/rgb --window 50
  
  # View single frame
  ros2 topic echo /recomo/rgb --once
  
  # Record video
  ros2 bag record /recomo/rgb /recomo/camera_info
  ```

### `/recomo/camera_info`
- **Type:** `sensor_msgs/CameraInfo`
- **Purpose:** Camera calibration and metadata
- **Includes:** 
  - Image dimensions (width, height)
  - Distortion parameters
  - Camera matrix
  - Projection matrix
  - Frame ID for TF transforms
- **QoS:** BEST_EFFORT, VOLATILE, depth=1
- **Synchronized:** Published with each `/recomo/rgb` frame

### `/recomo/rgb/telemetry`
- **Type:** `std_msgs/String`
- **Purpose:** Phone camera telemetry and encoder stats
- **Format:** JSON string
- **Contents:**
  - Zoom level
  - Resolution (width, height)
  - Codec (h264/h265)
  - Bitrate (kbps)
  - Camera facing (back/front)
  - Frame timestamp
- **QoS:** BEST_EFFORT, depth=1

---

## Camera Control via Video Topic (Legacy)

### Subscribers: `ws_to_image` node
These are alternative command topics namespaced under the video topic.

**Note:** The `/camera/*` topics (via `camera_control_relay.py`) are now the **preferred** method for camera control. These topics are legacy from the older architecture.

### `/recomo/rgb/cmd/zoom`
- **Type:** `std_msgs/String`
- **Purpose:** Zoom control (legacy)
- **Prefer:** Use `/camera/zoom` instead

### `/recomo/rgb/cmd/camera`
- **Type:** `std_msgs/String`
- **Purpose:** Camera switch (legacy)
- **Prefer:** Use `/camera/switch` instead

### `/recomo/rgb/cmd/lock`
- **Type:** `std_msgs/String`
- **Purpose:** AE/AWB lock (legacy)
- **Prefer:** Use `/camera/ae_lock` and `/camera/awb_lock` instead

### `/recomo/rgb/cmd/profile`
- **Type:** `std_msgs/String`
- **Purpose:** Video profile change (legacy)
- **Status:** Deprecated

### `/recomo/rgb/cmd/record`
- **Type:** `std_msgs/String`
- **Purpose:** Recording control (legacy)
- **Status:** Deprecated (use on-device recording scripts)

### `/recomo/rgb/cmd/keyframe`
- **Type:** `std_msgs/String`
- **Purpose:** Keyframe request (legacy)
- **Prefer:** Use `/camera/key_frame` instead

---

## Target Tracking Topics

### Publisher: `target_api.py` (Target API ROS2 node)
### Subscribers: Vision processing nodes, tracking algorithms

These topics communicate target selection and ROI information.

### `/target/roi` (deprecated, now `/target_roi`)
### `/target_roi`
- **Type:** `sensor_msgs/RegionOfInterest`
- **Purpose:** Selected target bounding box
- **Fields:**
  - `x_offset` (uint32): Left edge in pixels
  - `y_offset` (uint32): Top edge in pixels
  - `width` (uint32): Box width in pixels
  - `height` (uint32): Box height in pixels
  - `do_rectify` (bool): Whether coordinates need rectification
- **Coordinate System:** Pixel coordinates in source video resolution
- **Source:** User selection in CamViewer tablet app
- **Flow:** CamViewer → HTTP POST `/target` → Target API → ROS2 publish
- **Use Case:** Track selected object, crop processing region
- **Example:**
  ```bash
  # Monitor ROI updates
  ros2 topic echo /target_roi
  
  # Listen with custom node
  ros2 run your_package listen_target_roi
  ```

### Camera Info Topics (for ROI coordinate transforms)

The Target API subscribes to these to transform coordinates:

#### `/recomo/camera_info`
- **Purpose:** Source video resolution (phone camera)
- **Used for:** Converting normalized ROI to pixel coordinates

#### `/recomo/rgb/camera_info`
- **Purpose:** Target video resolution (downstream processing)
- **Used for:** ROI coordinate transformation for downscaled images

---

## System Architecture

```
┌─────────────────────────────────────────────────────────┐
│                     Phone Camera                        │
│                  (CamControl App)                       │
└─────────┬────────────────────────────┬─────────────────┘
          │                            │
          │ WebSocket :9090            │ WebSocket :9090
          │ (video + telemetry)        │ (control commands)
          │                            │
          ▼                            ▼
┌─────────────────────┐    ┌─────────────────────────────┐
│  ws_to_image node   │    │ camera_control_relay.py     │
│  (Video Publisher)  │    │   (Command Relay)           │
└─────────┬───────────┘    └─────────┬───────────────────┘
          │                          │
          │ Publishes                │ Subscribes
          │                          │
          ▼                          ▼
  /recomo/rgb             /camera/zoom
  /recomo/camera_info     /camera/ae_lock
  /recomo/rgb/telemetry   /camera/awb_lock
                          /camera/switch
                          /camera/bitrate
                          /camera/codec
                          /camera/key_frame

┌──────────────────────────────────────────────────────────┐
│                  CamViewer (Tablet)                      │
│                                                          │
│  • Displays /recomo/rgb video                           │
│  • Sends camera commands via developer mode             │
│  • Sends target selection → Target API                  │
└────────────────────────┬─────────────────────────────────┘
                         │
                         │ HTTP POST /target
                         │
                         ▼
                ┌────────────────────┐
                │  target_api.py     │
                │  (Target Publisher)│
                └─────────┬──────────┘
                          │
                          │ Publishes
                          │
                          ▼
                    /target_roi
```

---

## Topic Summary Table

| Topic | Type | Publisher | Purpose | QoS |
|-------|------|-----------|---------|-----|
| `/camera/zoom` | Float32 | User/Relay | Zoom control | Default |
| `/camera/ae_lock` | Bool | User/Relay | AE lock | Default |
| `/camera/awb_lock` | Bool | User/Relay | AWB lock | Default |
| `/camera/switch` | String | User/Relay | Camera switch | Default |
| `/camera/bitrate` | Int32 | User/Relay | Bitrate control | Default |
| `/camera/codec` | String | User/Relay | Codec selection | Default |
| `/camera/key_frame` | Empty | User/Relay | Keyframe request | Default |
| `/recomo/rgb` | Image | ws_to_image | Video stream | BEST_EFFORT |
| `/recomo/camera_info` | CameraInfo | ws_to_image | Camera metadata | BEST_EFFORT |
| `/recomo/rgb/telemetry` | String | ws_to_image | Phone telemetry | BEST_EFFORT |
| `/target_roi` | RegionOfInterest | target_api | Target selection | Default |

---

## Usage Examples

### 1. Camera Control from Command Line
```bash
# Switch to front camera and zoom in
ros2 topic pub --once /camera/switch std_msgs/String "data: 'front'"
sleep 1
ros2 topic pub --once /camera/zoom std_msgs/Float32 "data: 3.0"

# Lock exposure and white balance
ros2 topic pub --once /camera/ae_lock std_msgs/Bool "data: true"
ros2 topic pub --once /camera/awb_lock std_msgs/Bool "data: true"

# Increase bitrate for high quality
ros2 topic pub --once /camera/bitrate std_msgs/Int32 "data: 20000000"
```

### 2. Monitor Video Stream
```bash
# Check frame rate
ros2 topic hz /recomo/rgb

# View telemetry
ros2 topic echo /recomo/rgb/telemetry

# Check resolution
ros2 topic echo /recomo/camera_info --once
```

### 3. Record Video
```bash
# Record video with camera info
ros2 bag record /recomo/rgb /recomo/camera_info -o my_recording

# Play back
ros2 bag play my_recording
```

### 4. Listen to Target Selection
```bash
# Monitor ROI updates
ros2 topic echo /target_roi

# Use dedicated listener tool
cd orin/
python3 listen_target_roi.py
```

### 5. Automated Testing
```bash
# Run comprehensive camera control test
cd orin/
./test_camera_control.sh
```

---

## Implementation Files

### Camera Control
- `orin/camera_control_relay.py` - ROS2 relay node
- `orin/start_camera_relay.sh` - Launch script
- `orin/test_camera_control.sh` - Test all controls

### Video Streaming
- `orin/ros2_camcontrol/ros2_camcontrol/ws_to_image.py` - Video publisher node
- `orin/start_phone_ros2_bridge.sh` - Launch phone→ROS2 bridge
- `orin/ros2_camcontrol/launch/` - Launch files

### Target Tracking
- `orin/target_api.py` - Target API server + ROS2 publisher
- `orin/listen_target_roi.py` - ROI monitoring tool
- `camviewer/.../OrinTargetClient.kt` - CamViewer target sender

### Service Management
- `orin/start_all_services.sh` - Start all services
- `orin/stop_all_services.sh` - Stop all services

---

## Best Practices

1. **Use `/camera/*` topics for camera control** - Preferred over legacy `/recomo/rgb/cmd/*`
2. **BEST_EFFORT QoS for video** - Reduces latency, drops old frames
3. **Request keyframes** - Before recording or when new subscribers join
4. **Monitor telemetry** - Check encoder health and camera state
5. **Use camera_info** - For coordinate transforms and calibration
6. **Test with scripts** - Use `test_camera_control.sh` before custom integration

---

## Troubleshooting

### Camera control not working
```bash
# Check if relay is running
ps aux | grep camera_control_relay

# Check topic subscriptions
ros2 topic info /camera/zoom

# Test manually
ros2 topic pub --once /camera/zoom std_msgs/Float32 "data: 2.0"
```

### No video on /recomo/rgb
```bash
# Check if phone bridge is running
ps aux | grep ws_to_image

# Check phone WebSocket connection
cd scripts/
python3 ws_probe.py

# Monitor node logs
ros2 run ros2_camcontrol ws_to_image --host <phone-ip>
```

### Target ROI not publishing
```bash
# Check if target API is running
ps aux | grep target_api

# Check HTTP endpoint
curl http://localhost:8082/health

# Monitor ROI topic
ros2 topic echo /target_roi
```

---

## Related Documentation

- [CAMERA_CONTROL_RELAY_README.md](../orin/CAMERA_CONTROL_RELAY_README.md) - Camera control details
- [TARGET_API_README.md](../orin/TARGET_API_README.md) - Target API documentation
- [SETUP_GUIDE.md](../orin/SETUP_GUIDE.md) - ROS2 environment setup
- [THREE_WAY_CAMERA_CONTROL.md](THREE_WAY_CAMERA_CONTROL.md) - Control architecture
- [README.md](../README.md) - Main project documentation
