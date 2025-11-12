# Three-Way Camera Control Pipeline

Complete implementation of unified camera control from three different sources.

## Architecture Overview

```
┌──────────────┐        ┌──────────────────┐        ┌────────────────────┐
│   WebUI      │        │  CamViewer       │        │  ROS2 Topics       │
│  (Browser)   │        │  Developer Mode  │        │  (Orin)            │
│              │        │  (Tablet)        │        │                    │
└──────┬───────┘        └────────┬─────────┘        └──────────┬─────────┘
       │                         │                              │
       │ HTTP/WS                 │ WebSocket                    │ Subscribe
       │ (existing)              │ (implemented)                │ /camera/*
       │                         │                              │
       ▼                         ▼                              ▼
┌───────────────────────────────────────────────────────────────────────┐
│                    Phone camControl App (SM-S9280)                    │
│  ┌─────────────────────────────────────────────────────────────────┐  │
│  │                     ControlServer (port 8080)                   │  │
│  │  - Accepts WebSocket connections                                │  │
│  │  - Parses ControlCommand JSON                                   │  │
│  │  - Broadcasts commands to Camera2Controller                     │  │
│  └─────────────────────────────────────────────────────────────────┘  │
│                                    ▼                                   │
│  ┌─────────────────────────────────────────────────────────────────┐  │
│  │                    Camera2Controller                             │  │
│  │  - setZoomRatio(), setAeLock(), setAwbLock()                    │  │
│  │  - switchCamera(), setBitrate(), setCodec()                     │  │
│  │  - Applies settings via Camera2 API                             │  │
│  └─────────────────────────────────────────────────────────────────┘  │
│                                    ▼                                   │
│  ┌─────────────────────────────────────────────────────────────────┐  │
│  │                      Camera Hardware                             │  │
│  │  - Physical camera sensor                                        │  │
│  │  - Zoom, exposure, white balance control                        │  │
│  └─────────────────────────────────────────────────────────────────┘  │
└───────────────────────────────────────────────────────────────────────┘
                                    │
                                    ▼ Encoded video stream
┌──────────────────────────────────────────────────────────────────────┐
│                           Orin (172.16.30.234)                       │
│  - Receives H.264/H.265 stream via WebSocket                        │
│  - Publishes to ROS2 /recomo/rgb topic                              │
│  - ROI target control via target_api.py                             │
│  - Camera control relay via camera_control_relay.py                 │
└──────────────────────────────────────────────────────────────────────┘
```

## 1. WebUI Control (Existing)

**Source**: Browser accessing `http://172.16.30.28:8080`

**Implementation**: 
- Web interface in camControl app
- Direct WebSocket connection to phone
- Status: ✅ Already working

**Available Controls**:
- Zoom (1.0x - 10.0x)
- Camera switch (back/front)
- Video profile (resolution/FPS)
- Bitrate (2M - 20M)
- Codec (H.264/H.265)
- Recording start/stop
- AE/AWB lock

## 2. CamViewer Developer Mode (New)

**Source**: CamViewer app running on tablet (SM-S9280)

**Implementation**:
- Semi-transparent overlay at bottom of video screen
- Collapsible panel (tap arrow to expand/collapse)
- Connects to `phoneControlHost` setting (default: 172.16.30.28:8080)
- Sends WebSocket commands via `PhoneCameraClient`

**Files Modified**:
- `camviewer/src/main/java/com/example/camviewer/network/PhoneCameraClient.kt` (renamed from OrinCameraClient)
- `camviewer/src/main/java/com/example/camviewer/ui/screens/video/VideoViewModel.kt`
- `camviewer/src/main/java/com/example/camviewer/ui/screens/video/CameraControlPanel.kt`
- `camviewer/src/main/java/com/example/camviewer/data/model/Models.kt` (added phoneControlHost)
- `camviewer/src/main/java/com/example/camviewer/data/repository/SettingsRepository.kt`
- `camviewer/src/main/java/com/example/camviewer/di/NetworkModule.kt`

**Usage**:
1. Open CamViewer app on tablet
2. Go to Settings → Enable "Developer Mode"
3. Set "Phone Control Host" to phone's IP (e.g., 172.16.30.28)
4. Go to Video screen
5. Tap up arrow on bottom overlay to expand controls
6. Adjust zoom, locks, camera, bitrate, codec

**Available Controls**:
- Zoom slider (1.0x - 10.0x, 18 steps)
- AE Lock toggle
- AWB Lock toggle
- Camera selection (Back/Front chips)
- Bitrate chips (2M, 5M, 10M, 15M, 20M)
- Codec chips (H.264/H.265)

**Status**: ✅ Implemented and deployed

## 3. ROS2 Topic Control (New)

**Source**: Downstream ROS2 nodes on Orin

**Implementation**:
- Python script: `orin/camera_control_relay.py`
- Subscribes to camera control topics
- Forwards commands to phone via WebSocket

**Files Added**:
- `orin/camera_control_relay.py` (270 lines)
- `orin/CAMERA_CONTROL_RELAY_README.md` (documentation)
- `orin/requirements.txt` (updated with websockets)

**ROS2 Topics**:
| Topic | Type | Description |
|-------|------|-------------|
| `/camera/zoom` | `std_msgs/Float32` | Zoom ratio 1.0-10.0 |
| `/camera/ae_lock` | `std_msgs/Bool` | Auto Exposure lock |
| `/camera/awb_lock` | `std_msgs/Bool` | Auto White Balance lock |
| `/camera/switch` | `std_msgs/String` | "back" or "front" |
| `/camera/bitrate` | `std_msgs/Int32` | Bitrate in bps |
| `/camera/codec` | `std_msgs/String` | "h264" or "h265" |
| `/camera/key_frame` | `std_msgs/Empty` | Request key frame |

**Usage**:

Start the relay:
```bash
cd /path/to/camControl/orin
python3 camera_control_relay.py --phone-host 172.16.30.28
```

Example ROS2 commands:
```bash
# Set zoom to 3.0x
ros2 topic pub --once /camera/zoom std_msgs/Float32 "data: 3.0"

# Enable AE lock
ros2 topic pub --once /camera/ae_lock std_msgs/Bool "data: true"

# Switch to front camera
ros2 topic pub --once /camera/switch std_msgs/String "data: 'front'"

# Set bitrate to 15Mbps
ros2 topic pub --once /camera/bitrate std_msgs/Int32 "data: 15000000"
```

**Status**: ✅ Implemented (needs testing on Orin)

## Command Format

All three sources send JSON commands in this format to the phone's WebSocket server:

### SetZoomRatio
```json
{
  "type": "setZoomRatio",
  "value": 2.5
}
```

### SetAeLock
```json
{
  "type": "setAeLock",
  "value": true
}
```

### SetAwbLock
```json
{
  "type": "setAwbLock",
  "value": false
}
```

### SwitchCamera
```json
{
  "type": "switchCamera",
  "facing": "back"
}
```

### SetBitrate
```json
{
  "type": "setBitrate",
  "bitrate": 10000000
}
```

### SetCodec
```json
{
  "type": "setCodec",
  "codec": "h265"
}
```

### RequestKeyFrame
```json
{
  "type": "requestKeyFrame"
}
```

## Network Configuration

### Phone (SM-S9280)
- IP: `172.16.30.28` (example)
- WebSocket Server: port `8080`
- Runs: camControl app with ControlServer

### Orin
- IP: `172.16.30.234`
- Runs: 
  - `target_api.py` (ROI control)
  - `media_api.py` (file management)
  - `camera_control_relay.py` (camera control forwarding)

### Tablet (CamViewer)
- IP: varies
- Connects to:
  - Phone WebSocket (video stream): `ws://172.16.30.28:9090`
  - Phone WebSocket (camera control): `ws://172.16.30.28:8080`
  - Orin HTTP (ROI targets): `http://172.16.30.234:8080`
  - Orin HTTP (media): `http://172.16.30.234:8081`

## Testing Checklist

- [ ] **WebUI Control**
  - [ ] Open browser to `http://172.16.30.28:8080`
  - [ ] Test zoom slider
  - [ ] Test camera switch
  - [ ] Test bitrate change
  - [ ] Verify video stream updates

- [ ] **CamViewer Developer Mode**
  - [ ] Enable developer mode in settings
  - [ ] Set phone control host IP
  - [ ] Expand control overlay
  - [ ] Test each control (zoom, AE, AWB, camera, bitrate, codec)
  - [ ] Check phone logs for received commands
  - [ ] Verify camera responds

- [ ] **ROS2 Topic Control**
  - [ ] Install websockets: `pip install websockets`
  - [ ] Start relay: `python3 camera_control_relay.py --phone-host 172.16.30.28`
  - [ ] Publish to each topic
  - [ ] Check relay logs for forwarded commands
  - [ ] Check phone logs for received commands
  - [ ] Verify camera responds

- [ ] **Integration Test**
  - [ ] Control camera from all three sources simultaneously
  - [ ] Verify no conflicts or race conditions
  - [ ] Check that last command wins
  - [ ] Monitor phone CPU/network usage

## Troubleshooting

### CamViewer can't send commands
- Check phone IP in settings (Settings → Phone Control Host)
- Ensure phone's camControl app is running
- Verify phone and tablet on same network
- Test connectivity: `telnet 172.16.30.28 8080`
- Check CamViewer logs: `adb logcat | grep PhoneCameraClient`

### ROS2 relay can't connect
- Verify phone IP: `ping 172.16.30.28`
- Check phone WebSocket port: `telnet 172.16.30.28 8080`
- Ensure camControl app is running on phone
- Check relay logs for connection errors
- Verify ROS2 topics exist: `ros2 topic list | grep camera`

### Commands not applied
- Check phone logs: `adb logcat | grep ControlServer`
- Verify command format matches ControlCommand.kt
- Ensure camera is initialized and streaming
- Check for Camera2 API errors in phone logs

## Future Enhancements

- [ ] Add video profile control to CamViewer developer mode
- [ ] Add recording start/stop to CamViewer
- [ ] Implement command feedback/acknowledgment
- [ ] Add command rate limiting to prevent conflicts
- [ ] Create ROS2 service interface (request/response vs publish)
- [ ] Add camera state synchronization across all UIs
- [ ] Implement command queue with priority

## References

- Phone WebSocket server: `app/src/main/java/com/example/camcontrol/CamControlService.kt`
- Command definitions: `app/src/main/java/com/example/camcontrol/transport/ControlCommand.kt`
- CamViewer client: `camviewer/src/main/java/com/example/camviewer/network/PhoneCameraClient.kt`
- ROS2 relay: `orin/camera_control_relay.py`
- Documentation: `orin/CAMERA_CONTROL_RELAY_README.md`
