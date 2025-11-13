# WebSocket Architecture

Complete WebSocket communication architecture for the CamControl system.

## Overview

The system uses WebSocket connections for real-time, bidirectional communication between the phone camera and all clients (browsers, tablet, Orin). All WebSocket traffic flows through the phone's server on **port 9090**.

---

## High-Level Architecture

```
                    ┌─────────────────────────────────────────┐
                    │     Phone (CamControl App)              │
                    │     WebSocket Server :9090              │
                    │                                         │
                    │  ┌──────────────────────────────────┐   │
                    │  │  Camera2 API                     │   │
                    │  │  ↓                               │   │
                    │  │  MediaCodec H.265 Encoder        │   │
                    │  │  ↓                               │   │
                    │  │  Annex-B NAL Units               │   │
                    │  └──────────────┬───────────────────┘   │
                    │                 │                       │
                    │  ┌──────────────▼───────────────────┐   │
                    │  │  Ktor WebSocket Server           │   │
                    │  │                                  │   │
                    │  │  Endpoints:                      │   │
                    │  │  • ws://phone-ip:9090/           │   │
                    │  │    (video + telemetry)           │   │
                    │  │  • ws://phone-ip:9090/control    │   │
                    │  │    (commands)                    │   │
                    │  │                                  │   │
                    │  │  HTTP:                           │   │
                    │  │  • http://phone-ip:9090/         │   │
                    │  │    (WebUI HTML)                  │   │
                    │  └──────────────┬───────────────────┘   │
                    └─────────────────┼───────────────────────┘
                                     │
                                     │ All connections via WiFi
                                     │
        ┌────────────────────────────┼────────────────────────────┐
        │                            │                            │
        ▼                            ▼                            ▼
┌───────────────┐          ┌──────────────────┐       ┌─────────────────────┐
│   Browser     │          │  CamViewer       │       │  Orin (Jetson)      │
│   (WebUI)     │          │  (Tablet App)    │       │                     │
│               │          │                  │       │  ┌────────────────┐ │
│ WebSocket     │          │ WebSocket Client │       │  │ ws_to_image    │ │
│ Client        │          │                  │       │  │ (ROS2 bridge)  │ │
│               │          │ • Video Decode   │       │  │                │ │
│ • WebCodecs   │          │ • MediaCodec     │       │  │ • H.265 decode │ │
│   H.265       │          │   H.265          │       │  │ • RGB convert  │ │
│ • Video       │          │ • Developer Mode │       │  │ • Publish      │ │
│   Display     │          │   Controls       │       │  │   /recomo/rgb  │ │
│ • Camera      │          │ • Target         │       │  └────────────────┘ │
│   Controls    │          │   Selection      │       │                     │
│               │          │                  │       │  ┌────────────────┐ │
│               │          │                  │       │  │ camera_control │ │
│               │          │                  │       │  │ _relay.py      │ │
│               │          │                  │       │  │                │ │
│               │          │                  │       │  │ ROS2 topics →  │ │
│               │          │                  │       │  │ WS commands    │ │
│               │          │                  │       │  └────────────────┘ │
└───────────────┘          └──────────────────┘       └─────────────────────┘
```

---

## WebSocket Endpoints

### 1. Video Stream Endpoint: `ws://phone-ip:9090/`

**Purpose:** Real-time video streaming and telemetry broadcast

**Message Types:**
- **Binary frames** - H.265 encoded video (Annex-B NAL units)
- **Text frames** - JSON telemetry data

**Flow:** Phone → All connected clients (broadcast)

**Binary Message Format:**
```
Raw H.265 Annex-B NAL units
- Start codes: 0x00 0x00 0x00 0x01
- SPS/PPS/VPS parameter sets
- I-frames (keyframes)
- P-frames (predicted)
```

**Text Message Format (Telemetry):**
```json
{
  "timestamp": 1699876543210,
  "zoom": 2.5,
  "width": 1920,
  "height": 1080,
  "fps": 30,
  "codec": "h265",
  "bitrateKbps": 8200,
  "facing": "back"
}
```

**Clients:**
- WebUI browsers (WebCodecs decode)
- CamViewer tablet app (MediaCodec decode)
- Orin ws_to_image node (GStreamer decode → ROS2 publish)

**QoS:**
- Timeout: 75ms per client send
- Drops slow clients to prevent buffer buildup
- No message queuing (real-time only)

---

### 2. Control Endpoint: `ws://phone-ip:9090/control`

**Purpose:** Bidirectional camera control commands

**Message Types:**
- **Text frames** - JSON command messages

**Flow:** Any client → Phone (commands)

**Command Format:**
All commands use JSON with `"cmd"` discriminator field:

```json
{
  "cmd": "setZoomRatio",
  "value": 2.5
}
```

**Supported Commands:**

#### Zoom Control
```json
{
  "cmd": "setZoomRatio",
  "value": 3.0
}
```
- Range: 1.0 - 10.0

#### Camera Switch
```json
{
  "cmd": "switchCamera",
  "facing": "front"
}
```
- Values: `"back"`, `"front"`

#### Auto Exposure Lock
```json
{
  "cmd": "setAeLock",
  "value": true
}
```
- Values: `true`, `false`

#### Auto White Balance Lock
```json
{
  "cmd": "setAwbLock",
  "value": false
}
```
- Values: `true`, `false`

#### Bitrate Control
```json
{
  "cmd": "setBitrate",
  "bitrate": 15000000
}
```
- Range: 2,000,000 - 50,000,000 (2-50 Mbps)
- Clamped per resolution

#### Codec Switch
```json
{
  "cmd": "setCodec",
  "codec": "h265"
}
```
- Values: `"h264"`, `"h265"`
- Default: `"h265"`

#### Keyframe Request
```json
{
  "cmd": "requestKeyFrame"
}
```
- No parameters
- Forces immediate I-frame

**Clients:**
- WebUI (browser controls)
- CamViewer developer mode
- Orin camera_control_relay (ROS2 → WS bridge)
- Scripts (ws_cmd.py)

---

## Client Details

### 1. WebUI (Browser)

**Location:** `app/src/main/assets/index.html`

**Technology:**
- HTML5 + JavaScript
- WebCodecs API for H.265 decode (Safari, Chrome)
- Broadway.js fallback for H.264 (Firefox)

**Connection Flow:**
```javascript
// Video stream
const videoWs = new WebSocket('ws://phone-ip:9090/');
videoWs.binaryType = 'arraybuffer';

videoWs.onmessage = (event) => {
  if (event.data instanceof ArrayBuffer) {
    // Binary: H.265 NAL units
    decodeVideoFrame(event.data);
  } else {
    // Text: Telemetry JSON
    const telemetry = JSON.parse(event.data);
    updateUI(telemetry);
  }
};

// Control commands
const controlWs = new WebSocket('ws://phone-ip:9090/control');
controlWs.send(JSON.stringify({
  cmd: 'setZoomRatio',
  value: 2.5
}));
```

**Features:**
- Real-time video display
- Camera control UI (zoom slider, camera switch, bitrate)
- Telemetry display (resolution, FPS, codec, bitrate)
- Decoder type indicator (WebCodecs/Broadway)

---

### 2. CamViewer (Android Tablet App)

**Location:** `camviewer/src/main/java/com/example/camviewer/`

**Key Components:**
- `PhoneCameraClient.kt` - WebSocket client manager
- `VideoDecoder.kt` - MediaCodec H.265 decoder
- `VideoViewModel.kt` - Camera control state management

**Connection Flow:**
```kotlin
// Video stream
val videoClient = OkHttp WebSocketListener()
videoClient.connect("ws://$phoneIp:9090/")

override fun onMessage(webSocket: WebSocket, bytes: ByteString) {
    // Binary: H.265 NAL units → MediaCodec
    videoDecoder.decode(bytes.toByteArray())
}

override fun onMessage(webSocket: WebSocket, text: String) {
    // Text: Telemetry JSON
    val telemetry = Json.decodeFromString<Telemetry>(text)
    updateState(telemetry)
}

// Control commands
val controlClient = OkHttp WebSocketListener()
controlClient.connect("ws://$phoneIp:9090/control")
controlClient.send("""{"cmd":"setZoomRatio","value":2.5}""")
```

**Features:**
- Real-time video playback with MediaCodec
- Developer mode control overlay
- Target/ROI selection (tap/drag gestures)
- Bounding box visualization
- Media browser (connects to Orin media API)

---

### 3. Orin Phone-ROS2 Bridge

**Location:** `orin/ros2_camcontrol/ros2_camcontrol/ws_to_image.py`

**Script:** `orin/start_phone_ros2_bridge.sh`

**Connection Flow:**
```python
# WebSocket connection
ws = await websockets.connect('ws://phone-ip:9090/')

async for message in ws:
    if isinstance(message, bytes):
        # Binary: H.265 NAL units
        # → GStreamer decode → RGB conversion
        # → ROS2 sensor_msgs/Image publish
        process_video_frame(message)
    else:
        # Text: Telemetry JSON
        telemetry = json.loads(message)
        publish_telemetry(telemetry)
```

**Features:**
- H.265 hardware decode (NVDEC on Jetson)
- RGB conversion for ROS2
- Publishes to `/recomo/rgb` topic
- Publishes to `/recomo/camera_info` topic
- Publishes to `/recomo/rgb/telemetry` topic
- ~10 Hz publish rate (configurable)
- Resolution downscaling (1920×1080 → 640×480 for efficiency)

---

### 4. Orin Camera Control Relay

**Location:** `orin/camera_control_relay.py`

**Script:** `orin/start_camera_relay.sh`

**Connection Flow:**
```python
# WebSocket connection (control only)
ws = await websockets.connect('ws://phone-ip:9090/control')

# ROS2 subscriber callback
def zoom_callback(msg):
    command = {
        'cmd': 'setZoomRatio',
        'value': msg.data
    }
    await ws.send(json.dumps(command))

# Subscribe to ROS2 topics
node.create_subscription(Float32, '/camera/zoom', zoom_callback, 10)
# ... 6 more topics
```

**Features:**
- Bridges 7 ROS2 topics to WebSocket commands
- Async command queue to prevent blocking
- Reconnection on disconnect
- Status logging

**ROS2 Topics Subscribed:**
- `/camera/zoom` → `setZoomRatio`
- `/camera/ae_lock` → `setAeLock`
- `/camera/awb_lock` → `setAwbLock`
- `/camera/switch` → `switchCamera`
- `/camera/bitrate` → `setBitrate`
- `/camera/codec` → `setCodec`
- `/camera/key_frame` → `requestKeyFrame`

---

## Data Flow Diagrams

### Video Streaming Flow

```
Phone Camera (Camera2)
        ↓
MediaCodec Encoder (H.265)
        ↓
Annex-B NAL Units
        ↓
WebSocket Broadcast :9090/
        ↓
┌───────┴───────┬────────────┬──────────────┐
│               │            │              │
▼               ▼            ▼              ▼
Browser       Tablet      Orin ws_to_    Recording
WebCodecs     MediaCodec   image node     Script
Decode        Decode       GStreamer      (save .h265)
↓             ↓            ↓
Canvas        Surface      ROS2 Image
Display       View         /recomo/rgb
```

### Camera Control Flow

```
Control Source
(WebUI / CamViewer / ROS2 topic)
        ↓
JSON Command
{"cmd":"setZoomRatio","value":2.5}
        ↓
WebSocket :9090/control
        ↓
Phone CamControlService
        ↓
Camera2 CaptureRequest
        ↓
Physical Camera Hardware
```

### Target Selection Flow

```
User Tap/Drag in CamViewer
        ↓
Normalized Coordinates (0.0-1.0)
        ↓
HTTP POST to Orin Target API :8082
{"x":0.5,"y":0.5,"width":0.2,"height":0.2}
        ↓
Target API (target_api.py)
        ↓
Coordinate Transform
(normalized → pixel coordinates)
        ↓
ROS2 Publish /target_roi
(sensor_msgs/RegionOfInterest)
        ↓
Vision Processing Nodes
```

---

## WebSocket Protocol Details

### Connection Lifecycle

#### 1. Client Connection
```
Client → Server: HTTP Upgrade Request
GET ws://phone-ip:9090/ HTTP/1.1
Upgrade: websocket
Connection: Upgrade
Sec-WebSocket-Key: ...

Server → Client: HTTP 101 Switching Protocols
HTTP/1.1 101 Switching Protocols
Upgrade: websocket
Connection: Upgrade
```

#### 2. Active Communication
- **Video endpoint:** Server → Client (broadcast)
- **Control endpoint:** Client ↔ Server (bidirectional)

#### 3. Heartbeat/Keepalive
- Ping/Pong frames every 30 seconds
- Automatic in OkHttp and websockets library

#### 4. Disconnection
- Graceful close with status code
- Server removes client from broadcast list
- Client auto-reconnect logic

### Error Handling

#### Phone Server Side
```kotlin
// Timeout for slow clients
withTimeout(75) {
    session.send(Frame.Binary(...))
}
// Drop client on timeout to prevent blocking encoder
```

#### Client Side
```kotlin
override fun onFailure(webSocket: WebSocket, t: Throwable, response: Response?) {
    Log.e(TAG, "WebSocket error: ${t.message}")
    scheduleReconnect()
}
```

### Performance Characteristics

**Video Stream (Binary):**
- Frame size: 50 KB - 500 KB (depends on content, bitrate)
- Frequency: 30 FPS (H.265 encoded)
- Bandwidth: 8-20 Mbps typical
- Latency: 50-150ms end-to-end

**Telemetry (Text):**
- Message size: ~150 bytes JSON
- Frequency: 1-2 Hz
- Negligible bandwidth

**Control Commands (Text):**
- Message size: 30-80 bytes JSON
- Frequency: On-demand (user actions)
- Latency: <50ms typical

---

## Port Configuration

**All WebSocket traffic uses port 9090:**

| Component | Connection | Port | Protocol |
|-----------|------------|------|----------|
| Phone Server | Listen | 9090 | WS/HTTP |
| WebUI | Connect | 9090 | WS |
| CamViewer | Connect | 9090 | WS |
| Orin ws_to_image | Connect | 9090 | WS |
| Orin camera_relay | Connect | 9090 | WS |

**Other Ports (Non-WebSocket):**
| Service | Port | Protocol | Purpose |
|---------|------|----------|---------|
| Target API | 8082 | HTTP | ROI selection |
| Media API | 8081 | HTTP | Video browse/download |

---

## Network Requirements

### WiFi Configuration
- All devices must be on **same WiFi network**
- Phone IP must be reachable from all clients
- No firewall blocking port 9090

### Bandwidth Requirements
- **Phone upload:** 10-25 Mbps (multiple clients)
- **Per client download:** 8-20 Mbps
- **Recommend:** 5 GHz WiFi for stability

### Latency
- **WiFi latency:** <10ms typical
- **Encoding latency:** 20-50ms
- **Decoding latency:** 20-50ms
- **Network buffering:** 10-30ms
- **Total end-to-end:** 50-150ms

---

## Testing & Debugging

### Test WebSocket Connection
```bash
# Probe WebSocket
cd scripts/
python3 ws_probe.py

# Send command
python3 ws_cmd.py set-zoom 2.5

# Save video stream
python3 ws_save_h264.py -o test.h265 -d 10
```

### Monitor Traffic
```bash
# Browser DevTools
# Network tab → WS filter → Messages

# Android Logcat (CamViewer)
adb logcat | grep -E "PhoneCameraClient|VideoDecoder"

# Phone Logcat
adb logcat | grep CamControlService

# Orin (ws_to_image)
tail -f /tmp/ws_to_image.log
```

### Common Issues

**No video:**
- Check phone IP address
- Verify port 9090 open
- Restart CamControl app (resets encoder)

**Commands not working:**
- Verify JSON format with `"cmd"` field (not `"type"`)
- Check `/control` endpoint connection
- Monitor phone logs for command reception

**High latency:**
- Switch to 5 GHz WiFi
- Reduce bitrate: `ros2 topic pub --once /camera/bitrate std_msgs/Int32 "data: 5000000"`
- Check for network congestion

---

## Security Considerations

**Current Implementation:**
- ⚠️ No authentication
- ⚠️ No encryption (plain WS, not WSS)
- ⚠️ CORS allows all origins

**For Production:**
- Add authentication tokens
- Use WSS (WebSocket Secure)
- Restrict CORS origins
- Add rate limiting on commands
- Validate all JSON inputs

---

## Implementation Files

### Phone (Server)
- `app/src/main/java/com/example/camcontrol/CamControlService.kt`
  - WebSocket server setup
  - Video broadcast logic
  - Command handling
- `app/src/main/java/com/example/camcontrol/transport/ControlServer.kt`
  - Ktor WebSocket routes
  - Client connection management

### CamViewer (Client)
- `camviewer/src/main/java/com/example/camviewer/network/PhoneCameraClient.kt`
  - OkHttp WebSocket client
  - Connection lifecycle
  - Message parsing
- `camviewer/src/main/java/com/example/camviewer/video/VideoDecoder.kt`
  - MediaCodec H.265 decoder
  - Surface rendering

### Orin (Clients)
- `orin/ros2_camcontrol/ros2_camcontrol/ws_to_image.py`
  - Python websockets client
  - GStreamer pipeline
  - ROS2 publishing
- `orin/camera_control_relay.py`
  - Python websockets client
  - ROS2 subscription
  - Command forwarding

### WebUI (Client)
- `app/src/main/assets/index.html`
  - JavaScript WebSocket API
  - WebCodecs decoder
  - UI controls

### Testing Scripts
- `scripts/ws_probe.py` - Connection tester
- `scripts/ws_cmd.py` - Command sender
- `scripts/ws_save_h264.py` - Video recorder

---

## Related Documentation

- [ROS2_TOPICS_REFERENCE.md](ROS2_TOPICS_REFERENCE.md) - ROS2 topic details
- [THREE_WAY_CAMERA_CONTROL.md](THREE_WAY_CAMERA_CONTROL.md) - Control architecture
- [NETWORK_PROTOCOLS.md](NETWORK_PROTOCOLS.md) - Protocol specifications
- [README.md](../README.md) - Main project documentation
