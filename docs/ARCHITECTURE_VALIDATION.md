# Architecture Validation & Review

## ✅ Corrected Architecture

Based on user clarification on November 11, 2025, the architecture has been corrected and validated.

## System Roles (CORRECT)

### CamControl (Android Phone)
**Role**: VIDEO SOURCE
- ✅ Captures video from camera
- ✅ Encodes with MediaCodec (H.264/H.265)
- ✅ Broadcasts via WebSocket :9090
- ✅ Serves MULTIPLE clients simultaneously
- ✅ Receives control commands from any client

### Orin (Jetson Backend)
**Role**: VIDEO CLIENT #1 + TARGET API SERVER (Dual Role)

**As Video Client** (Already Implemented ✅):
- ✅ Connects to CamControl WS :9090
- ✅ Receives H.264/H.265 video stream
- ✅ Decodes with GStreamer (nvv4l2decoder)
- ✅ Publishes to ROS2 `/recomo/rgb` topic
- ✅ Optional RTSP restream on :8554

**As Target API Server** (To Implement ⏳):
- ⏳ HTTP/WebSocket server on :8080
- ⏳ Receives target ROI from CamViewer
- ⏳ Publishes to ROS2 `/target_roi` topic
- ⏳ Validates coordinates

### CamViewer (Android Tablet)
**Role**: VIDEO CLIENT #2 + USER INTERACTION

**Relationship to Web Browser Client**:
- ✅ **CamViewer ≈ Web Browser** for video viewing
- ✅ **CamViewer > Web Browser** with added target selection
- ✅ **Developer Mode ≈ WebUI** for camera control

**As Video Client** (same as browser):
- ⏳ Connects to CamControl WS :9090
- ⏳ Receives H.264/H.265 video stream (same as Orin)
- ⏳ Decodes with MediaCodec
- ⏳ Displays on SurfaceView

**As User Interface** (unique features):
- ⏳ Touch-based bounding box selection
- ⏳ Coordinate conversion (screen → frame)
- ⏳ **DIRECT connection to Orin :8080** (bypasses CamControl)
- ⏳ Sends target ROI (not video!)

**Developer Mode** (optional, equivalent to WebUI):
- ⏳ Camera control UI (zoom, exposure, codec, etc.)
- ⏳ Sends control commands to CamControl :9090
- ⏳ Real-time telemetry display
- ⏳ Same functionality as Web UI controls

### Web Browser
**Role**: VIDEO CLIENT #3 (Already Implemented ✅)
- ✅ Connects to CamControl WS :9090
- ✅ Receives H.264/H.265 video stream
- ✅ Decodes with WebCodecs/Broadway
- ✅ Displays in HTML Canvas
- ✅ Sends control commands (WebUI)

**Note**: CamViewer is equivalent to Web Browser for viewing, with additional target selection capability.

## Data Flow Validation

### ✅ Video Distribution (One Source → Many Clients)
```
CamControl :9090 (SOURCE)
    │
    ├───→ Orin (Client #1) → ROS2 /recomo/rgb
    ├───→ CamViewer (Client #2) → Display to user
    └───→ Browser (Client #3) → Display in web page

✅ Single source, multiple consumers
✅ All clients receive the SAME video
✅ Broadcast model, not relay
```

### ✅ Target Selection (CamViewer → Orin → ROS2)
```
User (draws bbox on CamViewer)
    ↓
CamViewer (calculates coordinates)
    ↓
HTTP POST to Orin :8080
    ↓
Orin (validates & publishes)
    ↓
ROS2 /target_roi topic
    ↓
Downstream tracking nodes

✅ CamViewer does NOT send video to Orin
✅ Orin already has video from CamControl
✅ Only coordinates are sent, not frames
```

### ✅ Camera Control (Any Client → CamControl)
```
Any Client (CamViewer dev mode, Browser WebUI, Orin automation)
    ↓
Control Command (JSON via WS :9090)
    ↓
CamControl (applies to camera)
    ↓
Telemetry Update (broadcast to ALL)
    ↓
All Clients (receive update)

✅ Any client can send commands
✅ CamViewer developer mode = WebUI controls
✅ Changes broadcast to all clients
✅ Telemetry synchronized across all
```

## Port Summary

| Component | Port | Role | Protocol |
|-----------|------|------|----------|
| CamControl | 9090 | Video broadcast + command receiver | WebSocket |
| Orin | 9090 | Video client (connects TO CamControl) | WebSocket Client |
| Orin | 8080 | Target API server (receives FROM CamViewer) | HTTP/WebSocket Server |
| Orin | 8554 | RTSP restream (optional) | RTSP |
| CamViewer | 9090 | Video client (connects TO CamControl) | WebSocket Client |
| CamViewer | 8080 | Target API client (connects TO Orin) | HTTP Client |
| Browser | 9090 | Video client (connects TO CamControl) | WebSocket |

## Network Topology

```
WiFi Network: 192.168.1.0/24

┌──────────────┐ 192.168.1.100
│  CamControl  │ :9090 (SERVER - broadcasts video)
└───────┬──────┘
        │
        │ (All clients connect here for video)
        │
        ├────────────────────┬─────────────────┐
        │                    │                 │
        ▼                    ▼                 ▼
┌──────────────┐     ┌──────────────┐   ┌──────────┐
│ Orin (Jetson)│     │  CamViewer   │   │ Browser  │
│ .200         │     │  .101        │   │ .50      │
│              │     │              │   │          │
│ :9090 client │     │ :9090 client │   │ WS client│
│ :8080 server │◄────┤ :8080 client │   │          │
│              │     │ (sends bbox) │   │          │
└──────────────┘     └──────────────┘   └──────────┘
        │
        │ ROS2 DDS
        ▼
┌──────────────┐
│  ROS2 Topics │
│ /recomo/rgb  │
│ /target_roi  │
└──────────────┘
```

## What Each Component Does NOT Do

### CamControl Does NOT:
- ❌ Consume video from other sources
- ❌ Connect to Orin as a client
- ❌ Process or track targets
- ❌ Run ROS2 nodes

### Orin Does NOT:
- ❌ Capture video (gets it from CamControl)
- ❌ Send video to CamViewer (CamViewer gets it from CamControl)
- ❌ Run on Android (runs on Jetson Linux)
- ❌ Have a camera (relies on CamControl's camera)

### CamViewer Does NOT:
- ❌ Capture its own video (gets it from CamControl)
- ❌ Stream video to Orin (Orin gets it from CamControl)
- ❌ Run tracking algorithms (only sends target selections)
- ❌ Publish to ROS2 directly (goes through Orin)

## Key Insights

### 1. CamViewer is an Enhanced Web Browser Client
- **Video viewing**: CamViewer ≈ Web Browser (both connect to CamControl :9090)
- **Camera control**: CamViewer developer mode = WebUI controls (equivalent functionality)
- **Enhancement 1**: Target selection capability (Web Browser doesn't have this)
- **Enhancement 2**: Direct Orin communication for targets (Web Browser doesn't need this)
- **Enhancement 3**: Touch-optimized tablet interface

### 2. CamControl is a Broadcaster, Not a Point-to-Point Server
- Multiple clients can connect simultaneously
- All clients receive the same stream
- No client is "special" or "primary"
- Commands from any client affect all clients

### 2. Orin Has Dual Independent Roles
- **Role 1**: Video consumer (like CamViewer and Browser)
- **Role 2**: Target API server (unique to Orin)
- These roles are independent and non-overlapping

### 3. Video and Target Selection are Separate Channels
- **Video channel**: CamControl :9090 → All Clients (broadcast)
- **Target channel**: CamViewer → Orin :8080 (point-to-point, DIRECT)
- **Camera control**: Any Client → CamControl :9090 (many-to-one)
- No video data in target selection messages
- Only coordinates, timestamps, and metadata
- Target selection bypasses CamControl entirely

### 4. CamViewer and Orin Both Consume the Same Video
- Both connect to CamControl :9090
- Both decode the same H.264/H.265 stream
- CamViewer uses it for display + UI
- Orin uses it for ROS2 publishing + tracking

### 5. All Clients Can Control the Camera (If Enabled)
- Any client can send commands to CamControl :9090
- **Web Browser**: Uses WebUI controls
- **CamViewer**: Uses developer mode (equivalent to WebUI)
- **Orin**: Can send automated commands
- CamControl broadcasts telemetry to ALL clients
- State is synchronized across all clients
- Developer mode in CamViewer is optional (can be disabled for operators)

## Implementation Priority

### Phase 1: Basic CamViewer Video Client ✅ Correct
1. Connect to CamControl :9090 (WebSocket client)
2. Receive video frames
3. Decode with MediaCodec
4. Display on SurfaceView

### Phase 2: Target Selection UI ✅ Correct
1. Implement bounding box drawing
2. Calculate coordinates
3. Validate and format payload

### Phase 3: Orin Target API ✅ Correct
1. Create FastAPI server on :8080
2. Receive POST requests from CamViewer
3. Publish to ROS2 /target_roi

### Phase 4: Optional Features ✅ Correct
1. Developer mode controls in CamViewer
2. Tracking feedback from Orin to CamViewer
3. Multiple target support
4. Target persistence

## Validation Checklist

- [x] CamControl is defined as VIDEO SOURCE (not just "server")
- [x] Orin is defined as VIDEO CLIENT + TARGET API SERVER (dual role)
- [x] CamViewer is defined as VIDEO CLIENT + UI (not video source)
- [x] Video flow is one-to-many (CamControl → Multiple Clients)
- [x] Target flow is point-to-point (CamViewer → Orin)
- [x] No video is sent from CamViewer to Orin
- [x] Orin gets video from CamControl, not from CamViewer
- [x] All clients can control camera via CamControl
- [x] Port assignments are clear and non-conflicting
- [x] ROS2 topics are on Orin, not on Android devices
- [x] Network topology supports multiple simultaneous clients

## Common Misconceptions (Corrected)

### ❌ WRONG: "CamViewer sends video to Orin"
✅ CORRECT: CamViewer and Orin BOTH get video from CamControl independently

### ❌ WRONG: "Orin is a relay between CamControl and CamViewer"
✅ CORRECT: Orin is a client that receives video directly from CamControl (like CamViewer)

### ❌ WRONG: "CamControl serves video to Orin, which then serves to CamViewer"
✅ CORRECT: CamControl broadcasts to both Orin AND CamViewer simultaneously

### ❌ WRONG: "Only CamViewer can control the camera"
✅ CORRECT: Any client can send control commands:
- Browser uses WebUI controls
- CamViewer uses developer mode (equivalent to WebUI)
- Even Orin can send automated commands

### ❌ WRONG: "Target selection goes through CamControl"
✅ CORRECT: Target selection goes directly from CamViewer to Orin (bypasses CamControl)

## Architecture Approval

**Status**: ✅ VALIDATED  
**Date**: November 11, 2025  
**Reviewed By**: User  
**Changes**: Corrected to clarify CamControl as broadcast source, not point-to-point server

## Next Steps

1. ✅ Architecture documentation updated
2. ⏳ Begin Phase 1: CamViewer basic video client
3. ⏳ Create Gradle module for camviewer
4. ⏳ Implement WebSocket client
5. ⏳ Implement video decoder

**Ready to proceed with implementation.**

---

**Document Version**: 1.1 (Corrected)  
**Last Updated**: November 11, 2025  
**Status**: Ready for Implementation
