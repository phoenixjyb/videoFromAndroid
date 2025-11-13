# CamViewer Architecture Clarifications

**Date**: November 11, 2025  
**Status**: Architecture Updated

## Key Clarifications from User

### 1. CamViewer ≈ Web Browser Viewer

**User's Statement**:
> "webViewer is somehow equivalent to camViewer, because raw videos are allowed to be viewed on it"

**What This Means**:
- **For video viewing**: CamViewer and Web Browser are equivalent
- Both connect to CamControl :9090 as video clients
- Both receive the same H.264/H.265 video stream
- Both can display live video to users
- **CamViewer is a superset**: It adds features beyond what Web Browser provides

### 2. Direct Orin Communication (Bypasses CamControl)

**User's Statement**:
> "in camViewer, we need to also it directly talk to Orin, and send stuff to Orin (not necessarily via the camControl)"

**What This Means**:
- **CamViewer → Orin** is a DIRECT connection
- Port: CamViewer connects to Orin :8080
- **Does NOT go through CamControl**
- Separate communication channel from video streaming
- Used for sending target selection coordinates
- **Important**: This is for target coordinates ONLY, not video

### 3. Developer Mode = WebUI Equivalent

**User's Statement**:
> "camViewer can have a developer mode, in which it is effectively having the same function as webUI mode"

**What This Means**:
- **Developer Mode** is an optional feature in CamViewer
- When enabled: CamViewer has **same camera control capabilities as WebUI**
- Can send camera control commands to CamControl :9090
- Includes: zoom, exposure, codec selection, bitrate, camera switching, etc.
- **Normal Mode** (default): Just video viewing + target selection (no camera controls)
- **Developer Mode** (toggle): Video + target selection + camera controls

## Updated Architecture Understanding

### Component Roles Clarified

#### CamControl (Android Phone)
- **Role**: VIDEO SOURCE + COMMAND RECEIVER
- **Port 9090**: Broadcasts video to all clients, receives control commands from any client
- **Serves**: Multiple simultaneous clients (one-to-many broadcast)

#### Orin (Jetson)
- **Role 1**: VIDEO CLIENT (connects to CamControl :9090)
  - Receives video stream
  - Publishes to ROS2 `/recomo/rgb`
  
- **Role 2**: TARGET API SERVER (listens on :8080)
  - Receives target coordinates from CamViewer
  - Publishes to ROS2 `/target_roi`

#### CamViewer (Android Tablet)
- **Role 1**: VIDEO CLIENT (connects to CamControl :9090)
  - Receives video stream (like Web Browser)
  - Displays to user
  - **Normal Mode**: Video viewing + target selection
  - **Developer Mode**: + Camera control (equivalent to WebUI)
  
- **Role 2**: TARGET SELECTION CLIENT (connects to Orin :8080)
  - Sends target coordinates DIRECTLY to Orin
  - Bypasses CamControl entirely
  - HTTP POST or WebSocket

#### Web Browser
- **Role**: VIDEO CLIENT (connects to CamControl :9090)
  - Receives video stream
  - WebUI for camera control
  - **Note**: CamViewer ≈ Web Browser + target selection

## Communication Channels

### Channel 1: Video Broadcasting (CamControl → All Clients)
```
CamControl :9090 (SERVER)
    ↓ H.264/H.265 video broadcast
    ├──→ Orin (client)
    ├──→ CamViewer (client)
    └──→ Browser (client)

✅ One-to-many broadcast
✅ All clients receive same stream
✅ No client is "special"
```

### Channel 2: Camera Control (Any Client → CamControl)
```
Clients:
- Web Browser (WebUI controls)
- CamViewer (developer mode)
- Orin (optional automation)
    ↓ JSON commands via WS
CamControl :9090 (RECEIVER)
    ↓ Apply to camera
    ↓ Broadcast telemetry
All Clients (receive updates)

✅ Many-to-one
✅ Any client can control
✅ Telemetry synchronized
```

### Channel 3: Target Selection (CamViewer → Orin DIRECT)
```
CamViewer (user draws bbox)
    ↓ Target coordinates only (NOT video)
    ↓ HTTP POST or WebSocket
Orin :8080 (TARGET API SERVER)
    ↓ Validate coordinates
    ↓ Publish to ROS2
ROS2 /target_roi topic

✅ Point-to-point
✅ BYPASSES CamControl
✅ Separate from video channel
✅ Coordinates only, not video
```

## Feature Comparison Matrix

| Feature | Web Browser | CamViewer Normal | CamViewer Dev Mode |
|---------|-------------|------------------|---------------------|
| **Video Viewing** | ✅ Yes | ✅ Yes | ✅ Yes |
| **Connection to CamControl** | ✅ :9090 | ✅ :9090 | ✅ :9090 |
| **Camera Control** | ✅ WebUI | ❌ No | ✅ Yes (equivalent) |
| **Telemetry Display** | ✅ Yes | ❌ No | ✅ Yes |
| **Target Selection** | ❌ No | ✅ Yes | ✅ Yes |
| **Direct Orin Connection** | ❌ No | ✅ :8080 | ✅ :8080 |
| **Touch Interface** | ❌ No | ✅ Yes | ✅ Yes |

## What CamViewer Is and Isn't

### ✅ CamViewer IS:
- A video client (like Web Browser)
- An enhanced viewer with target selection
- Equivalent to WebUI in developer mode
- A direct communicator with Orin (for targets)
- A tablet-optimized touch interface

### ❌ CamViewer IS NOT:
- A video source (CamControl is the source)
- A video relay (doesn't re-broadcast)
- A replacement for CamControl (they coexist)
- Always showing camera controls (normal mode hides them)
- Sending video to Orin (only coordinates)

## Mode Toggle Design

### Normal Mode (Default - For Operators)
**Purpose**: Simple interface for target selection
- **Visible**:
  - Video display (full screen)
  - Target selection overlay
  - Minimal controls (target, settings)
  - Connection status
  
- **Hidden**:
  - Camera controls (zoom, exposure, etc.)
  - Telemetry details
  - Developer diagnostics

### Developer Mode (Optional - For Technicians)
**Purpose**: Full control equivalent to WebUI
- **Visible**:
  - Everything from Normal Mode, PLUS:
  - Camera control panel (zoom, codec, bitrate, etc.)
  - Real-time telemetry (AF, AE, ISO, FPS)
  - Connection diagnostics
  - Encoder configuration
  
- **Toggle**: Settings menu or hidden gesture/button

## Implementation Impact

### Phase 2 (Video Streaming)
- Implement WebSocket client to CamControl :9090
- Same protocol as Web Browser client
- MediaCodec decoder (like Web Browser uses WebCodecs)
- **Reference**: Look at existing Web UI code for protocol details

### Phase 3 (Target Selection)
- Implement **DIRECT** HTTP/WebSocket client to Orin :8080
- Does **NOT** go through CamControl
- New protocol (to be defined with Orin team)
- Independent of video streaming channel

### Phase 5 (Developer Mode)
- **Camera Control UI**: Implement controls equivalent to WebUI
  - Zoom slider, camera switch, codec selector, etc.
  - Use **same commands** as WebUI sends to CamControl
  - Connect to CamControl :9090 (same as video channel)
  
- **Telemetry Display**: Parse telemetry from CamControl
  - Same format as WebUI receives
  - Display AF, AE, ISO, exposure, FPS, etc.
  
- **Mode Toggle**: Settings screen or developer gesture
  - Save preference (persist across restarts)
  - Default to Normal Mode for production

## Network Diagram (Updated)

```
┌──────────────────────────────────────────────────────────┐
│                    WiFi Network                          │
└──────────────────────────────────────────────────────────┘

         ┌─────────────────┐
         │   CamControl    │ 192.168.1.100
         │   (Phone)       │
         │   VIDEO SOURCE  │
         │                 │
         │   :9090 Server  │ (video broadcast + control receiver)
         └────────┬────────┘
                  │
    ┌─────────────┼──────────────┐
    │             │              │
    │             │              │
    ▼             ▼              ▼
┌────────┐  ┌──────────┐  ┌──────────┐
│  Orin  │  │CamViewer │  │ Browser  │
│  .200  │  │  .101    │  │  .50     │
│        │  │          │  │          │
│ VIDEO  │  │ VIDEO    │  │ VIDEO    │
│ CLIENT │  │ CLIENT   │  │ CLIENT   │
│ :9090  │  │ :9090    │  │ :9090    │
│        │  │          │  │          │
│        │  │ Developer│  │ WebUI    │
│        │  │ Mode:    │  │ Controls │
│        │  │ Controls │  │          │
│        │  │ :9090    │  │          │
│        │  │          │  │          │
│ TARGET │◄─┤ TARGET   │  │          │
│ API    │  │ CLIENT   │  │          │
│ :8080  │  │ :8080    │  │          │
│ SERVER │  │ (DIRECT) │  │          │
└────┬───┘  └──────────┘  └──────────┘
     │
     │ ROS2 DDS
     ▼
┌──────────────┐
│  ROS2 Topics │
│ /recomo/rgb  │ (video frames)
│ /target_roi  │ (target coords)
└──────────────┘
```

## Summary of Changes

### Before Clarification
- ❌ Thought CamViewer was just a video client
- ❌ Didn't clarify relationship to Web Browser
- ❌ Unclear about developer mode functionality

### After Clarification
- ✅ CamViewer ≈ Web Browser for video viewing
- ✅ CamViewer adds target selection (unique feature)
- ✅ Developer mode = WebUI equivalent (camera control)
- ✅ Direct Orin communication (bypasses CamControl)
- ✅ Two independent channels: video + target selection

## Next Steps

1. **Review this document** ✅
2. **Confirm understanding** ⏳
3. **Proceed with implementation** ⏳
   - Phase 1: Project setup
   - Phase 2: Video client (like Web Browser)
   - Phase 3: Target selection (direct to Orin)
   - Phase 5: Developer mode (equivalent to WebUI)

---

**Document Status**: ✅ Updated with clarifications  
**Architecture Status**: ✅ Validated and correct  
**Ready for Implementation**: ✅ Yes

