# System Architecture Overview

**Last Updated**: November 13, 2025  
**Status**: Production - Fully Operational

## Purpose

This document provides a high-level overview of the complete CamControl system architecture. For detailed specifications, see the reference documents listed at the end.

## System Components

The system consists of three main components working together:

1. **CamControl** (Android Phone) — **VIDEO SOURCE**: Camera capture, H.265 encoding, and WebSocket broadcast
2. **CamViewer** (Android Tablet) — **VIDEO CLIENT**: Real-time viewer with camera controls and target selection
3. **Orin** (Jetson Backend) — **PROCESSING HUB**: ROS2 bridge, camera control relay, target API, and media server

## Core Architecture Principle

**CamControl broadcasts to multiple clients simultaneously:**
- All clients receive the same H.265 video stream via WebSocket :9090
- Any client can send camera control commands via WebSocket :9090/control
- Orin bridges video to ROS2 topics and relays ROS2 commands back to phone
- Three-way control: WebUI (primary dev), CamViewer (field ops), ROS2 topics (automation)

## High-Level Architecture

```
                    ┌─────────────────────────────────────────┐
                    │      CamControl (Phone)                 │
                    │      📹 VIDEO SOURCE                     │
                    │                                         │
                    │  • Camera2 API + MediaCodec H.265       │
                    │  • WebSocket Server :9090               │
                    │  • Broadcasts to all clients            │
                    │  • Receives control commands            │
                    └────────────┬────────────────────────────┘
                                 │
                                 │ ws://phone-ip:9090
                                 │ (video + telemetry + control)
                    ┌────────────┴────────────┬───────────────┐
                    │                         │               │
                    ▼                         ▼               ▼
        ┌───────────────────┐   ┌───────────────────┐  ┌──────────┐
        │  Orin (Jetson)    │   │  CamViewer        │  │ Browser  │
        │  🎯 PROCESSING     │   │  📱 FIELD OPS      │  │ 💻 DEV   │
        │                   │   │                   │  │          │
        │ • Video→ROS2      │   │ • Video Display   │  │ • WebUI  │
        │   /recomo/rgb     │   │ • Dev Controls    │  │ • Camera │
        │ • ROS2→Commands   │   │ • Target Select   │  │   Control│
        │   /camera/*       │   │ • Media Browser   │  │ • Primary│
        │ • Target API      │◄──┤   → Orin :8082    │  │   Tool   │
        │   :8082           │   │ • Media Download  │  │          │
        │ • Media API       │◄──┤   → Orin :8081    │  │          │
        │   :8081           │   │                   │  │          │
        └───────┬───────────┘   └───────────────────┘  └──────────┘
                │
                │ DDS
                ▼
        ┌───────────────┐
        │  ROS2 Topics  │
        │               │
        │ • /recomo/rgb │ ← Video stream
        │ • /camera/*   │ → Camera control (7 topics)
        │ • /target_roi │ ← Target selection
        └───────────────┘
```

## Component Summary

### 1. CamControl (Phone Camera App)

**Role**: Video source and command receiver  
**Package**: `com.example.camcontrol`  
**Hardware**: Android phone with Camera2 API support

**Key Features**:
- Camera2 API for camera hardware access
- MediaCodec H.265 encoder (default, H.264 fallback)
- Ktor WebSocket server on port 9090
- Broadcasts to unlimited concurrent clients
- Accepts commands from any connected client
- On-device MP4 recording with MediaMuxer
- Default bitrate: ~8.2 Mbps (1080p30 H.265)

**Network Endpoints**:
- `ws://phone-ip:9090/` - Video stream + telemetry (broadcast)
- `ws://phone-ip:9090/control` - Camera commands (receive)
- `http://phone-ip:9090/` - WebUI HTML page

**Status**: ✅ Fully operational

---

### 2. CamViewer (Tablet Viewer App)

**Role**: Field operations client with enhanced features  
**Package**: `com.example.camviewer`  
**Hardware**: Android tablet

**Key Features**:
- MediaCodec H.265 video decoder
- Real-time video playback on SurfaceView
- **Developer mode** camera controls (complementary to WebUI)
- Target/ROI selection (tap/drag gestures)
- Bounding box visualization
- Media browser (browse/download videos from Orin)
- Settings management (phone IP, Orin IP, dev mode toggle)

**Network Connections**:
- `ws://phone-ip:9090/` - Receive video (WebSocket client)
- `ws://phone-ip:9090/control` - Send commands (WebSocket client)
- `http://orin-ip:8082/target` - Send target ROI (HTTP POST)
- `http://orin-ip:8081/media/*` - Browse/download media (HTTP GET)

**Status**: ✅ Fully operational

---

### 3. Orin (Jetson Backend)

**Role**: Processing hub with multiple services  
**Hardware**: NVIDIA Jetson Orin (or similar)

**Four Main Services**:

#### A. Phone-ROS2 Bridge (`ws_to_image` node)
- Connects to phone WebSocket :9090
- GStreamer H.265 decode (NVDEC hardware acceleration)
- RGB conversion and ROS2 publishing
- Publishes to `/recomo/rgb` topic (~10 Hz)
- Publishes to `/recomo/camera_info` topic
- Downscales 1920×1080 → 640×480 for efficiency

#### B. Camera Control Relay (`camera_control_relay.py`)
- Subscribes to 7 ROS2 `/camera/*` topics
- Forwards commands to phone via WebSocket :9090/control
- Enables ROS2-based automation and control
- Async command queue prevents blocking

#### C. Target API Server (`target_api.py` on :8082)
- HTTP REST API for target selection
- Receives ROI from CamViewer (normalized coordinates)
- Transforms coordinates using camera_info
- Publishes to ROS2 `/target_roi` topic
- WebSocket support for real-time updates

#### D. Media API Server (`media_api.py` on :8081)
- HTTP REST API for media management
- Serves videos from `saved_videos/` directory
- Thumbnail generation and caching
- Download with Range header support (resume capability)
- Delete functionality
- Metadata query (resolution, duration, codec, etc.)

**Network Endpoints**:
- `ws://phone-ip:9090/` - Video consumer (client)
- `ws://phone-ip:9090/control` - Command sender (client)
- `http://orin-ip:8082/*` - Target API (server)
- `http://orin-ip:8081/*` - Media API (server)

**ROS2 Topics**:
- **Publishes**: `/recomo/rgb`, `/recomo/camera_info`, `/target_roi`
- **Subscribes**: `/camera/*` (7 control topics)

**Status**: ✅ Fully operational

---

### 4. WebUI (Browser Interface)

**Role**: Primary development and testing interface  
**Technology**: HTML5 + JavaScript (WebCodecs API)  
**Access**: `http://phone-ip:9090/`

**Key Features**:
- WebCodecs API for H.265 decode (Safari 16.4+, Chrome 94+)
- Broadway.js fallback for H.264 (Firefox)
- Real-time video display on Canvas
- Camera control UI (zoom, switch, bitrate, codec, AE/AWB locks)
- Telemetry display (resolution, FPS, codec, bitrate)
- Decoder type indicator
- **Primary tool for developers** (not CamViewer developer mode)

**Status**: ✅ Fully operational

#### Communication Protocol
```json
// Control Commands (received)
{"cmd": "setZoomRatio", "value": 2.5}
{"cmd": "switchCamera", "facing": "front"}
{"cmd": "setVideoProfile", "width": 1920, "height": 1080, "fps": 30}
{"cmd": "setBitrate", "bitrate": 8000000}
{"cmd": "setCodec", "codec": "h265"}
{"cmd": "startRecording", "name": "optional_filename"}
{"cmd": "stopRecording"}

// Telemetry (sent)
## Communication Protocols

All network communication uses **WiFi on port 9090** (phone) and additional Orin services on **8081** (media) and **8082** (target).

### Video Streaming
- **Protocol**: WebSocket binary frames
- **Format**: H.265 Annex-B NAL units (default), H.264 fallback
- **Bitrate**: ~8.2 Mbps (1080p30, configurable 2-50 Mbps)
- **Latency**: 50-150ms end-to-end

### Camera Control
- **Protocol**: WebSocket JSON text frames
- **Discriminator**: `"cmd"` field (not `"type"`)
- **Commands**: 7 types (zoom, ae_lock, awb_lock, switch, bitrate, codec, key_frame)

### Target Selection
- **Protocol**: HTTP POST to Orin :8082
- **Format**: JSON with normalized coordinates
- **Flow**: CamViewer → Orin → ROS2 `/target_roi`

### Media Retrieval
- **Protocol**: HTTP REST API from Orin :8081
- **Features**: List, thumbnail, download, delete
- **Support**: Range headers for resume capability

## Data Flows

### Video Distribution (1→Many)
```
Phone Camera → H.265 Encode → WebSocket :9090 → Broadcast
                                    ↓
                    ┌───────────────┼───────────────┐
                    ↓               ↓               ↓
                  Orin          CamViewer        Browser
                (GStreamer)    (MediaCodec)    (WebCodecs)
                    ↓
                ROS2 /recomo/rgb
```

### Camera Control (Many→1)
```
WebUI / CamViewer / ROS2 → JSON Command → Phone :9090/control
                                            ↓
                                    Camera2 API applies
                                            ↓
                            Telemetry broadcast to all
```

### Target Tracking (CamViewer→Orin→ROS2)
```
User Selection → CamViewer → HTTP POST :8082 → Orin
                                                  ↓
                                          ROS2 /target_roi
```

## Port Summary

| Service | Port | Protocol | Purpose |
|---------|------|----------|---------|
| Phone WebSocket | 9090 | WS/HTTP | Video + Control |
| Orin Target API | 8082 | HTTP | ROI selection |
| Orin Media API | 8081 | HTTP | Media browse/download |
| ROS2 DDS | N/A | DDS | Internal topics |

## Project Status

### ✅ Fully Operational
- Video streaming (H.265) to all clients
- Three-way camera control (WebUI, CamViewer, ROS2)
- Phone-to-ROS2 bridge (`/recomo/rgb`)
- ROS2-to-phone relay (`/camera/*` topics)
- Target selection and ROI publishing
- Media retrieval API
- On-device recording

### 🚧 Future Enhancements
- ROS2 video publisher optimization
- Complete vision pipeline (tracking loop)
- Production deployment packaging
- Authentication and encryption

## Implementation Files

### Phone (CamControl)
- `app/src/main/java/com/example/camcontrol/`
  - `CamControlService.kt` - Main service, WebSocket server
  - `MainActivity.kt` - Camera2 pipeline
  - `encode/VideoEncoder.kt` - H.265 encoder
  - `transport/ControlServer.kt` - Ktor routes

### Tablet (CamViewer)
- `camviewer/src/main/java/com/example/camviewer/`
  - `MainActivity.kt` - Main app
  - `network/PhoneCameraClient.kt` - WebSocket client
  - `video/VideoDecoder.kt` - H.265 decoder
  - `network/OrinTargetClient.kt` - Target API client
  - `ui/screens/` - Video display, controls, media browser

### Orin Services
- `orin/start_phone_ros2_bridge.sh` - Video→ROS2 bridge
- `orin/camera_control_relay.py` - ROS2→phone commands
- `orin/target_api.py` - Target selection API
- `orin/media_api.py` - Media management API
- `orin/start_all_services.sh` - Start all services
- `orin/stop_all_services.sh` - Stop all services

## Related Documentation

For detailed specifications, see:

- **[WEBSOCKET_ARCHITECTURE.md](WEBSOCKET_ARCHITECTURE.md)** - Complete WebSocket protocol details
- **[ROS2_TOPICS_REFERENCE.md](ROS2_TOPICS_REFERENCE.md)** - All ROS2 topics and usage
- **[THREE_WAY_CAMERA_CONTROL.md](THREE_WAY_CAMERA_CONTROL.md)** - Camera control implementation
- **[NETWORK_PROTOCOLS.md](NETWORK_PROTOCOLS.md)** - Network protocol specifications
- **[../README.md](../README.md)** - Main project documentation

---

**Last Updated**: November 13, 2025  
**System Version**: v1.0 - Production Ready
```

### 2. CamViewer (ENHANCED VIDEO CLIENT - Android Tablet) — NEW
**Package**: `com.example.camviewer`  
**Application ID**: `com.example.camviewer`  
**Role**: Enhanced video consumer with target selection, media retrieval, and optional camera control

#### Relationship to Web UI
**WebUI is the PRIMARY developer interface** for CamControl:
- **WebUI Purpose**: Direct phone interaction, testing, primary development tool
- **CamViewer Purpose**: Operator interface with additional Orin integration

**CamViewer ≈ WebViewer + Enhanced Features**

- **Basic Viewing**: Same as WebViewer — connects to CamControl :9090, receives video, displays to user
- **Enhancement 1**: Direct communication with Orin for target selection (WebViewer doesn't have this)
- **Enhancement 2**: Media retrieval from Orin (download recorded videos/images)
- **Enhancement 3**: Optional developer mode with camera control (complementary to WebUI)

#### Operating Modes

**Normal Mode** (Default - Operator/User View):
- ✅ View live video from CamControl (same as WebViewer)
- ✅ Select targets via bounding box or long-press
- ✅ Send target coordinates **directly to Orin** (not via CamControl)
- ✅ Browse and download recorded media from Orin (videos/images)
- ❌ No camera control features (simplified interface)

**Developer Mode** (Advanced - Complementary to WebUI):
- ✅ All Normal Mode features (video + target selection + media retrieval)
- ✅ **Camera control** (complementary to WebUI, for field use)
- ✅ Exposure, zoom, focus, ISO controls
- ✅ Camera mode selection, codec selection
- ✅ Real-time telemetry display
- ✅ Sends camera commands to CamControl :9090
- ⚠️ **Note**: WebUI is the PRIMARY tool for development/testing

#### Key Features
- **Video Streaming (Client)**
  - WebSocket client connects to CamControl at `ws://<camcontrol-ip>:9090/control`
  - Receives H.264/H.265 video stream (same as all other clients)
  - MediaCodec hardware-accelerated decoding
  - Low-latency video rendering (SurfaceView/TextureView)
  - **Does NOT re-stream or re-broadcast video**

- **User Interaction**
  - Live video display with touch interaction
  - Bounding box drawing (drag to create selection)
  - Long-press target selection (creates default-sized bbox)
  - Visual feedback for selections
  - Pan/zoom controls (optional)

- **Target Selection & Communication**
  - Interactive bounding box selection on video
  - Convert screen coordinates to image frame coordinates
  - **Send target ROI DIRECTLY to Orin** (bypasses CamControl)
  - Target API: `POST http://<orin-ip>:8080/api/target` or `ws://<orin-ip>:8080/tracking`
  - **Independent channel from video streaming**
  - Orin publishes to ROS2 `/target_roi` topic

- **Media Retrieval from Orin (NEW)**
  - Browse recorded videos and images stored on Orin
  - Download media files to tablet for viewing/sharing
  - Media Gallery UI with thumbnails and metadata
  - Filter by date, type, or tracking session
  - Media API: `GET http://<orin-ip>:8081/api/media/*`
  - Progressive download with resume capability
  - Local caching and organization

- **Developer Mode (Optional - Toggle in Settings)**
  - Camera control interface (sends commands to CamControl :9090)
  - Real-time telemetry display (receives from CamControl)
  - Connection status monitoring
  - Encoder configuration controls
  - **Complementary to WebUI** (WebUI is primary for development/testing)
  - Useful for field adjustments when laptop not available

#### Network Connections

**Connection 1 - To CamControl** (WebSocket Client): 
- **Endpoint**: `ws://<camcontrol-ip>:9090/control`
- **Protocol**: WebSocket (binary + JSON text frames)
- **Purpose**: Video streaming + camera control (dev mode)
- **Data Flow**: 
  - **Receives**: H.264/H.265 binary frames + JSON telemetry
  - **Sends**: JSON control commands (only in developer mode)
- **Role**: Same connection model as Web Browser client
  
**Connection 2 - To Orin (Target Selection)** (HTTP/WebSocket Client):
- **Endpoint**: `http://<orin-ip>:8080/api/target` (HTTP POST) OR `ws://<orin-ip>:8080/tracking` (WebSocket)
- **Protocol**: HTTP REST API or WebSocket
- **Purpose**: Send target ROI coordinates
- **Data Flow**: 
  - **Sends**: JSON payload with bounding box coordinates
  - **Receives**: Confirmation + optional tracking feedback
- **Role**: DIRECT connection — bypasses CamControl entirely
- **Independence**: Separate channel from video streaming

**Connection 3 - To Orin (Media Retrieval)** (HTTP Client - NEW):
- **Endpoints**: 
  - `GET http://<orin-ip>:8081/api/media/list` - List available media
  - `GET http://<orin-ip>:8081/api/media/metadata/{id}` - Get metadata
  - `GET http://<orin-ip>:8081/api/media/thumbnail/{id}` - Get thumbnail
  - `GET http://<orin-ip>:8081/api/media/download/{id}` - Download file
  - `DELETE http://<orin-ip>:8081/api/media/{id}` - Delete file (optional)
- **Protocol**: HTTP REST API with Range requests (for resume)
- **Purpose**: Browse and download recorded videos/images from Orin
- **Data Flow**: 
  - **Sends**: HTTP GET requests with optional Range headers
  - **Receives**: JSON metadata + binary media files (MP4, JPEG, PNG)
- **Role**: Media gallery and file transfer
- **Features**: Progressive download, resume capability, thumbnail previews

#### UI Layout
```
┌─────────────────────────────────────┐
│  [ CamViewer ]         [Dev Mode ▼] │  ← Toolbar
├─────────────────────────────────────┤
│                                     │
│                                     │
│        Video Display Area           │  ← SurfaceView
│      (with touch interaction)       │
│                                     │
│         [  Bounding Box  ]          │  ← Visual overlay
│                                     │
├─────────────────────────────────────┤
│ Status: Connected | FPS: 30 | ...  │  ← Status bar
├─────────────────────────────────────┤
│ [🎯 Target] [⚙️ Settings] [📊 Info] │  ← Action buttons
└─────────────────────────────────────┘
```

#### Developer Mode Panel
```
┌─────────────────────────────────────┐
│  Camera Control                      │
│  • Zoom: [━━━━●━━━] 2.5x            │
│  • Camera: [Front] [Back]           │
│  • Profile: [1920x1080@30fps ▼]    │
│  • Codec: [H.264] [H.265]           │
│  • Bitrate: [8 Mbps]                │
│                                     │
│  Telemetry                          │
│  • AF: FOCUSED_LOCKED               │
│  • AE: CONVERGED                    │
│  • ISO: 250                         │
│  • Exposure: 16.7 ms                │
│  • FPS: 29.8                        │
│                                     │
│  Connection                         │
│  • CamControl: 192.168.1.100:9090  │
│  • Orin: 192.168.1.200:8080        │
│  • Status: ✅ Connected             │
└─────────────────────────────────────┘
```

#### Network Endpoints
- **WebSocket Client**: Connects to `ws://<camcontrol-ip>:9090/control`
  - Receives video and telemetry
  - Sends control commands (in dev mode)
  
- **Target Selection API**: `http://<orin-ip>:8080/target` or `ws://<orin-ip>:8080/tracking`
  ```json
  {
    "type": "roi_selection",
    "timestamp": 1699776000000,
    "bbox": {
      "x": 420,
      "y": 360,
      "width": 200,
      "height": 300
    },
    "frame_resolution": {
      "width": 1920,
      "height": 1080
    },
    "normalized": {
      "x": 0.21875,
      "y": 0.3333,
      "width": 0.1042,
      "height": 0.2778
    }
  }
  ```

#### Module Structure
```
camviewer/
├── build.gradle.kts
├── src/main/
    ├── AndroidManifest.xml
    ├── java/com/example/camviewer/
    │   ├── MainActivity.kt                    # Main activity with navigation
    │   ├── ui/
    │   │   ├── VideoDisplayFragment.kt        # Video view with touch interaction
    │   │   ├── MediaGalleryFragment.kt        # Media browser (NEW)
    │   │   ├── MediaDetailFragment.kt         # Media detail view (NEW)
    │   │   ├── DeveloperModeFragment.kt       # Control panel
    │   │   ├── TargetSelectionView.kt         # Overlay for bbox drawing
    │   │   └── TelemetryView.kt               # Telemetry display
    │   ├── network/
    │   │   ├── CamControlClient.kt            # WebSocket client for CamControl
    │   │   ├── OrinTargetClient.kt            # HTTP/WS client for target selection
    │   │   ├── OrinMediaClient.kt             # HTTP client for media retrieval (NEW)
    │   │   └── ProtocolModels.kt              # Data classes for protocols
    │   ├── video/
    │   │   ├── VideoDecoder.kt                # MediaCodec decoder
    │   │   ├── VideoRenderer.kt               # Surface rendering
    │   │   └── FrameProcessor.kt              # Frame analysis
    │   ├── media/
    │   │   ├── MediaRepository.kt             # Media data management (NEW)
    │   │   ├── MediaDownloader.kt             # Download manager with resume (NEW)
    │   │   ├── ThumbnailCache.kt              # Thumbnail caching (NEW)
    │   │   └── MediaMetadata.kt               # Metadata models (NEW)
    │   ├── selection/
    │   │   ├── BoundingBoxSelector.kt         # Touch-based bbox selection
    │   │   ├── CoordinateConverter.kt         # Screen ↔ image coordinates
    │   │   └── TargetTracker.kt               # Track selection state
    │   └── util/
    │       ├── Logger.kt
    │       └── PreferenceManager.kt           # Settings storage
    └── res/
        ├── layout/
        │   ├── activity_main.xml
        │   ├── fragment_video_display.xml
        │   ├── fragment_media_gallery.xml     # NEW
        │   ├── fragment_media_detail.xml      # NEW
        │   ├── fragment_developer_mode.xml
        │   └── item_media_thumbnail.xml       # NEW
        ├── menu/
        │   └── main_menu.xml                  # Navigation menu
        └── values/
            ├── strings.xml
            └── themes.xml
```

### 3. Orin Backend (VIDEO CLIENT + TARGET API SERVER)
**Role**: Dual-purpose - Video consumer AND target selection receiver

#### Role 1: Video Consumer (Already Implemented ✅)
- **WebSocket client** connects to CamControl at `ws://<camcontrol-ip>:9090/control`
- Receives H.264/H.265 video stream
- GStreamer pipeline: `appsrc → h264parse → nvv4l2decoder → nvvidconv → display/appsink`
- **Publishes to ROS2**: `/recomo/rgb` topic (sensor_msgs/Image)
- Optional RTSP restreaming on port 8554
- Same video source as CamViewer and Web Browser

**Current Status**: Fully implemented and working
- `orin/ws_h264_gst.py` - Display pipeline
- `orin/ws_h264_rtsp_server.py` - RTSP restream
- `orin/ros2_camcontrol/ws_to_image.py` - ROS2 publisher

#### Role 2: Target Selection API Server (NEW - To Implement)
- **HTTP/WebSocket server** on port 8080
- Receives target ROI from CamViewer
- **Publishes to ROS2**: `/target_roi` topic
- Validates and processes target coordinates
- Optional: Send tracking feedback back to CamViewer

**To Implement**:
- `orin/target_api_server.py` - FastAPI server
- ROS2 message type: `TargetROI.msg`
- ROS2 tracking node: `target_tracker_node.py`

#### New Orin Components
```
orin/
├── target_api_server.py              # NEW: HTTP/WS server for target selection
├── ros2_camcontrol/
│   ├── ros2_camcontrol/
│   │   ├── ws_to_image.py            # Existing: video to ROS2
│   │   ├── target_tracker_node.py    # NEW: tracking node
│   │   └── tracking_visualizer.py    # NEW: visualization
│   └── msg/
│       ├── TargetROI.msg             # NEW: ROI message
│       └── TrackingResult.msg        # NEW: tracking result
└── tracking/
    ├── __init__.py
    ├── tracker_interface.py          # NEW: tracking algorithm interface
    └── simple_tracker.py             # NEW: basic tracking implementation
```

#### Target Selection Message (ROS2)
```python
# TargetROI.msg
std_msgs/Header header
float32 x          # Normalized x coordinate (0-1)
float32 y          # Normalized y coordinate (0-1)
float32 width      # Normalized width (0-1)
float32 height     # Normalized height (0-1)
int32 frame_width  # Original frame resolution
int32 frame_height
uint64 timestamp_ms
```

## Communication Flows

### Flow 1: Video Broadcasting (One-to-Many)
```
CamControl (SOURCE) → WebSocket :9090 → ALL CLIENTS (simultaneously)
1. CamControl: Camera capture → MediaCodec encode → H.264 Annex-B
2. CamControl: Broadcast binary frames via WebSocket to ALL connected clients
3. Orin: Receive frames → GStreamer decode → ROS2 /recomo/rgb
4. CamViewer: Receive frames → MediaCodec decode → SurfaceView display
5. Browser: Receive frames → WebCodecs decode → Canvas display

Key Points:
- Single video source (CamControl)
- Multiple simultaneous clients (Orin, CamViewer, Browser)
- All clients receive the SAME video stream
- Broadcast model, not point-to-point
```

### Flow 2: Camera Control (Many-to-One)
```
Any Client → CamControl
1. Client (CamViewer/Browser): User adjusts control (zoom slider, etc.)
2. Client: Send JSON command via WebSocket to CamControl
3. CamControl: Apply command to camera
4. CamControl: Broadcast telemetry update to ALL clients
5. All Clients: Receive telemetry → update UI

Example: CamViewer changes zoom
- CamViewer → CamControl: {"cmd": "setZoomRatio", "value": 2.5}
- CamControl: Apply zoom to camera
- CamControl → ALL clients: {"zoom": 2.5, "fps": 30, ...}
- Orin, CamViewer, Browser: All receive the same telemetry update
```

### Flow 3: Target Selection (CamViewer → Orin → ROS2)
```
CamViewer → Orin → ROS2
1. User draws bounding box on CamViewer video display
2. CamViewer: Convert screen coords → video frame coords
3. CamViewer: Send ROI to Orin via HTTP POST or WebSocket
   POST http://<orin-ip>:8080/target
   {
     "bbox": {"x": 420, "y": 360, "width": 200, "height": 300},
     "frame_resolution": {"width": 1920, "height": 1080},
     "timestamp": 1699776000000
   }
4. Orin: Receive ROI → validate coordinates
5. Orin: Publish to ROS2 /target_roi topic
6. ROS2 Tracking Node: Subscribe to /target_roi and /recomo/rgb
7. ROS2 Tracking Node: Process tracking
8. ROS2 Tracking Node: Publish results to /tracking_result

Key Points:
- Target selection is SEPARATE from video streaming
- CamViewer does NOT send video to Orin
- CamViewer only sends target coordinates
- Orin already has the video from CamControl
```

### Flow 4: Complete Pipeline
```
┌─────────────┐
│ CamControl  │ (VIDEO SOURCE)
│   (Phone)   │
└──────┬──────┘
       │ WS :9090 (VIDEO BROADCAST)
       │
       ├───────────────────┬────────────────────┐
       │                   │                    │
       ▼                   ▼                    ▼
┌──────────┐        ┌──────────┐        ┌──────────┐
│   Orin   │        │CamViewer │        │ Browser  │
│ (Client1)│        │(Client2) │        │(Client3) │
└─────┬────┘        └─────┬────┘        └──────────┘
      │                   │
      │ Decode            │ Decode & Display
      │ Publish           │ User draws bbox
      │ /recomo/rgb       │
      │                   │ HTTP POST :8080
      │                   └──────────┐
      │                              │
      ▼                              ▼
┌─────────────┐          ┌─────────────────┐
│ ROS2 Topic  │          │ Orin Target API │
│ /recomo/rgb │          │  (NEW Server)   │
└─────────────┘          └────────┬────────┘
                                  │
                                  ▼
                         ┌─────────────────┐
                         │  ROS2 Topic     │
                         │  /target_roi    │
                         └─────────────────┘
```

## Network Ports Summary

| Component | Port | Protocol | Purpose | Direction |
|-----------|------|----------|---------|-----------|
| CamControl | 9090 | WebSocket + HTTP | Video broadcast + camera control | Server (all clients connect) |
| Orin | 8080 | HTTP/WebSocket | Target selection API | Server (CamViewer connects) |
| Orin | 8081 | HTTP REST | Media retrieval API | Server (CamViewer connects) |
| Orin RTSP | 8554 | RTSP | Video restreaming (optional) | Server |
| ROS2 | N/A | DDS | Internal ROS2 communication | Pub/Sub |

## Complete Protocol & Connection Matrix

### Connection 1: CamControl → All Clients (Video Broadcasting)
```
Purpose: Live video streaming + telemetry broadcast
Protocol: WebSocket (binary frames + JSON text frames)
Port: 9090
Direction: CamControl (SERVER) → Clients (Orin, CamViewer, Browser)
Endpoint: ws://<camcontrol-ip>:9090/control

Binary Frames (Video):
- Format: H.264 or H.265 Annex-B NAL units
- FPS: 30 (configurable 15-60)
- Bitrate: 8 Mbps (configurable 1-20 Mbps)
- Resolution: 1920x1080 (configurable 720p-4K)

Text Frames (Telemetry - Broadcast to all):
{
  "type": "telemetry",
  "af": "FOCUSED_LOCKED",
  "ae": "CONVERGED",
  "iso": 250,
  "expNs": 16666667,
  "zoom": 1.0,
  "fps": 29.8,
  "camera": "back",
  "codec": "h264"
}

Clients:
- Orin: GStreamer decode → ROS2
- CamViewer: MediaCodec decode → Display
- Browser: WebCodecs decode → Canvas
```

### Connection 2: Clients → CamControl (Camera Control)
```
Purpose: Camera control commands
Protocol: WebSocket JSON text frames
Port: 9090 (same connection as video)
Direction: Clients → CamControl (RECEIVER)

Commands:
{"cmd": "setZoomRatio", "value": 2.5}
{"cmd": "switchCamera", "facing": "back"}
{"cmd": "setVideoProfile", "width": 1920, "height": 1080, "fps": 30}
{"cmd": "setBitrate", "bitrate": 8000000}
{"cmd": "setCodec", "codec": "h265"}

Senders:
- ✅ Browser (WebUI) - PRIMARY for development/testing
- ✅ CamViewer (dev mode) - Complementary for field use
- ✅ Orin (automation) - Optional scripted control
```

### Connection 3: CamViewer → Orin (Target Selection)
```
Purpose: Send target ROI coordinates
Protocol: HTTP POST or WebSocket
Port: 8080
Direction: CamViewer → Orin (DIRECT, bypasses CamControl)
Endpoint: POST http://<orin-ip>:8080/api/target

Request:
{
  "type": "roi_selection",
  "timestamp": 1699776000000,
  "bbox": {"x": 420, "y": 360, "width": 200, "height": 300},
  "frame_resolution": {"width": 1920, "height": 1080},
  "normalized": {"x": 0.21875, "y": 0.3333, "width": 0.1042, "height": 0.2778}
}

Response:
{"status": "accepted", "target_id": "uuid-1234"}

Processing: Orin validates → publishes to ROS2 /target_roi
```

### Connection 4: CamViewer → Orin (Media Retrieval - NEW)
```
Purpose: Browse and download recorded videos/images
Protocol: HTTP REST API
Port: 8081
Direction: CamViewer → Orin
Base URL: http://<orin-ip>:8081/api/media

Endpoints:
- GET /list?type={video|image|all} - List media files
- GET /metadata/{id} - Get file metadata
- GET /thumbnail/{id} - Get thumbnail image
- GET /download/{id} - Download file (with Range support)
- DELETE /{id} - Delete file (optional)

Response (list):
{
  "items": [
    {
      "id": "uuid", "type": "video", "filename": "track.mp4",
      "size_bytes": 45678901, "duration_ms": 120000,
      "resolution": "1920x1080", "created_at": "2025-11-11T14:30:00Z",
      "thumbnail_url": "/api/media/thumbnail/uuid",
      "download_url": "/api/media/download/uuid"
    }
  ]
}

Features:
- Thumbnail previews
- Progressive download with resume (Range requests)
- Filter by type, date, tracking session
- Metadata with tracking info
```

### Connection 5: Orin → ROS2 (Internal)
```
Purpose: Publish video frames and target selections
Protocol: DDS (ROS2 native)
Topics:
- /recomo/rgb (sensor_msgs/Image) - Video frames from CamControl
- /target_roi (TargetROI) - Target coordinates from CamViewer

Publisher: ws_to_image.py, target_tracker_node.py
```

## Conflict Avoidance

### Package Separation
- **CamControl**: `com.example.camcontrol`
- **CamViewer**: `com.example.camviewer`
- No shared package names, no runtime conflicts

### Port Separation
- CamControl uses port 9090 (unchanged)
- Orin uses port 8080 for new target API
- No port conflicts

### Build System
- Both apps in same Gradle project but separate modules
- Shared utility code via separate module (optional)
- Independent build/deploy

## Deployment Scenarios

### Scenario 1: Lab Testing
```
Device 1 (CamControl): Connect to WiFi, run CamControl app
Device 2 (CamViewer): Connect to same WiFi, connect to Device 1 IP
Orin: Connect to same network, run video processing + tracking
```

### Scenario 2: Field Deployment
```
CamControl: Worn by person/attached to vehicle
CamViewer: Hand-held by operator
Orin: Local edge server or vehicle-mounted
Network: Private WiFi hotspot or 5G
```

### Scenario 3: Development
```
CamControl: Physical device via USB (adb forward)
CamViewer: Emulator or second device
Orin: Development machine or actual Jetson
```

## Security Considerations

### Authentication (Future)
- Add API key or token-based auth for CamControl WebSocket
- Mutual TLS for production deployments
- Orin target API should require authentication

### Network Security
- Use private networks (VPN, local WiFi)
- Consider HTTPS/WSS for production
- Rate limiting on Orin API

### Privacy
- Video stream should not be exposed to public internet
- Target selection data should be ephemeral
- Consider adding video anonymization option

## Performance Targets

| Metric                    | Target      | Notes                          |
|---------------------------|-------------|--------------------------------|
| Video latency (E2E)       | < 200ms     | CamControl → CamViewer        |
| Frame rate                | 30 fps      | Configurable                   |
| Target selection latency  | < 100ms     | CamViewer → Orin              |
| Tracking update rate      | 10-30 Hz    | Depending on algorithm        |
| Video resolution          | 1080p       | Configurable (720p-4K)        |

## Future Enhancements

### Phase 1 (Current)
- ✅ CamControl video streaming
- ✅ Web UI control
- ✅ Orin video ingest
- ⏳ CamViewer app (in progress)
- ⏳ Target selection protocol (in progress)

### Phase 2
- Multi-target tracking
- Target persistence across frames
- Re-identification after occlusion
- Audio streaming integration

### Phase 3
- Multiple CamControl sources
- CamViewer split-screen (multiple cameras)
- Cloud backend option
- Recorded video playback in CamViewer

### Phase 4
- AI-assisted target selection
- Automatic person detection
- Gesture-based controls
- AR overlay on CamViewer

## References

- [DIARY.md](DIARY.md) — Project development diary
- [PROJECT_STATUS_SUMMARY.md](PROJECT_STATUS_SUMMARY.md) — Current status
- [orin/ARCHITECTURE.md](../orin/ARCHITECTURE.md) — Orin-specific details
- [orin/CAMERA_CONTROL_USAGE.md](../orin/CAMERA_CONTROL_USAGE.md) — Camera control guide
