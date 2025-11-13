# System Architecture Diagrams

## System Overview

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                        COMPLETE SYSTEM ARCHITECTURE                          │
└─────────────────────────────────────────────────────────────────────────────┘

                    ┌──────────────────┐
                    │   Mac Browser    │
                    │   (Web Client)   │
                    │                  │
                    │  • WebCodecs     │
                    │  • Controls      │
                    │  • Telemetry     │
                    └────────┬─────────┘
                             │ HTTP/WS :9090
                             │
                             ▼
        ┌────────────────────────────────────────┐
        │      Android CamControl (Server)       │
        │      Package: com.example.camcontrol   │
        │                                        │
        │  ┌──────────────────────────────┐     │
        │  │  Camera2 Pipeline            │     │
        │  │  • Camera access             │     │
        │  │  • Preview + Encoder surface │     │
        │  └──────────────────────────────┘     │
        │               │                        │
        │               ▼                        │
        │  ┌──────────────────────────────┐     │
        │  │  MediaCodec Encoder          │     │
        │  │  • H.264/H.265 encoding      │     │
        │  │  • Annex-B format            │     │
        │  └──────────────────────────────┘     │
        │               │                        │
        │               ▼                        │
        │  ┌──────────────────────────────┐     │
        │  │  Ktor WebSocket Server       │     │
        │  │  • Port 9090                 │     │
        │  │  • Binary frames (video)     │     │
        │  │  • Text frames (telemetry)   │     │
        │  │  • Commands (JSON)           │     │
        │  └──────────────────────────────┘     │
        └────────────┬───────────────────────────┘
                     │
                     │ WS :9090
                     │
                     ├─────────────────────┐
                     │                     │
                     ▼                     ▼
        ┌─────────────────────┐   ┌──────────────────────┐
        │  Android CamViewer  │   │   Jetson Orin        │
        │  (Client - NEW)     │   │   (Backend)          │
        │  Package:           │   │                      │
        │  com.example.       │   │  ┌────────────────┐  │
        │  camviewer          │   │  │  WS Ingest     │  │
        │                     │   │  │  (Python)      │  │
        │ ┌─────────────────┐ │   │  └────────┬───────┘  │
        │ │ WS Client       │ │   │           │          │
        │ └────────┬────────┘ │   │           ▼          │
        │          │          │   │  ┌────────────────┐  │
        │          ▼          │   │  │  GStreamer     │  │
        │ ┌─────────────────┐ │   │  │  • h264parse   │  │
        │ │ MediaCodec      │ │   │  │  • nvv4l2dec   │  │
        │ │ Decoder         │ │   │  │  • Display     │  │
        │ └────────┬────────┘ │   │  └────────┬───────┘  │
        │          │          │   │           │          │
        │          ▼          │   │           ▼          │
        │ ┌─────────────────┐ │   │  ┌────────────────┐  │
        │ │ SurfaceView     │ │   │  │  ROS2 Node     │  │
        │ │ Display         │ │   │  │  • Image pub   │  │
        │ └────────┬────────┘ │   │  │  • Target sub  │  │
        │          │          │   │  └────────┬───────┘  │
        │          ▼          │   │           │          │
        │ ┌─────────────────┐ │   │           ▼          │
        │ │ BBox Selector   │ │   │  ┌────────────────┐  │
        │ │ (Touch UI)      │ │   │  │  Tracking      │  │
        │ └────────┬────────┘ │   │  │  Algorithm     │  │
        │          │          │   │  └────────────────┘  │
        │          ▼          │   │                      │
        │ ┌─────────────────┐ │   │  ┌────────────────┐  │
        │ │ Orin API Client │─┼───┼─▶│  Target API    │  │
        │ │ (HTTP)          │ │   │  │  (FastAPI)     │  │
        │ └─────────────────┘ │   │  │  Port 8080     │  │
        └─────────────────────┘   │  └────────────────┘  │
                                  └──────────────────────┘
```

## Data Flow: Video Streaming

```
┌──────────────┐    ┌──────────────┐    ┌──────────────┐
│  CamControl  │    │  CamViewer   │    │     Orin     │
└──────┬───────┘    └──────┬───────┘    └──────┬───────┘
       │                   │                   │
       │ 1. Camera Capture │                   │
       │◄──────────────────┤                   │
       │                   │                   │
       │ 2. Encode H.264   │                   │
       │───────────────────┤                   │
       │                   │                   │
       │ 3. WS Binary Frame│                   │
       ├──────────────────▶│                   │
       │                   │                   │
       │                   │ 4. Decode         │
       │                   │───────────────────┤
       │                   │                   │
       │                   │ 5. Display        │
       │                   │◄──────────────────┤
       │                   │                   │
       │ 3. WS Binary Frame│                   │
       ├───────────────────┼──────────────────▶│
       │                   │                   │
       │                   │                   │ 6. Decode
       │                   │                   │────────▶
       │                   │                   │
       │                   │                   │ 7. ROS2
       │                   │                   │────────▶
       │                   │                   │
```

## Data Flow: Target Selection

```
┌──────────────┐    ┌──────────────┐    ┌──────────────┐
│  CamControl  │    │  CamViewer   │    │     Orin     │
└──────┬───────┘    └──────┬───────┘    └──────┬───────┘
       │                   │                   │
       │ Video Stream      │                   │
       ├──────────────────▶│                   │
       │                   │                   │
       │                   │ 1. User draws     │
       │                   │    bounding box   │
       │                   │◄──────────────────┤
       │                   │                   │
       │                   │ 2. Convert coords │
       │                   │    (screen→frame) │
       │                   │───────────────────┤
       │                   │                   │
       │                   │ 3. POST /target   │
       │                   │   {bbox, res, ts} │
       │                   ├──────────────────▶│
       │                   │                   │
       │                   │                   │ 4. Validate
       │                   │                   │────────▶
       │                   │                   │
       │                   │                   │ 5. Publish
       │                   │                   │ to ROS2
       │                   │                   │ /target_roi
       │                   │                   │────────▶
       │                   │                   │
       │                   │ 6. OK response    │
       │                   │◄──────────────────┤
       │                   │                   │
       │                   │ 7. Show feedback  │
       │                   │◄──────────────────┤
       │                   │                   │
```

## Component Interaction: Developer Mode

```
┌──────────────┐    ┌──────────────┐
│  CamControl  │    │  CamViewer   │
│              │    │  (Dev Mode)  │
└──────┬───────┘    └──────┬───────┘
       │                   │
       │                   │ 1. User adjusts
       │                   │    zoom slider
       │                   │◄──────────────
       │                   │
       │ 2. setZoomRatio   │
       │◄──────────────────┤
       │    {value: 2.5}   │
       │                   │
       │ 3. Apply zoom     │
       │───────────────────┤
       │                   │
       │ 4. Telemetry      │
       │    {zoom: 2.5,    │
       │     fps: 30, ...} │
       ├──────────────────▶│
       │                   │
       │                   │ 5. Update UI
       │                   │────────────▶
       │                   │
```

## Network Topology

```
        WiFi Network (e.g., 192.168.1.0/24)
        ┌──────────────────────────────────┐
        │                                  │
        │                                  │
   ┌────┴────┐         ┌────┴────┐   ┌────┴────┐
   │ Control │         │ Viewer  │   │  Orin   │
   │ Phone   │         │ Device  │   │ Jetson  │
   │         │         │         │   │         │
   │ :9090   │◄────────┤ Client  │   │ :8080   │
   │         │    WS   │         │   │         │
   │         │         │         ├──▶│ Target  │
   │         │         │         │HTTP│   API   │
   │         │◄────────┼─────────┼───┤         │
   │         │         │         │   │ :8554   │
   │         │         │         │   │  RTSP   │
   └─────────┘         └─────────┘   └─────────┘
   .100                .101           .200
```

## Module Dependencies

```
CamControl (Server)
├── Camera2 API (Android)
├── MediaCodec (Android)
├── Ktor Server
│   ├── WebSocket support
│   └── HTTP server
├── Kotlinx Coroutines
└── Kotlinx Serialization

CamViewer (Client)
├── OkHttp (WebSocket client)
├── MediaCodec (Android)
├── SurfaceView (Android)
├── Ktor Client
│   └── HTTP client
├── Kotlinx Coroutines
├── Kotlinx Serialization
└── Material Design 3

Orin Backend
├── Python 3.8+
├── FastAPI (REST API)
├── Uvicorn (ASGI server)
├── ROS2 Humble
├── GStreamer
│   ├── gst-python
│   └── nvv4l2decoder
└── websockets (Python)
```

## File Structure

```
camControl/
│
├── app/                              # CamControl module
│   ├── build.gradle.kts
│   └── src/main/
│       ├── AndroidManifest.xml
│       ├── java/com/example/camcontrol/
│       │   ├── MainActivity.kt
│       │   ├── CamControlService.kt
│       │   ├── camera/
│       │   │   └── Camera2Controller.kt
│       │   ├── encode/
│       │   │   ├── VideoEncoder.kt
│       │   │   └── VideoRecorder.kt
│       │   └── transport/
│       │       ├── ControlServer.kt
│       │       ├── ControlCommand.kt
│       │       └── Telemetry.kt
│       └── assets/
│           └── index.html
│
├── camviewer/                        # CamViewer module (NEW)
│   ├── build.gradle.kts
│   └── src/main/
│       ├── AndroidManifest.xml
│       ├── java/com/example/camviewer/
│       │   ├── MainActivity.kt
│       │   ├── network/
│       │   │   ├── CamControlClient.kt
│       │   │   ├── OrinClient.kt
│       │   │   └── ProtocolModels.kt
│       │   ├── video/
│       │   │   ├── VideoDecoder.kt
│       │   │   ├── VideoRenderer.kt
│       │   │   └── FrameProcessor.kt
│       │   ├── selection/
│       │   │   ├── BoundingBoxSelector.kt
│       │   │   ├── CoordinateConverter.kt
│       │   │   └── TargetTracker.kt
│       │   └── ui/
│       │       ├── VideoDisplayFragment.kt
│       │       ├── DeveloperModeFragment.kt
│       │       ├── TargetSelectionView.kt
│       │       └── TelemetryView.kt
│       └── res/
│           ├── layout/
│           │   ├── activity_main.xml
│           │   ├── fragment_video_display.xml
│           │   └── fragment_developer_mode.xml
│           └── values/
│               ├── strings.xml
│               ├── colors.xml
│               └── themes.xml
│
├── orin/                             # Orin backend
│   ├── ws_h264_gst.py
│   ├── ws_h264_rtsp_server.py
│   ├── target_api_server.py          # NEW
│   ├── ros2_camcontrol/
│   │   ├── package.xml
│   │   ├── CMakeLists.txt
│   │   ├── msg/
│   │   │   ├── TargetROI.msg         # NEW
│   │   │   └── TrackingResult.msg    # NEW
│   │   └── ros2_camcontrol/
│   │       ├── ws_to_image.py
│   │       ├── target_tracker_node.py # NEW
│   │       └── tracking_visualizer.py # NEW
│   └── tracking/                     # NEW
│       ├── __init__.py
│       ├── tracker_interface.py
│       └── simple_tracker.py
│
├── docs/
│   ├── SYSTEM_ARCHITECTURE.md        # NEW - This file
│   ├── CAMVIEWER_IMPLEMENTATION_PLAN.md # NEW
│   ├── CAMVIEWER_SUMMARY.md          # NEW
│   ├── ARCHITECTURE_DIAGRAMS.md      # NEW - Current file
│   ├── DIARY.md
│   └── PROJECT_STATUS_SUMMARY.md
│
├── scripts/
├── tests/
├── build.gradle.kts
└── settings.gradle.kts               # UPDATED
```

## State Machine: Connection Management

```
        ┌─────────────┐
        │ DISCONNECTED│
        └──────┬──────┘
               │ connect()
               ▼
        ┌─────────────┐
        │ CONNECTING  │
        └──────┬──────┘
               │ success
               ▼
        ┌─────────────┐    network error    ┌─────────────┐
        │  CONNECTED  ├───────────────────▶ │ RECONNECTING│
        └──────┬──────┘                     └──────┬──────┘
               │                                   │
               │ disconnect()      retry           │
               │                  ◄────────────────┘
               │                  fail after N tries
               ▼                         │
        ┌─────────────┐                  │
        │ DISCONNECTED│◄─────────────────┘
        └─────────────┘
```

## Sequence: End-to-End Target Selection

```
User        CamViewer       CamControl      Orin          ROS2
 │              │               │            │             │
 │ 1. Watch     │               │            │             │
 │   video      │               │            │             │
 ├─────────────▶│               │            │             │
 │              │ 2. Stream     │            │             │
 │              │◄──────────────┤            │             │
 │              │               │            │             │
 │ 3. Draw bbox │               │            │             │
 ├─────────────▶│               │            │             │
 │              │               │            │             │
 │              │ 4. Convert    │            │             │
 │              │   coords      │            │             │
 │              │───────────────┤            │             │
 │              │               │            │             │
 │              │ 5. POST       │            │             │
 │              │   /target     │            │             │
 │              ├───────────────┼───────────▶│             │
 │              │               │            │             │
 │              │               │            │ 6. Publish  │
 │              │               │            │   /target_roi
 │              │               │            ├────────────▶│
 │              │               │            │             │
 │              │               │            │             │ 7. Track
 │              │               │            │             │───────▶
 │              │               │            │             │
 │              │ 8. OK         │            │             │
 │              │◄──────────────┼────────────┤             │
 │              │               │            │             │
 │ 9. Feedback  │               │            │             │
 │◄─────────────┤               │            │             │
 │              │               │            │             │
```

---

**Document**: Architecture Diagrams  
**Version**: 1.0  
**Last Updated**: November 11, 2025
