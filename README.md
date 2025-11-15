# CamControl — Remote Android Camera Control System

English | [中文](README_zh.md)

## Overview

Remote-control Android phone camera with real-time H.265 video streaming and three-way control: **WebUI**, **CamViewer tablet app**, and **ROS2 topics** from Jetson Orin.

**Status:** ✅ **Fully operational** - Three-way camera control working, H.265 streaming, ROS2 integration complete.

## Key Features

- 🎥 **Real-time H.265 (HEVC) video streaming** - High quality, low bandwidth
- 🎛️ **Three-way camera control** - WebUI, Android tablet app, or ROS2 topics
- 📱 **Dual camera support** - Switch between front/back cameras remotely
- 🔍 **Advanced controls** - Zoom (1-10x), bitrate (1-50 Mbps), AE/AWB lock, codec selection
- 🤖 **ROS2 integration** - Full camera control via ROS2 topics on Jetson Orin
- 🎯 **Target tracking** - ROI selection, bounding box visualization, coordinate transforms
- 💾 **Media management** - Browse, download, delete recorded videos via API
- 📹 **On-device recording** - Accurate timestamps, 4K support, auto file retrieval
- 📼 **Local recording** - CamViewer tablet can record live streams to MP4
- 📂 **Dual gallery** - View local recordings and synced videos from Orin
- 🖥️ **Remote service control** - Start/stop Orin services from tablet with PIN protection
- 🌐 **Multi-client streaming** - Broadcast to multiple viewers simultaneously
- 🔧 **Developer mode** - Tablet app with visual camera control overlay

## System Components

- **CamControl** (Phone App): Camera source with H.265 encoder, WebSocket server on port 9090
- **CamViewer** (Tablet App): Video viewer with developer mode for camera control + Orin service management
- **Web UI** (Browser): Web-based viewer and control interface  
- **Orin ROS2 Relay**: Bridges ROS2 topics to camera control commands
- **Orin Service Control API**: REST API for remote service management (port 8083)

## Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                  CamControl (Phone)                             │
│                  📱 Camera Source                                │
│                                                                 │
│  • Camera2 API + MediaCodec H.265 Encoder                       │
│  • WebSocket Server :9090                                       │
│  • Endpoints: / (video+telemetry), /control (commands)          │
│  • Broadcasts video to all connected clients                    │
└────────────┬────────────────────────────────────────────────────┘
             │
             │ ws://phone-ip:9090
             │
    ┌────────┼──────────┬─────────────────────┐
    │        │          │                     │
    ▼        ▼          ▼                     ▼
┌────────┐ ┌──────┐ ┌─────────┐     ┌──────────────────┐
│WebUI   │ │Tablet│ │ Orin    │     │ ROS2 Topics      │
│Browser │ │App   │ │ Ingest  │     │                  │
│        │ │      │ │         │     │ /recomo/film/zoom│
│Control │ │Video │ │ROS2 Pub │◄────┤ /recomo/film/ae  │
│+ View  │ │View  │ │         │     │ /recomo/film/awb │
│        │ │Control│ │         │     │ /recomo/film/... │
└────────┘ └──────┘ └─────────┘     └──────────────────┘
                                             ▲
                                             │
                                     camera_control_relay.py
```

**Three-Way Camera Control:**
1. **WebUI** (`http://phone-ip:9090/`) - Browser-based viewer with controls
2. **CamViewer Developer Mode** - Tablet app UI (zoom, camera switch, bitrate, codec)
3. **ROS2 Topics** - Publish to `/recomo/film/*` topics on Orin, relay forwards to phone

**Video Flow:**
- Phone encodes H.265 → broadcasts to all WebSocket clients
- Clients decode and display (WebCodecs for browser, MediaCodec for Android)

**Command Flow:**  
- Any client → `ws://phone-ip:9090/control` → JSON command → phone camera adjusts
- ROS2: Topic → camera_control_relay.py → WebSocket command → phone

## Quick Start

### 1. Install Apps (Pre-built APKs included)
```bash
# Phone (camera source)
adb -s <phone-serial> install app/build/outputs/apk/debug/app-debug.apk

# Tablet (viewer)
adb -s <tablet-serial> install camviewer/build/outputs/apk/debug/camviewer-debug.apk
```

### 2. Start Camera Server (Phone)
- Launch CamControl app
- Note the IP address shown (depends on network):
  - **ZeroTier**: `192.168.100.156`
  - **T8Space**: `172.16.30.28`
- Keep app in foreground

### 3. Connect Clients

**CamViewer (Tablet):**
- Open Settings → Select Network Preset
- Choose "ZeroTier" or "T8Space"
- Toggle Developer Mode ON for camera controls

**WebUI (Browser):**

ZeroTier network:
```
http://192.168.100.156:9090
```

T8Space network:
```
http://172.16.30.28:9090
```

**Orin Services:**
```bash
# ZeroTier (default)
cd orin && ./start_all_services.sh

# T8Space
cd orin && NETWORK_CONFIG=t8space ./start_all_services.sh
```

**ROS2 (Orin):**
```bash
cd orin/
./setup_camera_relay.sh          # First time only
./start_camera_relay.sh --phone-host <phone-ip>

# Test controls
ros2 topic pub --once /recomo/film/zoom std_msgs/Float32 "data: 3.0"
ros2 topic pub --once /recomo/film/switch std_msgs/String "data: 'front'"
```

## Camera Control Commands

All commands use JSON with `"cmd"` discriminator field:

```json
{"cmd":"setZoomRatio","value":2.5}
{"cmd":"switchCamera","facing":"back"}      // "back" or "front"  
{"cmd":"setAeLock","value":true}
{"cmd":"setAwbLock","value":false}
{"cmd":"setBitrate","bitrate":5000000}      // bits per second
{"cmd":"setCodec","codec":"h265"}           // "h264" or "h265"
{"cmd":"requestKeyFrame"}
```

### ROS2 Control Topics
```bash
/recomo/film/zoom       std_msgs/Float32    # 1.0 - 10.0
/recomo/film/ae_lock    std_msgs/Bool       # Auto exposure lock
/recomo/film/awb_lock   std_msgs/Bool       # Auto white balance lock
/recomo/film/switch     std_msgs/String     # "back" or "front"
/recomo/film/bitrate    std_msgs/Int32      # bits/second
/recomo/film/codec      std_msgs/String     # "h264" or "h265"
/recomo/film/key_frame  std_msgs/Empty      # Request keyframe
```

## Orin Service Management

**Remote Control from Tablet:** Start/stop Orin services (Target API, Media API) from CamViewer app.

### Features
- 🖥️ **Service Status Monitoring** - Real-time view of running services with PIDs
- ▶️ **Start/Stop Controls** - Start All / Stop All buttons with PIN protection
- 🔒 **PIN Protection** - Optional PIN to prevent unauthorized service control
- 📋 **Service Logs** - Expandable log viewer for each service
- 🔄 **Auto-refresh** - Toggle automatic status updates every 5 seconds
- 🌐 **Network Aware** - Automatically detects ZeroTier/T8Space network

### Setup (Orin)

**Initial Setup (runs on boot):**
```bash
cd /home/nvidia/videoFromAndroid/orin
sudo ./setup_recomo_service_control.sh
```

This installs a systemd service that:
- Starts automatically on boot
- Restarts automatically on failure
- Runs on port 8083
- Includes PIN protection (default: 1234)

**Manual Restart (if needed):**
```bash
cd /home/nvidia/videoFromAndroid/orin
./restart_recomo_api.sh
```

### Usage (Tablet)

1. Open CamViewer app
2. Navigate to "Orin" tab (computer icon)
3. View service status:
   - **Target API** (port 8082) - ROS2 target tracking
   - **Media API** (port 8081) - Video file management
4. Click "Start All" or "Stop All" (PIN required if configured)
5. Expand service cards to view logs
6. Toggle auto-refresh for real-time monitoring

### Configuration

**Change PIN (Orin):**
Edit `/etc/systemd/system/recomo_service_control.service`:
```ini
Environment="SERVICE_CONTROL_PIN=your-pin-here"
```
Then restart: `sudo systemctl restart recomo_service_control`

**Change PIN (Tablet):**
Settings → Security → Service Control PIN

**API Endpoints:**
```
GET  http://orin-ip:8083/api/services/status      # Get service status
POST http://orin-ip:8083/api/services/start       # Start all services
POST http://orin-ip:8083/api/services/stop        # Stop all services
GET  http://orin-ip:8083/api/services/logs/{id}   # Get service logs
```

## Project Structure

```
app/                         # CamControl phone app (camera source)
  src/main/java/com/example/camcontrol/
    MainActivity.kt          # Camera2 pipeline
    CamControlService.kt     # WebSocket server + encoder
    encode/VideoEncoder.kt   # MediaCodec H.265 encoder (default)
    transport/ControlServer.kt # Ktor WS server :9090
  src/main/assets/index.html # Built-in WebUI

camviewer/                   # CamViewer tablet app (viewer + controls)
  src/main/java/com/example/camviewer/
    MainActivity.kt          # Video display + dev controls
    network/PhoneCameraClient.kt  # WebSocket client
    video/VideoDecoder.kt    # MediaCodec H.265 decoder (default)
    video/VideoRecorder.kt   # MP4 recording with MediaMuxer
    ui/screens/video/        # Video display + control panel
    ui/screens/media/        # Gallery with two tabs (local/synced)
    ui/screens/orin/         # Orin service control screen
    data/repository/MediaRepository.kt      # Media sync from Orin
    data/repository/OrinServiceRepository.kt # Service control API client
    data/models/ServiceModels.kt            # Service status models

orin/                        # Jetson Orin ROS2 integration
  camera_control_relay.py    # ROS2 topics → WebSocket commands
  service_control_api.py     # REST API for service management
  target_api.py              # ROS2 target tracking API (port 8082)
  media_api.py               # Video file management API (port 8081)
  setup_camera_relay.sh      # One-time setup script
  start_camera_relay.sh      # Launch relay
  start_all_services.sh      # Start Target + Media APIs
  stop_all_services.sh       # Stop all services (port-aware)
  setup_recomo_service_control.sh  # Install systemd service
  restart_recomo_api.sh      # Quick restart helper
  recomo_service_control.service   # systemd service config
  test_camera_control.sh     # Test all controls via ROS2
  *.md                       # Setup guides and documentation

scripts/                     # Recording and testing utilities
  record_on_device.py        # Best: accurate MP4 recording
  record_video.py            # WebSocket-based recording
  ws_probe.py, ws_save_h264.py, ws_cmd.py

docs/                        # Project documentation
  *.md                       # Architecture, setup guides, status
```

## Technical Details

**Video Codec:** H.265 (HEVC) default on all components for better compression
- Phone encoder: MediaCodec with MIME_TYPE_HEVC
- CamViewer decoder: MediaCodec with MIMETYPE_VIDEO_HEVC  
- WebUI: WebCodecs H.265 decode (Chrome/Safari)

**Transport:** WebSocket on port 9090
- `/` endpoint: Video (binary H.265 Annex-B frames) + Telemetry (JSON)
- `/control` endpoint: Command messages (JSON with "cmd" discriminator)

**Command Format:**
```json
{
  "cmd": "setZoomRatio",    // Must use "cmd" not "type"
  "value": 2.5
}
```

**Browser Compatibility:**
- ✅ Safari 16.4+ (macOS 13.3+) — WebCodecs H.265
- ✅ Chrome 94+ — WebCodecs H.265
- ⚠️ Firefox — No WebCodecs, Broadway.js fallback

## Recording Videos

**Recommended: On-Device Recording**
```bash
# High quality 4K H.265 recording
python3 scripts/record_on_device.py -d 30 -c h265 -b 15000000 --profile 3840x2160@30 -z 2.0
```

**Features:**
- ✅ Accurate timestamps (no compression)
- ✅ Auto file retrieval from device
- ✅ Configurable quality settings
- ✅ Direct MediaMuxer recording on phone

## Orin Integration

**ROS2 Camera Control Relay:**
```bash
cd orin/
./setup_camera_relay.sh              # First time setup
./start_camera_relay.sh --phone-host 172.16.30.28
```

**ROS2 Control Topics:**
- `/recomo/film/zoom` (Float32)
- `/recomo/film/ae_lock` (Bool)  
- `/recomo/film/awb_lock` (Bool)
- `/recomo/film/switch` (String: "back"/"front")
- `/recomo/film/bitrate` (Int32)
- `/recomo/film/codec` (String: "h264"/"h265")
- `/recomo/film/key_frame` (Empty)

**Test Controls:**
```bash
./orin/test_camera_control.sh
```

## CamViewer Recording & Gallery

**Recording Live Streams (Tablet):**
- Press the record button while viewing video
- H.265 stream recorded directly to MP4 with MediaMuxer
- Files saved to `/sdcard/Movies/recomoVideosRawStream/`
- Recordings include proper CSD (VPS/SPS/PPS) for H.265 playback

**Gallery with Two Tabs:**

1. **Local Recordings Tab**
   - Shows videos recorded on the tablet
   - Location: `/sdcard/Movies/recomoVideosRawStream/`
   - Direct access from Files app
   - Delete recordings from gallery

2. **Synced from Orin Tab**
   - Shows videos downloaded from Orin server
   - Location: `/sdcard/Movies/syncRecomo/`
   - Browse media from Orin's API
   - Download and manage synced videos

**Storage Locations:**
```bash
# Tablet local recordings
/sdcard/Movies/recomoVideosRawStream/

# Videos synced from Orin
/sdcard/Movies/syncRecomo/
```

## Development History

### Phase 1: Core Streaming Infrastructure
- **Initial WebSocket Architecture** (Port 9090)
  - Camera2 API + MediaCodec H.264 encoder
  - WebSocket server with video broadcast and telemetry
  - Browser-based WebUI with WebCodecs/Broadway.js fallback
  - Bitrate control (1-50 Mbps) and dynamic quality adjustment

### Phase 2: Multi-Codec Support
- **H.265 (HEVC) Implementation**
  - Dual codec support: H.264 and H.265
  - Runtime codec switching via `setCodec` command
  - Profile/level optimization for encoder stability
  - Set H.265 as default for better compression (Nov 2025)

### Phase 3: Three-Way Control System
- **CamViewer Tablet App**
  - Real-time H.265 video playback
  - Developer mode with camera control overlay
  - Zoom, camera switch, bitrate, codec controls
  - ROI/bounding box visualization for target tracking

- **Orin ROS2 Integration**
  - `camera_control_relay.py`: ROS2 topics → WebSocket commands
  - 7 camera control topics (`/recomo/film/zoom`, `/recomo/film/switch`, etc.)
  - Comprehensive test script for all controls
  - JSON discriminator fix: `"type"` → `"cmd"` (Nov 2025)

### Phase 4: Media Management & Target Tracking
- **Media Retrieval API** (Orin)
  - `media_api.py`: Browse/download recorded videos
  - Thumbnail generation and video playback
  - Delete functionality and download management
  - FastAPI server with lifespan handlers

- **Target API** (Orin)
  - `target_api.py`: Target selection and ROI publishing
  - Bounding box support with pixel/normalized coordinates
  - ROS2 `/target/roi` topic for vision pipeline
  - `listen_target_roi.py`: Monitor target updates
  - Camera info synchronization for coordinate transforms

### Phase 5: Recording & Quality Optimization
- **On-Device Recording** (Recommended)
  - `record_on_device.py`: Direct MediaMuxer recording on phone
  - Accurate timestamps and configurable quality
  - Auto file retrieval via ADB
  - Supports 4K H.265 recording at 30fps

- **Quality Testing Tools**
  - Bitrate sweep automation
  - Quality test scripts with multiple profiles
  - Stream diagnostics and encoder optimization
  - AVC High/Main profile preference

### Recent Improvements (November 2025)
- ✅ **H.265 Default**: All components aligned on HEVC codec
- ✅ **JSON Format Fix**: Standardized `"cmd"` discriminator field
- ✅ **Three-Way Control**: WebUI, CamViewer, ROS2 all operational
- ✅ **Pre-built APKs**: Added to repository for easy deployment
- ✅ **Port 9090**: Unified across all components

## Current Status

✅ **Fully Operational:**
- Three-way camera control (WebUI, CamViewer, ROS2)
- H.265 video streaming to multiple clients
- Real-time camera parameter adjustment (zoom, AE/AWB lock, bitrate, codec)
- On-device accurate recording with configurable quality
- ROS2 topic control relay with all 7 commands
- Media retrieval API on Orin (browse/download/delete videos)
- Target selection API with ROI publishing

🚧 **In Development:**
- ROS2 video publisher optimization (`/recomo/rgb` stability)
- Complete vision pipeline integration (target tracking → camera control loop)
- Production deployment packaging

## Troubleshooting

**No video in viewer:**
- Restart CamControl app on phone (resets encoder)
- Check phone IP address matches in client settings
- Verify both devices on same WiFi network

**Commands not working:**
- Check JSON format uses `"cmd"` field (not `"type"`)
- Verify WebSocket connection established
- Check phone logs: `adb logcat | grep CamControlService`

**ROS2 relay issues:**
- Ensure phone host IP is correct
- Check relay logs for connection errors
- Verify ROS2 topics exist: `ros2 topic list`

**H.265 decoder issues:**
- Not all devices support HEVC hardware decode
- Check `adb logcat | grep MediaCodec` for decoder errors
- Fallback: Switch to H.264 via `ros2 topic pub --once /recomo/film/codec std_msgs/String "data: 'h264'"`

## Changelog

### November 2025
- **13 Nov**: Added MP4 recording feature to CamViewer with MediaMuxer
- **13 Nov**: Implemented dual-tab gallery (Local Recordings + Synced from Orin)
- **13 Nov**: Changed storage to public folders (`/sdcard/Movies/recomoVideosRawStream/`, `/sdcard/Movies/syncRecomo/`)
- **13 Nov**: Fixed H.265 keyframe detection (NAL type extraction) for proper CSD
- **13 Nov**: Added pre-built APKs to repository for deployment convenience
- **13 Nov**: Set H.265 (HEVC) as default codec across all components
- **12 Nov**: Fixed JSON discriminator from `"type"` to `"cmd"` for phone compatibility
- **12 Nov**: Added comprehensive camera control test script (`test_camera_control.sh`)
- **11 Nov**: Implemented three-way camera control pipeline (WebUI + CamViewer + ROS2)
- **11 Nov**: Added developer mode camera control overlay in CamViewer

### October 2025
- **Oct**: Phase 4 - Media Retrieval API complete (browse/download/delete videos)
- **Oct**: Added ROS2 target listener and ROI support in CamViewer
- **Oct**: Implemented hybrid tap/drag gesture for target selection
- **Oct**: Added Target API with bounding box support
- **Oct**: Created CamViewer Android app (Phase 2 complete)

### September 2025
- **Sep**: Fixed ROS2 image publisher rate (achieved 8.8 Hz with optimizations)
- **Sep**: Added camera control integration to ROS2 node
- **Sep**: Reorganized project structure for professional layout

### August 2025
- **Aug**: Optional H.265 (HEVC) support with codec switching
- **Aug**: Added bitrate control end-to-end (1-50 Mbps)
- **Aug**: Implemented WebSocket timeout/drop for slow clients
- **Aug**: Added WebUI status strip with decoder type and FPS

### July 2025
- **Jul**: Fixed video streaming with encoder mutex and Broadway fallback
- **Jul**: Safari WebCodecs support, browser compatibility notes
- **Jul**: Added custom app icon
- **Jul**: Orin: RTSP restream and ROS2 Humble image publisher

### June 2025
- **Jun**: Added quality test and automation scripts
- **Jun**: Implemented zoom control and recording defaults
- **Jun**: Enhanced video recording with quality controls

### Initial Release (May 2025)
- **May**: Initial WebSocket architecture on port 9090
- **May**: Camera2 API + MediaCodec H.264 encoder
- **May**: Browser WebUI with WebCodecs/Broadway.js
- **May**: Telemetry relay and control commands

## Contributing

This is a research project for remote camera control and target tracking. For questions or issues, please refer to the documentation in `docs/`.

## License

[Add your license here]

---

For Chinese version, see [README_zh.md](README_zh.md).
