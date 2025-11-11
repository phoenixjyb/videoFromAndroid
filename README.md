# CamControl — Android Camera Streaming (Android + Mac Web + Orin)

English | [中文](README_zh.md)

## Overview
- Purpose: Remote-control an Android phone camera, preview in a Mac web browser or Android viewer app, and ingest/decode on Jetson Orin for processing or restreaming.
- Transport: One Ktor WebSocket endpoint on Android (`/control`, port `9090`) carries both control/telemetry (text JSON) and video (binary H.264 Annex‑B).
- Status: End‑to‑end preview and control are working; Orin ingest/RTSP/ROS2 scaffolds are in place. **CamViewer** Android client app in development for interactive target selection.

## System Components
- **CamControl** (Android Server): Camera streaming and control server
- **CamViewer** (Android Client): Video viewer with interactive target selection *(in development)*
- **Web UI** (Browser): Web-based viewer and control interface
- **Orin Backend**: Video processing, tracking, and ROS2 integration

For detailed system architecture, see [docs/SYSTEM_ARCHITECTURE.md](docs/SYSTEM_ARCHITECTURE.md).

## Architecture (Runtime)
```
                    ┌──────────────────────────────────────────┐
                    │      CamControl (Phone - SOURCE)        │
                    │      📹 Video Broadcasting                │
                    │   • Camera2 + MediaCodec Encoder         │
                    │   • WebSocket Server :9090               │
                    │   • Broadcasts to ALL clients            │
                    └──────────────┬───────────────────────────┘
                                   │ WS :9090
                    ┌──────────────┴──────────────┐
                    │                             │
                    ▼                             ▼
    ┌───────────────────────┐       ┌───────────────────────┐
    │  Orin (Jetson)        │       │  CamViewer (Tablet)   │
    │  🎯 CLIENT #1          │       │  📱 CLIENT #2          │
    │                       │       │                       │
    │  Video Consumer:      │       │  Video Consumer:      │
    │  • WS Client          │       │  • WS Client          │
    │  • GStreamer Decode   │       │  • MediaCodec Decode  │
    │  • Publish /recomo/rgb│       │  • Display to User    │
    │                       │       │                       │
    │  Target API:          │◄──────┤  Target Selection:    │
    │  • HTTP Server :8080  │       │  • Bounding Box UI    │
    │  • Publish /target_roi│       │  • Send to Orin       │
    └───────────────────────┘       └───────────────────────┘
                                             │
                                             │
                                             ▼
                                    ┌───────────────┐
                                    │  Web Browser  │
                                    │  💻 CLIENT #3  │
                                    │  • WebCodecs  │
                                    │  • Controls   │
                                    └───────────────┘

Key Flows:
• Video: CamControl → (Orin, CamViewer, Browser) [broadcast]
• Control: Any Client → CamControl [commands]
• Target: CamViewer → Orin → ROS2 /target_roi [selection]
```

- CamControl: **Video source** that broadcasts H.264/H.265 via WebSocket to all clients
- Orin: **Video client** (receives from CamControl, publishes to ROS2 `/recomo/rgb`) + **Target API server** (receives selections from CamViewer, publishes to ROS2 `/target_roi`)
- CamViewer: **Video client** (receives from CamControl) + **UI for target selection** (sends to Orin) *(in development)*
- Browser: **Video client** (receives from CamControl via WebCodecs/Broadway)

## Software Structure
```
app/                         # CamControl - Android camera server
  src/main/java/com/example/camcontrol/
    CamControlService.kt     # Foreground service: WS server, encoder, telemetry relay
    MainActivity.kt          # Camera control pipeline (preview + attach encoder surface)
    camera/Camera2Controller.kt
    encode/VideoEncoder.kt   # MediaCodec H.264 (Annex‑B)
    transport/ControlServer.kt# Ktor server (HTTP + WS /control)
    transport/ControlCommand.kt, Telemetry.kt
  src/main/assets/index.html # Web UI (WebCodecs decode + controls)

camviewer/                   # CamViewer - Android viewer client (IN DEVELOPMENT)
  src/main/java/com/example/camviewer/
    MainActivity.kt          # Video viewer with target selection
    network/                 # WebSocket client, Orin API client
    video/                   # MediaCodec decoder, renderer
    selection/               # Bounding box selector, coordinate converter
    ui/                      # Video display, developer mode, overlays

scripts/                     # Production recording & streaming scripts
  record_on_device.py        # On-device MP4 recording (MediaMuxer)
  record_video.py            # Stream capture with WebSocket
  ws_probe.py, ws_save_h264.py, ws_cmd.py, ws_record.py
  test_with_subscriber.sh    # ROS2 integration test
  stream_diagnostics.sh      # Stream health monitoring

tools/                       # Development utilities
  quick_start.sh             # Build, install, forward, start
  quick_logs.sh              # Monitor Android logcat

tests/                       # Test scripts and validation
  test_websocket.py, test_commands.py, test_camera_switch.py
  test_telemetry_ws.py, test_webui_commands.py
  test_recomo_rgb_ros2.sh    # ROS2 integration test
  webcodecs-selftest.html    # Browser WebCodecs validation

orin/                        # Jetson Orin integration
  ws_h264_gst.py             # WS ingest → GStreamer decode/display (NVDEC)
  ws_h264_rtsp_server.py     # WS → RTSP restream (rtsp://<orin-ip>:8554/cam)
  ros2_camcontrol/           # ROS2 Humble pkg (WS → sensor_msgs/Image + camera control)
    ros2_camcontrol/ws_to_image.py         # Main ROS2 node
    ros2_camcontrol/camera_control_test.py # Interactive test tool
    msg/                     # Custom ROS2 message types
  CAMERA_CONTROL_USAGE.md    # Camera control guide
  CAMERA_CONTROL_TEST_RESULTS.md # Test validation report

docs/                        # Project documentation
  DIARY.md                   # Project diary & roadmap
  PROJECT_STATUS_SUMMARY.md  # Current status snapshot
  ProjectSetup.md            # Initial setup guide
  WIFI_ACCESS.md             # Network configuration

device_info/                 # Hardware specifications
  sms9160Capability.txt, sms9280Capability.txt
  systemChart.txt            # System architecture diagram

assets/                      # Images and media
  logoRef.png                # Project logo

README.md, README_zh.md      # Bilingual overview (this file)
```

## Protocols
- WebSocket endpoint: `ws://<phone-ip>:9090/control`
  - Text frames (JSON):
    - `{"cmd":"setZoomRatio","value":2.5}`
    - `{"cmd":"switchCamera","facing":"front"}`
    - `{"cmd":"setAeLock","value":true}` / `setAwbLock`, `startRecording`, `stopRecording`, `setVideoProfile`
  - Text telemetry (JSON): `{af, ae, iso, expNs, zoom, fps}`
  - Binary frames: H.264 Annex‑B (SPS/PPS precede IDR)

## Getting Started
### Android (build + run)
- Connect phone via USB; enable developer mode.
- Build + install: `./gradlew installDebug`
- Start app; set up port forwarding: `adb forward tcp:9090 tcp:9090`

### Mac Web Preview
- Open `http://localhost:9090` in a compatible browser (see Browser Compatibility below).
- If blank initially, refresh once (wait for SPS/PPS keyframe).
- **Browser Compatibility**:
  - ✅ **Safari 16.4+** (macOS 13.3+) — WebCodecs H.264 decode (recommended)
  - ✅ **Chrome 94+** — WebCodecs H.264 decode
  - ⚠️ **Edge** — WebCodecs may not be available; Broadway.js fallback used but may have issues
  - 🔧 **Firefox** — WebCodecs not supported; Broadway.js fallback used
- Troubleshooting:
  - Ensure app in foreground; `adb forward` active.
  - Proxy off/bypassed for localhost.
  - Check logs: `scripts/log.sh --both --forward`
  - Check browser console (F12) for decoder status: should show "WebCodecs" or "Broadway"
  - If decoder shows "None", try Safari or Chrome instead

### Save a local capture (Mac)
- `source .venv/bin/activate && python scripts/ws_save_h264.py --seconds 10 --out capture.h264`
- Remux to MP4: `ffmpeg -y -f h264 -i capture.h264 -c copy capture.mp4`

### Record video with custom quality settings (Mac)

**Recommended: On-Device Recording (MediaMuxer)**

Record video directly on Android with MediaMuxer for accurate timestamps and reliable quality:

```bash
# Quick 5-second recording with defaults (H.265, 1080p@30, 5 Mbps)
python3 scripts/record_on_device.py -d 5

# Custom quality 10-second recording
python3 scripts/record_on_device.py -d 10 -c h265 -b 8000000 --profile 1920x1080@30

# High quality 4K recording with zoom
python3 scripts/record_on_device.py -d 30 -c h265 -b 15000000 --profile 3840x2160@30 -z 2.0
```

**Features**:
- ✅ **Accurate timestamps** - Video duration matches recording time
- ✅ **Auto file retrieval** - Pulls MP4 from device automatically  
- ✅ **Configurable quality** - Codec, bitrate, resolution, fps, zoom
- ✅ **Timestamped filenames** - `YYYYMMDD_HHMMSS_codec_resolution_fps_bitrate_duration.mp4`
- ✅ **Video info display** - Shows duration, fps, frame count

**Parameters**:
- `-d, --duration` — Recording duration in seconds (required)
- `-c, --codec` — h264 or h265 (default: h265)
- `-b, --bitrate` — Bitrate in bps (default: 5000000)
- `--profile` — WIDTHxHEIGHT@FPS (default: 1920x1080@30)
- `-z, --zoom` — Zoom ratio (default: 1.0)
- `-H, --host` — WebSocket host (default: localhost)

**Alternative: WebSocket Streaming** (⚠️ has timestamp compression issues)

```bash
# Using record.sh wrapper (less reliable)
./scripts/record.sh -d 10 -c h264 -b 8000000 -p 1920x1080@30
```

Note: WebSocket streaming recordings have compressed timestamps (5s recording → ~1-2s video). Use `record_on_device.py` for accurate recordings. 

**For more recording options and troubleshooting**, see `scripts/README.md`

### Orin — Display/Decode
- Install deps (Jetson): `python3-gi`, `gir1.2-gstreamer-1.0`, GStreamer plugins, `nvidia-l4t-gstreamer`, `websockets`.
- Run: `python3 orin/ws_h264_gst.py --host <android-ip> --codec h265` (default codec is now HEVC; add `--codec h264` if you revert the phone encoder)

### Orin — RTSP Restream
- `python3 orin/ws_h264_rtsp_server.py --host <android-ip> --codec h265`
- Play: `rtsp://<orin-ip>:8554/cam`

### Orin — ROS2 Image Publisher (Quick Start)
- Prereqs: `android-tools-adb`, ROS2 Humble, GStreamer deps, and the `ros2_camcontrol` package built (`colcon build --symlink-install`).
- With the Android phone connected over USB and the CamControl app streaming, run:
  ```bash
  ./quick_start.sh
  ```
  This script verifies ADB connectivity, forwards `localhost:9100 → device:9090`, sources the ROS2 workspace, and launches `ros2_camcontrol.ws_to_image` targeting `/recomo/rgb` at 10 Hz (HEVC by default).
- To collect a one-shot snapshot of CPU usage, topic rate, and recent logs, use the diagnostics helper:
  ```bash
  ./scripts/stream_diagnostics.sh              # runs quick_start, samples ros2 topic hz, tails logs
  ./scripts/stream_diagnostics.sh --dry-run-publish  # skip publishing to measure pipeline latency only
  ```

- **Performance**: The node publishes 640×480 RGB8 images at **~8.8 Hz** sustained rate with BEST_EFFORT QoS, achieving 7.3× improvement over the original 1920×1080 baseline (~1.2 Hz). The downscaling is performed by hardware-accelerated nvvidconv for minimal CPU overhead.

- **Test with image saver**: To verify actual throughput with a real subscriber that saves images to disk:
  ```bash
  ./scripts/test_with_subscriber.sh 50         # save 50 images and measure rate
  ```
  Images are saved to timestamped folders under `saved_images/run_YYYYMMDD_HHMMSS/` with frame counts and statistics.

- While the node is running you can inspect the ROS graph:
  ```bash
  source /opt/ros/humble/setup.bash
  source /home/nvidia/videoFromAndroid/orin/ros2_camcontrol/install/setup.bash
  ros2 topic hz /recomo/rgb --window 50
  ros2 topic echo /recomo/rgb --once
  ```
  The stream is 640×480 RGB8 images published on `/recomo/rgb` with matching `/recomo/camera_info`. Original phone stream is 1920×1080 but downscaled on Orin for efficient ROS2 transport.

### Orin — ROS2 Image Publisher
- ROS2 Humble: `source /opt/ros/humble/setup.bash`
- Build: `cd orin/ros2_camcontrol && colcon build --symlink-install && source install/setup.bash`
- Run: `ros2 run ros2_camcontrol ws_to_image --host <android-ip> --topic /recomo/rgb --rate 10 --codec h265`
  - Add `--camera-info-file <path>` to load calibration YAML/JSON and publish `camera_info`.
  - Use `--camera-info-topic` or `--frame-id` to match downstream expectations.

## What’s Done
- Unified WS port 9090; single WS carries control/telemetry/video.
- Android: service manages WS + encoder; activity manages camera; explicit broadcast commands.
- Web UI: WebCodecs decode with Annex‑B→AVCC conversion; resets on error; Broadway fallback.
- Verified controls (zoom, camera switch) and telemetry.
- Streaming verified in browser and saved to file; binary frames confirmed.
- Orin: WS ingest with NVDEC decode; RTSP restream; ROS2 image publisher scaffold.

## What’s Next
- Android:
  - Audio capture + MP4 mux; profile‑change handshake; resilience/backoff.
  - Manual exposure/ISO, AF controls; persist settings.
- Orin:
  - `tee` fan‑out: display + appsink (CUDA/DeepStream) + RTSP; reconnect & metrics.
  - NVMM zero‑copy path in ROS2 for CUDA ingestion.
- Web UI:
  - Decoder status (WebCodecs/Broadway), FPS overlay, reconnect button; optional WebRTC/MSE path.

## Troubleshooting
- ADB forward: `adb forward --list` should show `tcp:9090 tcp:9090`.
- Proxy: disable/bypass for `localhost`; env vars `http_proxy` etc must not intercept.
- Logs: `adb logcat -v time -s ControlServer:D CamControlService:D MainActivity:D`.
- WS probe: `python scripts/ws_probe.py` shows binary frames when streaming.

---
For the Chinese version, see [README_zh.md](README_zh.md).
