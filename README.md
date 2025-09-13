# CamControl — Android Camera Streaming (Android + Mac Web + Orin)

English | [中文](README_zh.md)

## Overview
- Purpose: Remote-control an Android phone camera, preview in a Mac web browser, and ingest/decode on Jetson Orin for processing or restreaming.
- Transport: One Ktor WebSocket endpoint on Android (`/control`, port `9090`) carries both control/telemetry (text JSON) and video (binary H.264 Annex‑B).
- Status: End‑to‑end preview and control are working; Orin ingest/RTSP/ROS2 scaffolds are in place.

## Architecture (Runtime)
```
Mac (Browser)  ←→  Android CamControl (Ktor WS 9090)  →  Orin (WS ingest)
    │  HTML/JS         └─ Camera2 + Encoder (H.264)         ├─ GStreamer NVDEC display
    │  WebCodecs            ↑  Broadcast commands            ├─ RTSP restream (rtsp://:8554/cam)
    └─ Controls JSON        │  Telemetry JSON                └─ ROS2 node publishes sensor_msgs/Image
```

- Browser: Connects to `ws://<host>:9090/control`, decodes video via WebCodecs (Annex‑B→AVCC) with Broadway fallback.
- Android: Ktor serves the web UI (`/`), WS `/control`, H.264 from MediaCodec; forwards UI commands to Activity; broadcasts telemetry.
- Orin: Native Python tools receive WS frames and feed GStreamer (`appsrc → h264parse → nvv4l2decoder`). Optional RTSP server and ROS2 publisher.

## Software Structure
```
app/                         # Android application module
  src/main/java/com/example/camcontrol/
    CamControlService.kt     # Foreground service: WS server, encoder, telemetry relay
    MainActivity.kt          # Camera control pipeline (preview + attach encoder surface)
    camera/Camera2Controller.kt
    encode/VideoEncoder.kt   # MediaCodec H.264 (Annex‑B)
    transport/ControlServer.kt# Ktor server (HTTP + WS /control)
    transport/ControlCommand.kt, Telemetry.kt
  src/main/assets/index.html # Web UI (WebCodecs decode + controls)

scripts/
  ws_probe.py                # Quick check: text/binary frames
  ws_save_h264.py            # Save WS H.264 to file (waits for SPS/IDR)
  ws_record.py               # Trigger on‑device MP4 recording via WS
  log.sh, webui.sh           # Dev helpers (port 9090)

orin/
  ws_h264_gst.py             # WS ingest → GStreamer decode/display (NVDEC)
  ws_h264_rtsp_server.py     # WS → RTSP restream (rtsp://<orin-ip>:8554/cam)
  ros2_camcontrol/           # ROS2 Humble pkg (WS → sensor_msgs/Image)
  README.md, ARCHITECTURE.md # Orin docs

README.md, README_zh.md      # Bilingual overview
DIARY.md                     # Project diary & roadmap
PROJECT_STATUS_SUMMARY.md    # Snapshot of current status
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
- Open `http://localhost:9090` in Edge/Chrome.
- If blank initially, refresh once (wait for SPS/PPS keyframe).
- Troubleshooting:
  - Ensure app in foreground; `adb forward` active.
  - Proxy off/bypassed for localhost.
  - Check logs: `scripts/log.sh --both --forward`

### Save a local capture (Mac)
- `source .venv/bin/activate && python scripts/ws_save_h264.py --seconds 10 --out capture.h264`
- Remux to MP4: `ffmpeg -y -f h264 -i capture.h264 -c copy capture.mp4`

### Orin — Display/Decode
- Install deps (Jetson): `python3-gi`, `gir1.2-gstreamer-1.0`, GStreamer plugins, `nvidia-l4t-gstreamer`, `websockets`.
- Run: `python3 orin/ws_h264_gst.py --host <android-ip>`

### Orin — RTSP Restream
- `python3 orin/ws_h264_rtsp_server.py --host <android-ip>`
- Play: `rtsp://<orin-ip>:8554/cam`

### Orin — ROS2 Image Publisher
- ROS2 Humble: `source /opt/ros/humble/setup.bash`
- Build: `cd orin/ros2_camcontrol && colcon build --symlink-install && source install/setup.bash`
- Run: `ros2 run ros2_camcontrol ws_to_image --host <android-ip> --topic /camera/image_rgb`

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

