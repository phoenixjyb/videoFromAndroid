# CamControl — Remote Android Camera Control System

English | [中文](README_zh.md)

## Overview

Remote-control Android phone camera with real-time H.265 video streaming and three-way control: **WebUI**, **CamViewer tablet app**, and **ROS2 topics** from Jetson Orin.

**Status:** ✅ **Fully operational** - Three-way camera control working, H.265 streaming, ROS2 integration complete.

## System Components

- **CamControl** (Phone App): Camera source with H.265 encoder, WebSocket server on port 9090
- **CamViewer** (Tablet App): Video viewer with developer mode for camera control
- **Web UI** (Browser): Web-based viewer and control interface  
- **Orin ROS2 Relay**: Bridges ROS2 topics to camera control commands


## Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                  CamControl (Phone)                             │
│                  � Camera Source                                │
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
┌────────┐ ┌──────┐ ┌─────────┐     ┌──────────────┐
│WebUI   │ │Tablet│ │ Orin    │     │ ROS2 Topics  │
│Browser │ │App   │ │ Ingest  │     │              │
│        │ │      │ │         │     │ /recomo/film/zoom│
│Control │ │Video │ │ROS2 Pub │◄────┤ /camera/ae   │
│+ View  │ │View  │ │         │     │ /camera/awb  │
│        │ │Control│ │         │     │ /camera/...  │
└────────┘ └──────┘ └─────────┘     └──────────────┘
                                             ▲
                                             │
                                     camera_control_relay.py
```

**Three-Way Camera Control:**
1. **WebUI** (`http://phone-ip:9090/`) - Browser-based viewer with controls
2. **CamViewer Developer Mode** - Tablet app UI (zoom, camera switch, bitrate, codec)
3. **ROS2 Topics** - Publish to `/camera/*` topics on Orin, relay forwards to phone

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
- Note the IP address shown (e.g., `172.16.30.28`)
- Keep app in foreground

### 3. Connect Clients

**WebUI (Browser):**
```
http://<phone-ip>:9090/
```

**CamViewer (Tablet):**
- Settings → Enter phone IP address
- Toggle Developer Mode ON for camera controls

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
{"cmd":"switchCamera","facing":"back"}      # "back" or "front"  
{"cmd":"setAeLock","value":true}
{"cmd":"setAwbLock","value":false}
{"cmd":"setBitrate","bitrate":5000000}      # bits per second
{"cmd":"setCodec","codec":"h265"}           # "h264" or "h265"
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
