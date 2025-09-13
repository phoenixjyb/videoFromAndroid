Orin Ingest Service — H.264 over WebSocket (Jetson Orin)

Overview
- Purpose: receive the Android app’s H.264 Annex‑B video stream over WebSocket and decode with Jetson hardware (NVDEC) via GStreamer.
- Transport: WebSocket binary frames from `ws://<android-ip>:9090/control` (same endpoint used by the browser preview). Text frames (telemetry) are ignored.
- Decode: `nvv4l2decoder` → `nvvidconv` → `nveglglessink` (fallback to software decode/display if hardware elements aren’t available).

Key Features
- Low‑latency path with GStreamer `appsrc` fed directly by the WS client.
- Auto‑detects Jetson hardware decoder and falls back to software (`avdec_h264`) if missing.
- Configurable sink (e.g., `nveglglessink`, `fakesink`) and host/port.

Directory
- `ws_h264_gst.py` — main receiver/decoder
- `ws_h264_rtsp_server.py` — WS→RTSP restream (rtsp://<orin-ip>:8554/cam)
- `ARCHITECTURE.md` — design and next steps

ROS 2 (Humble) — Image publisher
- Package: `orin/ros2_camcontrol`
- Node: `ws_to_image` subscribes to Android WS stream and publishes `sensor_msgs/Image` (RGB)

Build (on Orin)
```bash
sudo apt-get update
sudo apt-get install -y python3-gi gir1.2-gstreamer-1.0 gstreamer1.0-tools \
  gstreamer1.0-plugins-base gstreamer1.0-plugins-good gstreamer1.0-plugins-bad \
  nvidia-l4t-gstreamer
python3 -m pip install websockets

# RTSP restream
python3 ws_h264_rtsp_server.py --host <android-ip>

# ROS2 image publisher (Humble)
source /opt/ros/humble/setup.bash
cd ros2_camcontrol
colcon build --symlink-install
source install/setup.bash
ros2 run ros2_camcontrol ws_to_image --host <android-ip> --topic /camera/image_rgb
```

Prerequisites (Jetson Orin)
1) System packages (usually preinstalled on Jetson, otherwise install):
   - GStreamer core and plugins: `gstreamer1.0-tools gstreamer1.0-plugins-base gstreamer1.0-plugins-good gstreamer1.0-plugins-bad`
   - Jetson HW accel: `nvidia-l4t-gstreamer` (or equivalent L4T meta-package)
   - Python GI bindings: `python3-gi gir1.2-gstreamer-1.0`

   Example (Ubuntu-based Jetson):
   ```bash
   sudo apt-get update
   sudo apt-get install -y python3-gi gir1.2-gstreamer-1.0 gstreamer1.0-tools \
       gstreamer1.0-plugins-base gstreamer1.0-plugins-good gstreamer1.0-plugins-bad \
       nvidia-l4t-gstreamer
   ```

2) Python deps:
   - websockets: `python3 -m pip install websockets`

Usage
```bash
# On Jetson Orin
cd orin
python3 ws_h264_gst.py --host <android-ip> --port 9090 --sink nveglglessink

# Examples
python3 ws_h264_gst.py --host 192.168.1.50               # defaults to 9090 + nveglglessink
python3 ws_h264_gst.py --host 192.168.1.50 --sink fakesink --no-hw  # headless SW decode
```

Notes
- If connecting via USB/ADB instead of LAN, forward the port on the host that has ADB, and route from Orin to that host.
- The Android server emits Annex‑B samples with SPS/PPS before IDR; pipeline caps are set to `stream-format=byte-stream, alignment=au`.
