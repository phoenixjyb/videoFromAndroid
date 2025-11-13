Orin Ingest Architecture

Goals
- Ingest Android H.264 stream over WebSocket with minimal latency.
- Decode with Jetson NVDEC and display or fan out to processing/streaming.
- Provide a foundation for future modules (analytics, restreaming, recording).

Components
- `WS Ingest` (this repo): connects to `ws://<android-ip>:9090/control`, separates binary (video) from text (telemetry), and feeds `appsrc`.
- `GStreamer Decode`: `appsrc` → `h264parse` → `nvv4l2decoder` → `nvvidconv` → `nveglglessink` (or headless `fakesink`).
- `Fan‑out (future)`: `tee` decoded frames to
  - `appsink` for CUDA inference (DeepStream/pyCUDA/CV)
  - `nvv4l2h264enc ! rtph264pay` for RTP/RTSP restream
  - `webrtcbin` for WebRTC publishing
  - `splitmuxsink` for local MP4 recording

Implemented Modules
- Display/Decode: `ws_h264_gst.py` (WS → Decode → Display)
- RTSP Restream: `ws_h264_rtsp_server.py` (WS → RTP pay → RTSP server)
- ROS2 Publisher: `ros2_camcontrol/ws_to_image.py` (WS → Decode → appsink → sensor_msgs/Image)

Message Flow
1) Android encodes Annex‑B H.264, broadcasts binary frames over WS; telemetry text frames intermixed.
2) Orin WS client reads frames; binary frames → `appsrc`; text frames ignored or forwarded to a metrics bus (future).
3) GStreamer decodes and renders; optionally tees for processing/restreaming.

Future Extensions
- Control bridge: forward Orin UI input to Android WS (zoom/switch) for all‑native control panel.
- RTSP/WebRTC gateway: provide LAN‑friendly URLs for other consumers.
- Resilience: WS reconnect, jitter buffer in `appsrc` pacing, health metrics.

CUDA Considerations
- For true GPU‑zero‑copy to CUDA, keep frames in NVMM (use `nvv4l2decoder ! nvvidconv ! video/x-raw(memory:NVMM),format=RGBA`) and interop via NvBufSurface/CUDA EGL.
- The current ROS2 node publishes CPU `rgb8`. For CUDA ingestion, replace `appsink` with an NVIDIA sample interop (e.g., `appsink` with `memory:NVMM` and map with PyCUDA or use DeepStream’s `nvdsosd`/apps).
