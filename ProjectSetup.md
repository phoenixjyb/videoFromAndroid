Android Camera2 Remote Camera — Quickstart (macOS → Android → Jetson Orin)

Goal: Build a remotely controllable camera app on Android (Camera2) that previews, encodes, and streams to a Jetson Orin server, with bidirectional control/telemetry from your Mac.

⸻

1) What is Camera2?

Camera2 (android.hardware.camera2) is Android’s low-level camera framework. It exposes full manual control (ISO, shutter, lens focus distance), per‑frame metadata, high‑speed modes, RAW, logical multi‑camera, and 10‑bit/HDR pathways. It’s part of the Android framework—no extra Gradle dependency.

You “get” Camera2 by: creating an Android Studio project and importing android.hardware.camera2.*. That’s it.

⸻

2) Camera2 vs CameraX (when to choose which)

Dimension	CameraX (Jetpack)	Camera2 (Framework)
Abstraction	High-level UseCases (Preview, ImageCapture, Video, Analysis)	Low-level: devices, sessions, requests, surfaces
Time-to-ship	Faster	Slower (more boilerplate)
Cross-device behavior	Smoother defaults	You handle quirks
Controls	Common knobs; deeper via Camera2Interop	All low-level knobs (manual sensor, RAW, vendor tags, physical cameras)
High FPS / special modes	Limited	Best path (constrained high-speed sessions)
Custom encoder / zero‑copy	Less direct	Full control of surface graph

Recommendation: Start with Camera2 for robotics/remote-control needs. If you want higher-level ergonomics, you can wrap Camera2 or selectively use CameraX + Camera2Interop.

⸻

3) Capabilities to leverage on Galaxy S23/S24
	•	Logical multi‑camera (ID 0) + CONTROL_ZOOM_RATIO for smooth lens handovers.
	•	Manual sensor (AE_MODE_OFF, SENSOR_EXPOSURE_TIME, SENSOR_SENSITIVITY, LENS_FOCUS_DISTANCE).
	•	Per‑frame telemetry via TotalCaptureResult (AF/AE state, exposure time, ISO, zoom ratio, FPS).
	•	Constrained high‑speed video (use createConstrainedHighSpeedCaptureSession and device‑reported high‑speed size/FPS pairs).
	•	Stabilization: OIS and video EIS (mind FOV crop and FPS caps when EIS is ON).
	•	10‑bit dynamic range when the encoder, transport, and viewer support it.
	•	Stream Use Case hints (API 33+) to reduce latency/power.

Device: SM‑S9160 (from sms9160Capability.txt)
	•	Back logical Camera ID 0 (physical: 2,5,6), LEVEL_3, LOGICAL_MULTI_CAMERA, MANUAL_SENSOR, HIGH_SPEED_VIDEO, 10‑bit.
	•	Sizes: up to 3840×2160 (16:9), many aspect ratios; stable 1920×1080@30 available.
	•	FPS ranges include exact 24 and 30; high‑speed available but needs constrained session.
	•	Stabilization modes: OIS + video stabilization available; prefer OIS for consistent FOV.

⸻

4) macOS development setup
	•	Android Studio (includes SDK/Platform Tools/ADB)
	•	Optional: scrcpy (instant camera preview via USB/Wi‑Fi), ffmpeg for stream checks
	•	Optional: gRPC/protobuf if you use gRPC control channel

⸻

5) Android project skeleton (modules & responsibilities)
	•	camera/Camera2Controller — manages device → session → repeating requests
	•	encode/VideoEncoder — MediaCodec (H.264/HEVC/AV1) with input Surface
	•	transport/ — WebRTC publisher (LiveKit) or RTSP/SRT publisher (library) or custom WebSocket/gRPC
	•	ui/ — SurfaceView/TextureView preview + control panel (zoom, AE/AF/WB, manual toggle)

Manifest basics

<uses-permission android:name="android.permission.CAMERA"/>
<uses-permission android:name="android.permission.RECORD_AUDIO"/>
<uses-permission android:name="android.permission.INTERNET"/>
<uses-feature android:name="android.hardware.camera.any"/>

Runtime permission: request CAMERA (and RECORD_AUDIO if mic).

⸻

6) Core Camera2 pipeline (conceptual)
	1.	Choose camera: Prefer ID 0 (logical back) on S23/S24; read characteristics.
	2.	Surfaces:
	•	Preview surface (SurfaceView or TextureView)
	•	Encoder input surface (from MediaCodec)
	3.	Session: Create a regular session (or constrained high‑speed for 60/120/240 fps).
	4.	Repeating request (TEMPLATE_RECORD):
	•	Targets: preview + encoder surfaces
	•	AF: CONTINUOUS_VIDEO
	•	FPS: CONTROL_AE_TARGET_FPS_RANGE
	•	Zoom: CONTROL_ZOOM_RATIO (API 30+) or SCALER_CROP_REGION fallback
	•	Stabilization: start with OIS ON, EIS OFF
	5.	Telemetry: In CaptureCallback, read TotalCaptureResult and emit JSON (AF/AE state, ISO, exposure time, zoom, fps) at ~10–15 Hz.
	6.	Controls: Map UI/remote commands to setRepeatingRequest updates (zoom, AE lock, AWB lock, manual exposure, tap‑to‑focus).

⸻

7) Streaming to Jetson Orin (pick one)

A) WebRTC (LiveKit) — sub‑second latency + built‑in DataChannel
	•	Orin: run LiveKit server (Docker or binary). Optional TURN for WAN.
	•	Android: publish video track and open a DataChannel for commands & telemetry.
	•	Mac/Browser: subscribe and control.

B) MediaMTX (RTSP/SRT router) — simplest to stand up; great interoperability
	•	Orin: run MediaMTX (single arm64 binary). It accepts RTSP/RTMP/SRT and can output WebRTC/HLS/RTSP.
	•	Android: publish RTSP or SRT (library) from your encoder or built‑in Camera2 source.
	•	Control: separate path (WebSocket/gRPC) for commands & telemetry.

⸻

8) Control/telemetry channel options
	•	WebRTC DataChannel (if using WebRTC)
	•	WebSocket or gRPC (if using RTSP/SRT)
	•	BLE GATT (control‑only, offline/low‑power)

Payload example (JSON):

{"cmd":"setZoomRatio","value":1.6}
{"telemetry":{"af":2,"ae":2,"iso":400,"expNs":5000000,"zoom":1.60,"fps":30}}


⸻

9) Sensible first‑run defaults
	•	Resolution/FPS: 1920×1080 @ 30 fps
	•	Codec/bitrate: H.264, 8 Mbps, keyframe every 2 s
	•	Stabilization: OIS ON, EIS OFF
	•	Exposure: AE/AF/AWB ON; add tap‑to‑focus + AE/AWB lock buttons
	•	Telemetry: JSON @ 10–15 Hz

⸻

10) High‑speed mode checklist
	•	Read highSpeedVideoSizes and highSpeedVideoFpsRanges from StreamConfigurationMap.
	•	Create constrained high‑speed session and use createHighSpeedRequestList.
	•	Expect restricted sizes (e.g., 1080p or 720p) and tighter stabilizer/encoder limits.

⸻

11) Pitfalls to avoid
	•	Mismatched aspect ratios (preview vs encoder) → hidden scaler & latency.
	•	Enabling EIS when you need max FPS or exact FOV (EIS crops and may cap fps).
	•	Re‑issuing capture requests too frequently—batch parameter changes.
	•	Ignoring error callbacks—on onError/onDisconnected, close then reopen cleanly.

⸻

12) “First video in, first control out” plan
	1.	On Orin, start LiveKit or MediaMTX.
	2.	On Android, run your Camera2 app with preview + encoder.
	3.	Publish to Orin (WebRTC or RTSP/SRT).
	4.	From Mac, view the stream and send JSON commands (zoom/exposure/focus).
	5.	Verify telemetry cadence and stability; then add lens‑crossover smoothing (AE/AWB locks during zoom jumps).

⸻

13) Next steps (you can ask me to generate)
	•	Minimal Android module with Camera2Controller + MediaCodec (1080p30) + built‑in WebSocket server.
	•	LiveKit publisher variant (data channel wired for commands/telemetry).
	•	Orin Docker compose for LiveKit or MediaMTX + helper scripts (systemd).

⸻

14) Focused logging (helper script)

Use the helper to capture only relevant logs from this app:

```
bash scripts/log.sh               # filter by tags (default)
bash scripts/log.sh --pid         # filter by app PID only
bash scripts/log.sh --both        # combine PID and tags
bash scripts/log.sh --reverse     # also set up adb reverse 8080
bash scripts/log.sh --outfile logs/my_run.log
```

Manual alternatives:

```
adb logcat -v time -s MainActivity:V Camera2Controller:V VideoEncoder:V ControlServer:V VideoRecorder:V
PID=$(adb shell pidof -s com.example.camcontrol); adb logcat -v time --pid=$PID
```

Control JSON examples (WebSocket /control):

```
{"cmd":"setZoomRatio","value":2.0}
{"cmd":"setAeLock","value":true}
{"cmd":"setAwbLock","value":true}
{"cmd":"startRecording","name":"test"}
{"cmd":"stopRecording"}
{"cmd":"setVideoProfile","width":1920,"height":1080,"fps":30,"highSpeed":false}
{"cmd":"switchCamera","facing":"front"}
```
