# Scripts Directory

This directory contains tools for controlling and recording from the Android camera app.

## 📹 Recording Scripts

### ✅ Recommended: `record_on_device.py`
**On-device recording with MediaMuxer - produces accurate timestamps**

```bash
# Quick 5-second recording with defaults (H.265, 1080p@30, 5 Mbps)
python3 scripts/record_on_device.py -d 5

# Custom quality recording
python3 scripts/record_on_device.py -d 10 -c h265 -b 8000000 --profile 1920x1080@30

# High quality 4K recording with zoom
python3 scripts/record_on_device.py -d 30 -c h265 -b 15000000 --profile 3840x2160@30 -z 2.0
```

**How it works**:
- Sends `startRecording`/`stopRecording` commands to Android app
- Android uses MediaMuxer to write MP4 directly on device with correct timestamps
- Script automatically pulls the file from device after recording
- Displays video information (duration, fps, resolution)

**Features**:
- ✅ **Accurate timestamps** - Duration matches recording time (e.g., 5s recording = ~4.3s video)
- ✅ **Automatic file retrieval** - No manual adb pull needed
- ✅ **Configurable encoder** - Codec, bitrate, resolution, fps, zoom
- ✅ **Timestamped filenames** - `YYYYMMDD_HHMMSS_codec_resolution_fps_bitrate_duration.mp4`
- ✅ **Video info display** - Shows duration, fps, frame count automatically

**Parameters**:
- `-d, --duration` — Recording duration in seconds (required)
- `-c, --codec` — h264 or h265 (default: h265)
- `-b, --bitrate` — Bitrate in bps (default: 5000000)
- `--profile` — WIDTHxHEIGHT@FPS (default: 1920x1080@30)
- `-z, --zoom` — Zoom ratio (default: 1.0)
- `-H, --host` — WebSocket host (default: localhost)
- `-P, --port` — WebSocket port (default: 9090)

### `archive/record_on_device_simple.py`
**Simple on-device recording (no configuration)**

```bash
python3 scripts/archive/record_on_device_simple.py --seconds 10 --name myVideo
```

- **Best for**: Quick recordings with default settings
- **How it works**: Sends startRecording/stopRecording commands
- **Output**: MP4 file on device (manual pull required)
- **Timestamps**: ✅ Accurate (uses MediaMuxer)
- **Location**: Archived under `scripts/archive/`
- **Note**: Simpler version of record_on_device.py without encoder config or auto-pull

---

### ⚠️ Alternative: WebSocket Streaming

**Known Issue**: WebSocket streaming recordings have **compressed timestamps** causing duration mismatch.

#### `record_video.py` + `record.sh`

```bash
python3 scripts/record_video.py -d 10 -c h264 -b 8000000
# or
./scripts/record.sh -d 10 -c h264 -b 8000000
```

- **Best for**: Debugging, testing WebSocket streaming
- **How it works**: Captures H.264/H.265 frames from WebSocket and saves locally
- **Output**: H.264/H.265 file (optionally converted to MP4)
- **Timestamps**: ❌ Compressed (5s recording → ~1-2s video)
- **Issue**: Encoder produces 150 frames at 30fps (should be 5 seconds), but timestamps span only ~1.3 seconds
- **Root Cause**: PTS compressed somewhere in WebSocket streaming path
- **Status**: Investigation deferred. Use `record_on_device.py` for accurate recordings.

#### `ws_save_h264.py`

```bash
python3 scripts/ws_save_h264.py --seconds 10 --out capture.h264
```

- **Best for**: Low-level debugging, raw stream capture
- **Output**: Raw H.264/H.265 file (requires manual conversion)
- **Timestamps**: ❌ Compressed (same issue as record_video.py)

---

## Script Comparison

| Script | Method | Timestamp Accuracy | Use Case |
|--------|--------|-------------------|----------|
| `record_on_device.py` | MediaMuxer (on-device) | ✅ Accurate | **Recommended for all recordings** |
| `archive/record_on_device_simple.py` | MediaMuxer (on-device) | ✅ Accurate | Simple version (no config) |
| `record.sh` / `record_video.py` | WebSocket streaming | ❌ Compressed | Debugging/testing only |
| `ws_save_h264.py` | WebSocket streaming | ❌ Compressed | Raw H.264 capture |

---

## 🎮 Control Scripts

### `ws_cmd.py`
**Send control commands to Android app**

```bash
python3 scripts/ws_cmd.py setCodec h265
python3 scripts/ws_cmd.py setVideoProfile 1920 1080 30
python3 scripts/ws_cmd.py setBitrate 5000000
python3 scripts/ws_cmd.py setZoomRatio 2.0
python3 scripts/ws_cmd.py switchCamera front
```

- **Purpose**: Manual encoder configuration and camera control
- **Use case**: Pre-configure encoder before recording with archive/record_on_device_simple.py

### `ws_probe.py`
**Monitor WebSocket connection and frames**

```bash
python3 scripts/ws_probe.py
```

- **Purpose**: Check WebSocket connectivity, view text/binary frame stats
- **Use case**: Debugging connection issues, verifying stream is active

---

## 🛠️ Development & Testing Scripts

### `dev_cycle.sh`
Quick build-install-launch cycle for Android development

### `log.sh`
View Android logcat with filtering

### `webui.sh`
Launch web UI (port forwarding + open browser)

### `bitrate_sweep.sh`
Test multiple bitrate settings for quality comparison

### `quality_test.sh`
Automated quality testing across multiple configurations

---

## 📚 Documentation

- **README.md** (this file): Complete guide to all scripts, recording methods, and troubleshooting

---

## Quick Reference

| Task | Command |
|------|---------|
| **Record 5s video (best quality)** | `python3 scripts/record_on_device.py -d 5` |
| **Record 4K video** | `python3 scripts/record_on_device.py -d 30 --profile 3840x2160@30 -b 15000000` |
| **Quick recording (no config)** | `python3 scripts/archive/record_on_device_simple.py --seconds 10` |
| **Check connection** | `python3 scripts/ws_probe.py` |
| **Configure encoder** | `python3 scripts/ws_cmd.py setCodec h265` |
| **Change zoom** | `python3 scripts/ws_cmd.py setZoomRatio 2.0` |

---

## Troubleshooting

### Connection Issues

**Connection failed:**
```bash
# Ensure app is running
adb shell am start com.example.camcontrol/.MainActivity

# Set up port forwarding
adb forward tcp:9090 tcp:9090

# Start camera (tap START CAMERA button)
adb shell input tap 540 1800
```

### Recording Issues

**Which recording method to use?**
- ✅ Use `record_on_device.py` for accurate, production-ready recordings
- ⚠️ Avoid `record_video.py`/`record.sh` (timestamp compression issues)

**Video duration is wrong:**
- If using `record_video.py` → Switch to `record_on_device.py`
- MediaMuxer recordings have accurate timestamps
- WebSocket streaming has known PTS compression issue

**File not found after recording:**
- Check device: `adb shell ls /sdcard/Android/data/com.example.camcontrol/files/Movies/`
- The script sanitizes filenames (periods become underscores: `5.0s` → `5_0s`)
- Ensure encoder was configured before recording (script does this automatically)

**Video info not showing:**
- Install ffprobe: `brew install ffmpeg`
- Script will still work without it, just won't show detailed info
