# Deployment Checklist for Three-Way Camera Control

## Issue Found ⚠️

**CamViewer and ROS2 relay are connecting to the wrong WebSocket endpoint!**

### Current State

| Component | Connecting To | Status |
|-----------|--------------|--------|
| **WebUI** | `ws://phone:8080/control` | ✅ Correct (works) |
| **CamViewer** | `ws://phone:8080/` | ❌ Wrong endpoint |
| **ROS2 Relay** | `ws://phone:8080/` | ❌ Wrong endpoint |

### Phone's ControlServer Configuration

The phone's `ControlServer.kt` has these WebSocket endpoints:
- **Root endpoint** (`/`): Used for video streaming (binary frames)
- **Control endpoint** (`/control`): Used for camera commands (text/JSON)

Looking at line 94-109 in `ControlServer.kt`:
```kotlin
webSocket("/control") {  // ← Commands MUST go to /control endpoint
    connections += this
    for (frame in incoming) {
        if (frame is Frame.Text) {
            val command = frame.readText()
            onCommandReceived(command)  // ← This parses ControlCommand JSON
        }
    }
}
```

## What Needs to be Updated

### 1. ✅ Phone (camControl app)
**Status**: NO CHANGES NEEDED

The phone app already:
- Has ControlServer listening on port 8080
- Accepts commands at `/control` endpoint
- Parses all camera control commands (SetZoomRatio, SetAeLock, SetAwbLock, SwitchCamera, SetBitrate, SetCodec)
- WebUI works correctly (connects to `/control`)

**Last build**: Commit `8ebc11f` (from 6 days ago)
**Rebuild needed**: ❌ No - already has all required functionality

### 2. ❌ CamViewer (tablet)
**Status**: NEEDS UPDATE

**Problem**: PhoneCameraClient.kt line 35-38 connects to wrong endpoint:
```kotlin
httpClient.webSocket(
    host = phoneHost,
    port = 8080,
    path = "/"  // ❌ Wrong! Should be "/control"
) {
```

**Fix Required**:
```kotlin
httpClient.webSocket(
    host = phoneHost,
    port = 8080,
    path = "/control"  // ✅ Correct endpoint for commands
) {
```

**Current build**: Already installed on device from commit `2852beb`
**Rebuild needed**: ✅ YES - one-line fix required

### 3. ❌ ROS2 on Orin (camera_control_relay.py)
**Status**: NEEDS UPDATE

**Problem**: Line 54 constructs wrong WebSocket URI:
```python
self.ws_uri = f'ws://{phone_host}:{phone_port}/'  # ❌ Missing /control path
```

**Fix Required**:
```python
self.ws_uri = f'ws://{phone_host}:{phone_port}/control'  # ✅ Correct
```

**Current deployment**: File exists in git but NOT deployed to Orin yet
**Deployment needed**: ✅ YES - fix required and needs initial deployment

### 4. ✅ WebUI
**Status**: NO CHANGES NEEDED

The WebUI is served by the phone's camControl app and already connects to the correct endpoint (`/control`). It's part of the phone app's assets.

**Rebuild needed**: ❌ No - already correct

## Deployment Plan

### Step 1: Fix CamViewer
```bash
# On development machine
cd /Users/yanbo/Projects/camControl

# Edit PhoneCameraClient.kt line 37
# Change: path = "/"
# To:     path = "/control"

# Build and install
./gradlew :camviewer:installDebug

# Commit
git add camviewer/src/main/java/com/example/camviewer/network/PhoneCameraClient.kt
git commit -m "fix: connect to correct /control endpoint for camera commands"
git push
```

### Step 2: Fix and Deploy ROS2 Relay
```bash
# On development machine
cd /Users/yanbo/Projects/camControl

# Edit orin/camera_control_relay.py line 54
# Change: self.ws_uri = f'ws://{phone_host}:{phone_port}/'
# To:     self.ws_uri = f'ws://{phone_host}:{phone_port}/control'

# Commit
git add orin/camera_control_relay.py
git commit -m "fix: connect to correct /control endpoint for camera commands"
git push

# Deploy to Orin (SSH or file transfer)
scp orin/camera_control_relay.py orin:/path/to/deployment/
scp orin/requirements.txt orin:/path/to/deployment/
scp orin/CAMERA_CONTROL_RELAY_README.md orin:/path/to/deployment/

# On Orin via SSH
pip install websockets  # If not already installed
```

### Step 3: Test All Three Paths

After fixes deployed:

#### Test 1: WebUI (should still work)
```bash
# Open browser to http://172.16.30.28:8080
# Test zoom, camera switch, bitrate, codec
# ✅ Expected: Works (no changes made)
```

#### Test 2: CamViewer Developer Mode
```bash
# On tablet
# 1. Open CamViewer
# 2. Settings → Enable Developer Mode
# 3. Video screen → Expand developer controls
# 4. Test zoom, AE/AWB, camera, bitrate, codec

# On phone (check logs)
adb logcat | grep "ControlServer\|CamControlService"
# ✅ Expected: Should see "Command received: {\"type\":\"setZoomRatio\",...}"
```

#### Test 3: ROS2 Topics
```bash
# On Orin
python3 camera_control_relay.py --phone-host 172.16.30.28

# In another terminal on Orin
ros2 topic pub --once /camera/zoom std_msgs/Float32 "data: 3.0"
ros2 topic pub --once /camera/ae_lock std_msgs/Bool "data: true"
ros2 topic pub --once /camera/switch std_msgs/String "data: 'front'"

# Check relay logs
# ✅ Expected: "📤 Sending command to phone..."

# On phone (check logs)
adb logcat | grep "ControlServer\|CamControlService"
# ✅ Expected: Should see commands received and applied
```

## Summary

| Component | Current Status | Action Required | Priority |
|-----------|----------------|-----------------|----------|
| Phone camControl | ✅ Ready | None | - |
| WebUI | ✅ Ready | None | - |
| CamViewer | ❌ Wrong endpoint | Fix path = "/control" | 🔴 HIGH |
| ROS2 Relay | ❌ Wrong endpoint | Fix ws_uri path, deploy to Orin | 🔴 HIGH |

## Why This Matters

The phone's ControlServer has TWO different WebSocket endpoints:
1. **`/` (root)**: For video streaming - expects binary frames
2. **`/control`**: For camera commands - expects JSON text frames

Currently, CamViewer and ROS2 relay are trying to send commands to the video streaming endpoint, which won't process them. The phone will accept the connection but ignore the commands.

This is why WebUI works (it connects to `/control`) but the new control paths won't work without this fix.
