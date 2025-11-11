# CamViewer Project Summary

## Quick Overview

**CamViewer** is an Android tablet app with three major functionalities:

### 1. Live Video Viewing
- Connects to CamControl :9090 (like Web Browser)
- Displays H.264/H.265 video stream
- Hardware-accelerated decoding

### 2. Target Selection
- Interactive bounding box or long-press
- Sends coordinates **DIRECTLY to Orin :8080**
- Bypasses CamControl (separate channel)

### 3. Media Retrieval (NEW)
- Browse recorded videos/images from Orin
- Download media files to tablet
- Thumbnail gallery with metadata
- Progressive download with resume

### 4. Optional Developer Mode
- Camera control (complementary to WebUI)
- **Note**: WebUI is PRIMARY development interface
- Developer mode for field adjustments

**Key Insight**: 
- **WebUI = PRIMARY** developer interface for testing
- **CamViewer = OPERATOR** interface with Orin integration
- Developer mode in CamViewer is complementary, not replacement

## Critical Architecture Clarification

### ✅ CORRECT Understanding:

**CamControl = VIDEO SOURCE (broadcasts to multiple clients)**
- Role: Single source that streams video to ALL clients (one-to-many)
- Clients: Orin, CamViewer, Web Browser (all receive the SAME stream)
- Port: 9090 (WebSocket)
- Also receives control commands from any client

**Orin = VIDEO CLIENT #1 + TARGET API SERVER (dual role)**
- Role 1: Receives video from CamControl :9090 → publishes to ROS2 `/recomo/rgb`
- Role 2: Receives target selections from CamViewer :8080 → publishes to ROS2 `/target_roi`
- **Note**: Orin gets video from CamControl, NOT from CamViewer

**CamViewer = VIDEO CLIENT #2 + USER INTERFACE (tablet)**
- Similar to Web Browser client for video viewing
- **Enhancement 1**: Sends target coordinates DIRECTLY to Orin (not via CamControl)
- **Enhancement 2**: Developer mode with camera controls (equivalent to WebUI)

**Web Browser = VIDEO CLIENT #3**
- Connects to CamControl :9090 for video
- WebUI for camera control
- CamViewer ≈ Web Browser + target selection

### ❌ What CamViewer Does NOT Do:
- ❌ Does NOT receive video from Orin (gets video from CamControl)
- ❌ Does NOT send video to Orin (only sends target coordinates)
- ❌ Does NOT re-broadcast or relay video
- ❌ Does NOT process or decode video on Orin's behalf

### 📡 Communication Channels

**Channel 1: Video & Camera Control (CamControl ↔ All Clients)**
```
CamControl :9090 (SERVER)
    ↕ Video + Telemetry + Commands
CamViewer, Orin, Browser (CLIENTS)
```

**Channel 2: Target Selection (CamViewer → Orin DIRECT)**
```
CamViewer → Orin :8080 (BYPASSES CamControl)
    ↓ Target coordinates only
Orin → ROS2 /target_roi
```

## How It Works

```
Video Flow (One-to-Many Broadcast):
┌─────────────┐
│ CamControl  │ (Phone - VIDEO SOURCE)
└──────┬──────┘
       │ Broadcasts H.264 via WS :9090
       │
       ├──────────────┬─────────────┐
       │              │             │
       ▼              ▼             ▼
   ┌──────┐     ┌──────────┐   ┌─────────┐
   │ Orin │     │CamViewer │   │ Browser │
   └──────┘     └──────────┘   └─────────┘
    Client#1      Client#2       Client#3

Target Selection Flow (CamViewer → Orin → ROS2):
┌──────────┐         ┌──────┐         ┌─────────────┐
│CamViewer │─────────▶│ Orin │────────▶│ ROS2 Topic  │
│  (User)  │ bbox    │      │         │ /target_roi │
└──────────┘ coords  └──────┘         └─────────────┘
             :8080

Note: Orin already has video from CamControl!
```

## Key Features

### 1. Video Streaming
- Live H.264/H.265 video from CamControl
- Hardware-accelerated decoding (MediaCodec)
- Low latency (< 200ms target)
- Automatic reconnection

### 2. Target Selection
- **Draw bounding box**: Drag on screen to select region
- **Long-press**: Quick selection with default size
- **Visual feedback**: Highlighted selection box
- **Accurate coordinates**: Screen-to-frame transformation
- **Direct to Orin**: Sends via HTTP :8080 (bypasses CamControl)

### 3. Media Retrieval from Orin (NEW)
- **Browse media**: Recorded videos and images from Orin
- **Thumbnail gallery**: Grid view with previews
- **Filter & sort**: By type, date, tracking session
- **Download**: Progressive download with resume capability
- **Metadata**: Resolution, duration, tracking info, GPS (optional)
- **Local cache**: Store recently viewed media
- **Share**: Share downloaded media with other apps

### 4. Developer Mode (Optional - Complementary to WebUI)
- **Note**: WebUI is the PRIMARY development interface
- Camera control (zoom, resolution, bitrate, codec)
- Real-time telemetry display (AF, AE, ISO, FPS)
- Connection management
- Useful for field adjustments when laptop not available

### 5. Orin Integration
- REST API for target selection (HTTP :8080)
- Media API for browsing/downloading (HTTP :8081)
- ROS2 topic publishing (via Orin backend)
- Optional tracking feedback (future)

## No Conflicts with CamControl

✅ **Separate package names**:
- CamControl: `com.example.camcontrol`
- CamViewer: `com.example.camviewer`

✅ **Different roles**:
- CamControl: Server (camera streaming)
- CamViewer: Client (viewing + interaction)

✅ **Can run simultaneously**:
- Both apps can be installed and running
- No shared resources or databases
- Independent configurations

## Project Structure

```
camControl/                    # Existing project (keep as-is)
├── app/                       # CamControl server (UNCHANGED)
├── camviewer/                 # NEW: CamViewer client module
│   ├── build.gradle.kts
│   └── src/main/
│       ├── AndroidManifest.xml
│       ├── java/com/example/camviewer/
│       │   ├── MainActivity.kt
│       │   ├── network/       # WebSocket + HTTP clients
│       │   ├── video/         # Decoder + renderer
│       │   ├── selection/     # Bbox selector + coordinates
│       │   └── ui/            # Fragments + views
│       └── res/               # Layouts, strings, etc.
├── settings.gradle.kts        # UPDATE: add camviewer module
└── docs/
    ├── SYSTEM_ARCHITECTURE.md           # NEW: Full architecture
    ├── CAMVIEWER_IMPLEMENTATION_PLAN.md # NEW: Step-by-step plan
    └── CAMVIEWER_SUMMARY.md            # NEW: This file
```

## Updated Timeline

**Total**: ~4-5 weeks (20-27 days)

| Phase | Duration | Key Deliverables |
|-------|----------|------------------|
| 1. Project Setup | 2-3 days | Module structure, dependencies |
| 2. Video Streaming | 4-5 days | WebSocket client, video decoder, display |
| 3. Target Selection | 3-4 days | Bbox drawing, coordinate conversion |
| 4. Orin Target API | 2-3 days | Target API, ROS2 integration |
| 5. Media Retrieval | 3-4 days | Media gallery, download manager, Orin API |
| 6. Developer Mode | 3-4 days | Camera control UI, telemetry display |
| 7. Polish & Testing | 3-4 days | UI/UX, error handling, testing |

## What Gets Built

### CamViewer App (Android)
- WebSocket client to CamControl
- HTTP clients to Orin (target + media APIs)
- Video decoder (MediaCodec)
- Touch-based bbox selector
- Media gallery with downloads
- Optional developer control panel
- Settings management
- Error handling

### Orin Server Components (Python)
- Target selection API (FastAPI) - port 8080
- Media retrieval API (FastAPI) - port 8081
- Thumbnail generation (FFmpeg)
- File indexing and metadata
- ROS2 message publisher
- Error handling

### ROS2 Integration
- New message types: `TargetROI.msg`
- Updated tracking node
- Visualization tools

## Next Steps

### Ready to Start? Here's What We'll Do:

1. **Review Documents** (You):
   - Read [SYSTEM_ARCHITECTURE.md](SYSTEM_ARCHITECTURE.md)
   - Review [CAMVIEWER_IMPLEMENTATION_PLAN.md](CAMVIEWER_IMPLEMENTATION_PLAN.md)
   - Approve or suggest changes

2. **Phase 1: Setup** (2-3 days):
   - Update Gradle files
   - Create camviewer module
   - Set up basic project structure
   - Create skeleton MainActivity

3. **Phase 2: Video Streaming** (4-5 days):
   - Implement WebSocket client
   - Build video decoder
   - Create video display UI
   - Test end-to-end streaming

4. **Continue Phases 3-6** (see implementation plan)

## Questions to Consider

Before we start, please think about:

1. **Target Selection Behavior**:
   - Default bbox size for long-press? (e.g., 200x300 pixels)
   - Can user adjust bbox after drawing?
   - Multiple targets or single target at a time?

2. **Developer Mode**:
   - Always visible or hidden by default?
   - Which controls are most important?
   - Should we restrict some controls in production?

3. **Orin Communication**:
   - REST API or WebSocket for target selection?
   - Need bidirectional tracking feedback?
   - Error handling preferences?

4. **UI Preferences**:
   - Material Design 3 style?
   - Dark mode support?
   - Portrait or landscape orientation?

## Resources

- **Architecture**: [docs/SYSTEM_ARCHITECTURE.md](SYSTEM_ARCHITECTURE.md)
- **Implementation Plan**: [docs/CAMVIEWER_IMPLEMENTATION_PLAN.md](CAMVIEWER_IMPLEMENTATION_PLAN.md)
- **Existing Code**: See `app/` for CamControl reference
- **Protocols**: See README.md for WebSocket protocol details

## Approval Checklist

Before starting implementation:
- [ ] Architecture reviewed and approved
- [ ] Implementation plan makes sense
- [ ] Timeline is acceptable
- [ ] No concerns about conflicts with CamControl
- [ ] Questions about target selection answered
- [ ] UI/UX preferences clarified
- [ ] Ready to proceed with Phase 1

---

**Status**: ✅ Documentation Complete - Ready for Review  
**Next Action**: Review docs → Answer questions → Start Phase 1  
**Questions?**: Let me know if anything is unclear!
