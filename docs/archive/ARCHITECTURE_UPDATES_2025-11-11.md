# Architecture Updates - November 11, 2025

## Summary of Changes

This document summarizes the architecture updates made based on user clarifications regarding WebUI priority and the new media retrieval feature.

## Key Clarifications

### 1. WebUI is PRIMARY Developer Interface

**User Statement**:
> "webUI is always the first interface for developers that allow us directly interact with the phone (the main video shooting device), to test interaction, etc."

**Impact**:
- **WebUI = PRIMARY** tool for development and testing
- **CamViewer developer mode = COMPLEMENTARY** for field use
- Updated documentation to emphasize WebUI priority
- CamViewer is designed as an operator interface, not primary dev tool

**Changes Made**:
- All references to "developer mode" now clarify it's complementary to WebUI
- WebUI noted as PRIMARY in connection diagrams
- CamViewer positioned as field operations tool, not development tool

### 2. New Feature: Media Retrieval from Orin

**User Request**:
> "we need to add a third functionality that allows the user to fetch stuff (recorded videos or images) directly from Orin"

**New Functionality**:
- CamViewer can browse media stored on Orin
- Download videos and images to tablet
- Thumbnail gallery with filtering/sorting
- Progressive download with resume capability
- Metadata display (resolution, duration, tracking info)

**Technical Implementation**:
- New Orin service: Media API server on port 8081
- HTTP REST API with endpoints:
  - `GET /api/media/list` - List files with filtering
  - `GET /api/media/thumbnail/{id}` - Get thumbnail
  - `GET /api/media/download/{id}` - Download file (with Range support)
  - `GET /api/media/metadata/{id}` - Get metadata
  - `DELETE /api/media/{id}` - Delete file (optional)

### 3. Complete Protocol Documentation

**User Request**:
> "let us update the connections between these devices, what protocols are used, for each purpose"

**Created**:
- Comprehensive `NETWORK_PROTOCOLS.md` document
- Detailed specifications for all connections
- Protocol examples with request/response formats
- Security considerations
- Testing tools and commands

## Updated Documents

### 1. SYSTEM_ARCHITECTURE.md
**Changes**:
- Added media retrieval as third CamViewer functionality
- Updated CamViewer network connections (added port 8081)
- Clarified WebUI as PRIMARY developer interface
- Added complete protocol/connection matrix with examples
- Updated module structure with media components
- Enhanced port summary table

### 2. CAMVIEWER_IMPLEMENTATION_PLAN.md
**Changes**:
- Added Phase 5: Media Retrieval (3-4 days)
- Renumbered subsequent phases
- Updated timeline: 20-27 days total (was 17-23 days)
- Added media-related components to file checklist
- Updated project goals to include media retrieval
- Clarified WebUI as primary development interface

### 3. CAMVIEWER_SUMMARY.md
**Changes**:
- Restructured overview to highlight three main functionalities
- Added media retrieval feature description
- Updated comparison table (Web Browser vs CamViewer)
- Added "Primary Use Case" row (Development vs Field Operations)
- Updated key features with media retrieval details
- Updated timeline and deliverables

### 4. NETWORK_PROTOCOLS.md (NEW)
**Content**:
- Complete network topology diagram
- Connection matrix table
- Detailed specifications for 5 connection types:
  1. Video Broadcasting (CamControl → Clients)
  2. Camera Control (Clients → CamControl)
  3. Target Selection (CamViewer → Orin)
  4. Media Retrieval (CamViewer → Orin) - NEW
  5. ROS2 Integration (Orin Internal)
- Protocol examples with full request/response formats
- Security considerations
- Performance specifications
- Error handling guidelines
- Testing tools and commands

## Updated Architecture

### CamViewer Three Core Functions

```
┌─────────────────────────────────────────────┐
│           CamViewer (Tablet)                │
│                                             │
│  Function 1: Video Viewing                  │
│  ↓ Connection: WS :9090 to CamControl       │
│  ↓ Same as Web Browser client               │
│                                             │
│  Function 2: Target Selection               │
│  ↓ Connection: HTTP :8080 to Orin (DIRECT)  │
│  ↓ Sends target coordinates                 │
│                                             │
│  Function 3: Media Retrieval (NEW)          │
│  ↓ Connection: HTTP :8081 to Orin           │
│  ↓ Browse, download recorded media          │
│                                             │
│  Optional: Developer Mode                   │
│  ↓ Connection: WS :9090 to CamControl       │
│  ↓ Camera control (complementary to WebUI)  │
└─────────────────────────────────────────────┘
```

### Updated Port Assignments

| Component | Port | Protocol | Purpose |
|-----------|------|----------|---------|
| CamControl | 9090 | WebSocket + HTTP | Video + camera control + WebUI (PRIMARY) |
| Orin | 8080 | HTTP/WebSocket | Target selection API |
| Orin | 8081 | HTTP REST | **Media retrieval API (NEW)** |
| Orin | 8554 | RTSP | Video restreaming (optional) |

### Updated Data Flows

**Flow 1: Video (CamControl → All Clients)**
- WebSocket :9090
- H.264/H.265 binary frames + JSON telemetry
- One-to-many broadcast

**Flow 2: Camera Control (Clients → CamControl)**
- WebSocket :9090 (same connection as video)
- JSON commands
- **PRIMARY: Web Browser WebUI**
- Complementary: CamViewer dev mode, Orin automation

**Flow 3: Target Selection (CamViewer → Orin)**
- HTTP POST :8080 or WebSocket
- JSON target coordinates
- Direct connection (bypasses CamControl)

**Flow 4: Media Retrieval (CamViewer → Orin) - NEW**
- HTTP GET :8081
- JSON metadata + binary files
- Endpoints: list, thumbnail, download, metadata
- Progressive download with Range support

**Flow 5: ROS2 (Orin Internal)**
- DDS protocol
- Topics: /recomo/rgb, /target_roi
- Video frames + target coordinates

## New Components to Implement

### CamViewer (Android)
**New Files**:
- `network/OrinMediaClient.kt` - HTTP client for media API
- `media/MediaRepository.kt` - Data management
- `media/MediaDownloader.kt` - Download manager with resume
- `media/ThumbnailCache.kt` - Local thumbnail caching
- `ui/MediaGalleryFragment.kt` - Gallery UI
- `ui/MediaDetailFragment.kt` - Detail view
- `res/layout/fragment_media_gallery.xml`
- `res/layout/fragment_media_detail.xml`
- `res/layout/item_media_thumbnail.xml`

### Orin (Python)
**New Files**:
- `orin/media_api_server.py` - FastAPI server for media retrieval
  - List endpoint with filtering/sorting
  - Metadata endpoint
  - Thumbnail generation (using FFmpeg)
  - Download endpoint with Range support
  - Optional delete endpoint
  - File system indexing
  - Thumbnail caching

## Implementation Impact

### Timeline Update
- **Previous**: 17-23 days (3-5 weeks)
- **Updated**: 20-27 days (4-5 weeks)
- **Added**: Phase 5 (Media Retrieval) - 3-4 days

### Phase 5: Media Retrieval Details
**Tasks**:
1. Implement Orin media API server (FastAPI)
2. Implement CamViewer media client (Ktor HTTP)
3. Create media repository and download manager
4. Build media gallery UI (RecyclerView with thumbnails)
5. Build media detail UI (viewer + metadata)
6. Test end-to-end (record, browse, download, resume)

**Deliverables**:
- Working media gallery in CamViewer
- Media API server on Orin with thumbnails
- Download manager with resume capability
- Local thumbnail cache

## Testing Checklist

### Connection Testing
- [ ] Video streaming (CamControl :9090 → CamViewer)
- [ ] Camera control (CamViewer → CamControl :9090) - dev mode
- [ ] Target selection (CamViewer → Orin :8080)
- [ ] **Media list (CamViewer → Orin :8081)**
- [ ] **Media download (CamViewer → Orin :8081)**
- [ ] **Resume download after interruption**
- [ ] ROS2 topics (/recomo/rgb, /target_roi)

### Protocol Testing
- [ ] WebSocket video frames decode correctly
- [ ] Camera commands work (zoom, codec, etc.)
- [ ] Target coordinates accurate
- [ ] **Media API returns correct metadata**
- [ ] **Thumbnails generate and cache**
- [ ] **Range requests work for resume**

### UI Testing
- [ ] Video displays with low latency
- [ ] Bounding box draws correctly
- [ ] **Media gallery scrolls smoothly**
- [ ] **Thumbnails load efficiently**
- [ ] **Download progress shows correctly**
- [ ] Developer mode toggles properly

## Next Steps

1. **Review Updated Architecture** ✅
   - SYSTEM_ARCHITECTURE.md
   - CAMVIEWER_IMPLEMENTATION_PLAN.md
   - CAMVIEWER_SUMMARY.md
   - NETWORK_PROTOCOLS.md

2. **Confirm Priorities**
   - WebUI as PRIMARY dev interface - confirmed ✅
   - Media retrieval requirement - confirmed ✅
   - Protocol documentation - complete ✅

3. **Begin Implementation**
   - Phase 1: Project setup
   - Phase 2: Video streaming
   - Phase 3: Target selection
   - Phase 4: Orin target API
   - Phase 5: Media retrieval (NEW)
   - Phase 6: Developer mode
   - Phase 7: Polish & testing

## Questions for Consideration

### Media Retrieval
1. **Storage Location**: Where on Orin should media be stored?
   - Suggested: `/home/user/recorded_media/` with date-based subfolders
2. **Thumbnail Generation**: Pre-generate or on-demand?
   - Suggested: On-demand with caching
3. **Storage Limits**: Auto-cleanup old files?
   - Suggested: Optional auto-delete after N days
4. **Authentication**: Require login to delete files?
   - Suggested: Yes for delete, no for read
5. **Metadata**: What tracking info to store?
   - Suggested: Session ID, target count, GPS (optional), camera settings

### Developer Mode
1. **Default State**: Enabled or disabled by default?
   - Suggested: Disabled (Normal Mode default)
2. **Access Control**: Hidden setting or visible toggle?
   - Suggested: Visible in settings menu
3. **Feature Subset**: All controls or limited set?
   - Suggested: Essential controls only (zoom, camera, profile)

---

**Document Version**: 1.0  
**Date**: November 11, 2025  
**Status**: Architecture Updated  
**Ready for Implementation**: ✅ Yes
