# Network Protocols & Connections

**Date**: November 11, 2025  
**Status**: Complete Specification

## Overview

This document provides a comprehensive reference for all network protocols and connections used in the CamControl system, including CamControl (phone), CamViewer (tablet), Orin (backend), and Web Browser clients.

## Network Topology

```
┌──────────────────────────────────────────────────────────────┐
│                    WiFi Network 192.168.1.0/24               │
└──────────────────────────────────────────────────────────────┘

        ┌────────────────────────────┐
        │   CamControl (Phone)       │  192.168.1.100
        │   VIDEO SOURCE              │
        │                            │
        │   PORT :9090 (Server)      │
        │   • WebSocket Video        │
        │   • Camera Control         │
        │   • HTTP Web UI            │
        └──────────────┬─────────────┘
                       │
                       │ WebSocket :9090
                       │ (Video + Control)
            ┌──────────┴──────────┬──────────────┐
            │                     │              │
            ▼                     ▼              ▼
    ┌───────────────┐   ┌──────────────────┐  ┌─────────┐
    │ Orin (Jetson) │   │ CamViewer (Tab)  │  │ Browser │
    │ 192.168.1.200 │   │  192.168.1.101   │  │  .50    │
    │               │   │                  │  │         │
    │ CLIENT :9090  │   │  CLIENT :9090    │  │ CLIENT  │
    │ (video)       │   │  (video+control) │  │ :9090   │
    │               │   │                  │  │         │
    │ SERVER :8080  │◄──┤  CLIENT :8080    │  │ WebUI   │
    │ (targets)     │   │  (send targets)  │  │ PRIMARY │
    │               │   │                  │  │ DEV UI  │
    │ SERVER :8081  │◄──┤  CLIENT :8081    │  │         │
    │ (media API)   │   │  (get media)     │  │         │
    └───────┬───────┘   └──────────────────┘  └─────────┘
            │
            │ ROS2 DDS
            ▼
    ┌───────────────┐
    │  ROS2 Topics  │
    │ /recomo/rgb   │
    │ /target_roi   │
    └───────────────┘
```

## Connection Matrix

| Source | Destination | Port | Protocol | Purpose | Data Flow |
|--------|-------------|------|----------|---------|-----------|
| CamControl | All Clients | 9090 | WebSocket + HTTP | Video broadcast + Web UI | Video frames (binary) + Telemetry (JSON) → Clients |
| All Clients | CamControl | 9090 | WebSocket | Camera control | Control commands (JSON) → CamControl |
| CamViewer | Orin | 8080 | HTTP/WebSocket | Target selection | Target coords (JSON) → Orin |
| CamViewer | Orin | 8081 | HTTP | Media retrieval | Media files (binary) ← Orin, Requests (JSON) → Orin |
| Orin | ROS2 | N/A | DDS | Internal | Video frames + Target ROI → ROS2 Topics |
| Orin | Clients (optional) | 8554 | RTSP | Video restream | Video → RTSP clients |

## Detailed Protocol Specifications

### 1. Video Broadcasting (CamControl → All Clients)

**Purpose**: Live video streaming with telemetry  
**Protocol**: WebSocket (RFC 6455)  
**Port**: 9090  
**Direction**: CamControl (Server) → Orin, CamViewer, Browser (Clients)  
**URL**: `ws://<camcontrol-ip>:9090/control`

#### Connection Handshake
```http
GET /control HTTP/1.1
Host: 192.168.1.100:9090
Upgrade: websocket
Connection: Upgrade
Sec-WebSocket-Key: dGhlIHNhbXBsZSBub25jZQ==
Sec-WebSocket-Version: 13
```

#### Binary Frames (Video Data)
```
Frame Type: Binary (opcode 0x02)
Format: H.264 or H.265 Annex-B NAL units
Encoding: Raw bytes, no additional framing
Rate: 30 fps (configurable 15-60 fps)
Bitrate: 8 Mbps (configurable 1-20 Mbps)
Resolution: 1920x1080 (configurable 720p-4K)

NAL Unit Structure:
- Start code: 0x00 0x00 0x00 0x01
- NAL header: 1 byte (H.264) or 2 bytes (H.265)
- Payload: Variable length

Multiple NAL units may be sent in one WebSocket frame for efficiency.
Clients must parse start codes to extract individual NAL units.
```

#### Text Frames (Telemetry)
```json
{
  "type": "telemetry",
  "timestamp": 1699776000000,
  "af": "FOCUSED_LOCKED",      // Auto-focus state
  "ae": "CONVERGED",            // Auto-exposure state
  "iso": 250,                   // ISO value
  "expNs": 16666667,            // Exposure time (nanoseconds)
  "zoom": 1.0,                  // Zoom ratio (1.0-10.0)
  "fps": 29.8,                  // Actual frame rate
  "camera": "back",             // "front" or "back"
  "codec": "h264",              // "h264" or "h265"
  "resolution": "1920x1080",    // Current resolution
  "bitrate": 8000000,           // Current bitrate (bps)
  "orientation": 0              // Sensor orientation (degrees)
}
```

**Frequency**: Telemetry sent every 1 second or on significant changes  
**Broadcast**: Sent to ALL connected clients simultaneously

#### Client Implementations

**Orin (GStreamer)**:
```python
# appsrc → h264parse → nvv4l2decoder → appsink
# Binary frames pushed to appsrc
# Decoded frames published to ROS2 /recomo/rgb
```

**CamViewer (MediaCodec)**:
```kotlin
// WebSocket binary frames → MediaCodec decode queue
// Decoded frames → SurfaceView for display
// Low latency mode, hardware acceleration
```

**Browser (WebCodecs/Broadway)**:
```javascript
// WebSocket binary → VideoDecoder.decode()
// Decoded frames → Canvas rendering
```

---

### 2. Camera Control (Clients → CamControl)

**Purpose**: Remote camera control (zoom, exposure, codec, etc.)  
**Protocol**: WebSocket JSON text frames (same connection as video)  
**Port**: 9090  
**Direction**: Clients → CamControl (Server)  
**URL**: `ws://<camcontrol-ip>:9090/control` (same WebSocket)

#### Command Format
```json
{
  "cmd": "setZoomRatio",
  "value": 2.5,
  "timestamp": 1699776000000
}
```

#### Available Commands

**Zoom Control**:
```json
{
  "cmd": "setZoomRatio",
  "value": 1.0  // Range: 1.0 to max zoom (device dependent, typically 10.0)
}
```

**Camera Switch**:
```json
{
  "cmd": "switchCamera",
  "facing": "front"  // "front" or "back"
}
```

**Video Profile**:
```json
{
  "cmd": "setVideoProfile",
  "width": 1920,
  "height": 1080,
  "fps": 30  // Supported: 15, 24, 30, 60
}
```

**Bitrate**:
```json
{
  "cmd": "setBitrate",
  "bitrate": 8000000  // bits per second (1-20 Mbps typical)
}
```

**Codec**:
```json
{
  "cmd": "setCodec",
  "codec": "h265"  // "h264" or "h265"
}
```

**Exposure**:
```json
{
  "cmd": "setExposure",
  "value": 0  // Range: -2 to +2 EV
}
```

**Focus Mode**:
```json
{
  "cmd": "setFocusMode",
  "mode": "auto"  // "auto", "manual", "continuous"
}
```

**Recording Control**:
```json
{
  "cmd": "startRecording",
  "name": "optional_filename"  // Optional, defaults to timestamp
}

{
  "cmd": "stopRecording"
}
```

#### Response
```json
{
  "type": "ack",
  "cmd": "setZoomRatio",
  "status": "success",
  "timestamp": 1699776000100
}
```

Shortly after: Updated telemetry broadcast to ALL clients

#### Who Can Send Commands

- **Web Browser (PRIMARY)**: WebUI for development and testing
- **CamViewer (dev mode)**: Complementary for field adjustments
- **Orin (automation)**: Optional scripted control

**Note**: WebUI is the PRIMARY developer interface for direct phone interaction and testing.

---

### 3. Target Selection (CamViewer → Orin)

**Purpose**: Send target ROI coordinates for tracking  
**Protocol**: HTTP POST or WebSocket  
**Port**: 8080  
**Direction**: CamViewer → Orin (DIRECT, bypasses CamControl)  
**Base URL**: `http://<orin-ip>:8080/api/target`

#### Option A: HTTP REST API (Recommended)

**Endpoint**: `POST /api/target`

**Request Headers**:
```http
POST /api/target HTTP/1.1
Host: 192.168.1.200:8080
Content-Type: application/json
Content-Length: 287
```

**Request Body**:
```json
{
  "type": "roi_selection",
  "timestamp": 1699776000000,
  "bbox": {
    "x": 420,              // Pixel coordinates in frame
    "y": 360,
    "width": 200,
    "height": 300
  },
  "frame_resolution": {
    "width": 1920,
    "height": 1080
  },
  "normalized": {          // Normalized 0-1 coordinates
    "x": 0.21875,
    "y": 0.3333,
    "width": 0.1042,
    "height": 0.2778
  },
  "confidence": 1.0,       // User-selected = 1.0
  "selection_method": "bbox"  // "bbox" or "long_press"
}
```

**Response (Success)**:
```json
{
  "status": "accepted",
  "target_id": "uuid-1234-5678-90ab-cdef",
  "timestamp": 1699776000100,
  "message": "Target ROI published to /target_roi"
}
```

**Response (Error)**:
```json
{
  "status": "error",
  "error_code": "INVALID_COORDINATES",
  "message": "Bounding box exceeds frame boundaries",
  "timestamp": 1699776000100
}
```

#### Option B: WebSocket (For Bidirectional Tracking)

**Endpoint**: `ws://<orin-ip>:8080/tracking`

**Send (Target Selection)**:
```json
{
  "type": "target_selection",
  "timestamp": 1699776000000,
  "bbox": { /* same as HTTP */ },
  "frame_resolution": { /* same as HTTP */ },
  "normalized": { /* same as HTTP */ },
  "selection_method": "bbox"
}
```

**Receive (Tracking Updates - Optional)**:
```json
{
  "type": "tracking_update",
  "target_id": "uuid-1234",
  "timestamp": 1699776000150,
  "bbox": {
    "x": 430,
    "y": 365,
    "width": 200,
    "height": 300
  },
  "confidence": 0.95,
  "state": "tracking",  // "tracking", "lost", "occluded"
  "velocity": {
    "dx": 10,
    "dy": 5
  }
}
```

#### Orin Processing Flow
1. Receive target ROI from CamViewer
2. Validate coordinates (ensure within frame bounds)
3. Publish to ROS2 `/target_roi` topic
4. (Optional) Send tracking updates back to CamViewer

---

### 4. Media Retrieval (CamViewer → Orin)

**Purpose**: Browse and download recorded videos/images from Orin  
**Protocol**: HTTP REST API  
**Port**: 8081  
**Direction**: CamViewer → Orin  
**Base URL**: `http://<orin-ip>:8081/api/media`

#### Endpoint 1: List Media Files

**Request**:
```http
GET /api/media/list?type=all&limit=50&offset=0&sort=date_desc HTTP/1.1
Host: 192.168.1.200:8081
```

**Query Parameters**:
- `type`: `video` | `image` | `all` (default: `all`)
- `limit`: Number of items per page (default: 50, max: 100)
- `offset`: Pagination offset (default: 0)
- `sort`: `date_desc` | `date_asc` | `size_desc` | `size_asc` | `name` (default: `date_desc`)
- `session`: Filter by tracking session ID (optional)
- `date_from`: ISO 8601 timestamp (optional)
- `date_to`: ISO 8601 timestamp (optional)

**Response**:
```json
{
  "total": 156,
  "limit": 50,
  "offset": 0,
  "items": [
    {
      "id": "uuid-5678-90ab",
      "type": "video",
      "filename": "tracking_2025-11-11_14-30-00.mp4",
      "size_bytes": 45678901,
      "duration_ms": 120000,
      "resolution": "1920x1080",
      "codec": "h264",
      "fps": 30,
      "bitrate": 8000000,
      "created_at": "2025-11-11T14:30:00Z",
      "modified_at": "2025-11-11T14:32:00Z",
      "thumbnail_url": "/api/media/thumbnail/uuid-5678-90ab",
      "download_url": "/api/media/download/uuid-5678-90ab",
      "metadata": {
        "tracking_session": "session-abc-123",
        "target_count": 3,
        "annotations": ["person", "vehicle"],
        "camera": "back",
        "location": {
          "latitude": 37.7749,
          "longitude": -122.4194
        }
      }
    },
    {
      "id": "uuid-9012-cdef",
      "type": "image",
      "filename": "snapshot_2025-11-11_14-35-22.jpg",
      "size_bytes": 2456789,
      "resolution": "1920x1080",
      "created_at": "2025-11-11T14:35:22Z",
      "thumbnail_url": "/api/media/thumbnail/uuid-9012-cdef",
      "download_url": "/api/media/download/uuid-9012-cdef",
      "metadata": {
        "tracking_session": "session-abc-123",
        "target_id": "uuid-target-456"
      }
    }
  ]
}
```

#### Endpoint 2: Get Metadata

**Request**:
```http
GET /api/media/metadata/uuid-5678-90ab HTTP/1.1
Host: 192.168.1.200:8081
```

**Response**: Single item object (same structure as list item)

#### Endpoint 3: Get Thumbnail

**Request**:
```http
GET /api/media/thumbnail/uuid-5678-90ab?size=medium HTTP/1.1
Host: 192.168.1.200:8081
```

**Query Parameters**:
- `size`: `small` (160x90) | `medium` (320x180) | `large` (640x360) (default: `medium`)

**Response**:
```http
HTTP/1.1 200 OK
Content-Type: image/jpeg
Content-Length: 12345
Cache-Control: public, max-age=3600

<binary JPEG data>
```

#### Endpoint 4: Download File

**Request**:
```http
GET /api/media/download/uuid-5678-90ab HTTP/1.1
Host: 192.168.1.200:8081
```

**Response**:
```http
HTTP/1.1 200 OK
Content-Type: video/mp4
Content-Length: 45678901
Content-Disposition: attachment; filename="tracking_2025-11-11_14-30-00.mp4"
Accept-Ranges: bytes

<binary file data>
```

**Resume Support (Range Requests)**:
```http
GET /api/media/download/uuid-5678-90ab HTTP/1.1
Host: 192.168.1.200:8081
Range: bytes=1024-2047
```

Response:
```http
HTTP/1.1 206 Partial Content
Content-Type: video/mp4
Content-Length: 1024
Content-Range: bytes 1024-2047/45678901
Accept-Ranges: bytes

<binary chunk>
```

#### Endpoint 5: Delete File (Optional)

**Request**:
```http
DELETE /api/media/uuid-5678-90ab HTTP/1.1
Host: 192.168.1.200:8081
Authorization: Bearer <token>
```

**Response**:
```json
{
  "status": "deleted",
  "id": "uuid-5678-90ab",
  "filename": "tracking_2025-11-11_14-30-00.mp4",
  "timestamp": 1699776000000
}
```

---

### 5. ROS2 Integration (Orin Internal)

**Purpose**: Publish video frames and target selections to ROS2 ecosystem  
**Protocol**: DDS (Data Distribution Service)  
**Direction**: Orin → ROS2 Topics (Pub/Sub)

#### Topic 1: Video Frames

**Topic Name**: `/recomo/rgb`  
**Message Type**: `sensor_msgs/Image`  
**Frequency**: 30 Hz (matches video stream)  
**Publisher**: `ws_to_image.py` node  
**QoS**: `SENSOR_DATA` (Best Effort, Volatile)

**Message Structure**:
```python
std_msgs/Header header
  uint32 seq
  time stamp
  string frame_id
uint32 height
uint32 width
string encoding  # "rgb8" or "bgr8"
uint8 is_bigendian
uint32 step
uint8[] data
```

#### Topic 2: Target ROI

**Topic Name**: `/target_roi`  
**Message Type**: `TargetROI` (custom message)  
**Frequency**: On-demand (user selections)  
**Publisher**: `target_tracker_node.py`  
**QoS**: `RELIABLE` (Reliable, Transient Local)

**Message Definition** (`TargetROI.msg`):
```
std_msgs/Header header
float32 x          # Normalized x coordinate (0-1)
float32 y          # Normalized y coordinate (0-1)
float32 width      # Normalized width (0-1)
float32 height     # Normalized height (0-1)
int32 frame_width  # Original frame resolution
int32 frame_height
uint64 timestamp_ms
string selection_method  # "bbox" | "long_press"
float32 confidence       # 1.0 for user-selected
```

---

## Security Considerations

### Authentication (Future Enhancement)
- **API Keys**: Add `X-API-Key` header for all Orin endpoints
- **JWT Tokens**: Bearer token authentication for sensitive operations
- **Mutual TLS**: Certificate-based authentication for production

### Network Security
- **Private Networks**: Use isolated WiFi or VPN
- **Firewall Rules**: Restrict ports to known IPs
- **HTTPS/WSS**: Upgrade to secure protocols for production
- **Rate Limiting**: Prevent DoS attacks on Orin APIs

### Data Privacy
- **Video Encryption**: Consider encrypting video streams
- **Anonymization**: Option to blur faces/plates in recordings
- **Access Control**: Role-based permissions for media deletion

---

## Performance Specifications

| Metric | Target | Notes |
|--------|--------|-------|
| Video Latency (E2E) | < 200ms | CamControl → CamViewer |
| Frame Rate | 30 fps | Configurable 15-60 fps |
| Target Selection Latency | < 100ms | CamViewer → Orin |
| Media List API | < 500ms | For 100 items |
| Thumbnail Generation | < 200ms | Medium size |
| Download Speed | > 10 MB/s | Limited by WiFi |
| Video Bitrate | 8 Mbps | Configurable 1-20 Mbps |
| Resolution | 1080p | Configurable 720p-4K |

---

## Error Handling

### WebSocket Errors
- **Connection Failed**: Retry with exponential backoff (1s, 2s, 4s, 8s)
- **Connection Lost**: Attempt reconnection, notify user
- **Invalid Frame**: Log error, skip frame, continue decoding
- **Decoder Error**: Reset decoder, request keyframe

### HTTP Errors
- **400 Bad Request**: Show error message to user, log details
- **404 Not Found**: Resource deleted or moved
- **500 Internal Server Error**: Retry up to 3 times, then fail
- **503 Service Unavailable**: Backoff and retry

### Network Timeouts
- **Connection Timeout**: 10 seconds
- **Read Timeout**: 30 seconds for video, 60 seconds for downloads
- **Write Timeout**: 10 seconds

---

## Testing Tools

### WebSocket Testing
```bash
# Test video stream
wscat -c ws://192.168.1.100:9090/control

# Send camera control
wscat -c ws://192.168.1.100:9090/control
> {"cmd": "setZoomRatio", "value": 2.5}
```

### HTTP API Testing
```bash
# Target selection
curl -X POST http://192.168.1.200:8080/api/target \
  -H "Content-Type: application/json" \
  -d '{"type":"roi_selection","bbox":{"x":100,"y":100,"width":200,"height":200},...}'

# List media
curl http://192.168.1.200:8081/api/media/list?type=video&limit=10

# Download with resume
curl -C - -O http://192.168.1.200:8081/api/media/download/uuid-1234
```

### ROS2 Testing
```bash
# Subscribe to video topic
ros2 topic echo /recomo/rgb

# Subscribe to target ROI
ros2 topic echo /target_roi

# Publish test target
ros2 topic pub /target_roi TargetROI "..."
```

---

**Document Version**: 1.0  
**Last Updated**: November 11, 2025  
**Status**: Complete Specification  
**Next Review**: Before Phase 1 implementation
