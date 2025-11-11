# CamViewer Implementation Plan

## Executive Summary

This document outlines the step-by-step implementation plan for the CamViewer Android application, including architecture decisions, development phases, and integration with the existing CamControl system.

## Project Goals

1. Create a standalone Android viewer app that connects to CamControl
2. Display real-time video stream with low latency
3. Enable interactive target selection via bounding box
4. Send target selections to Orin for tracking
5. **Browse and download recorded media from Orin (videos/images)**
6. Provide optional developer mode for camera control (complementary to WebUI)
7. Maintain compatibility and avoid conflicts with CamControl

**Note**: WebUI (browser) is the PRIMARY developer interface for CamControl testing and development. CamViewer's developer mode is complementary for field use.

## Architecture Decisions

### 1. Project Structure
**Decision**: Multi-module Gradle project

**Rationale**:
- Share common code between CamControl and CamViewer
- Unified build system and dependency management
- Easier version control and CI/CD

**Structure**:
```
camControl/                      # Root project
├── app/                         # CamControl (existing)
├── camviewer/                   # CamViewer (new module)
├── common/                      # Shared code (new module, optional)
├── build.gradle.kts
└── settings.gradle.kts
```

### 2. Video Decoding
**Decision**: MediaCodec with SurfaceView rendering

**Rationale**:
- Hardware acceleration for H.264/H.265
- Low latency (< 100ms additional)
- Native Android support
- Same technology as CamControl encoder

**Alternative Considered**: FFmpeg
- Rejected: More complex, larger binary size, no significant advantage

### 3. Touch Interaction
**Decision**: Custom view overlay with gesture detection

**Rationale**:
- Fine-grained control over touch events
- Separate video rendering from interaction logic
- Easy to add visual feedback (bbox drawing)

### 4. Network Architecture
**Decision**: 
- WebSocket client (OkHttp + WebSocket) for CamControl
- Ktor HTTP client for Orin target API

**Rationale**:
- OkHttp is standard Android networking library
- WebSocket for real-time video streaming
- Ktor for consistency with CamControl server

### 5. Target Selection Protocol
**Decision**: RESTful API with JSON payload

**Rationale**:
- Simple, stateless
- Easy to debug and test
- Can add WebSocket later for bidirectional tracking feedback

**Endpoint**: `POST http://<orin-ip>:8080/target`

## Implementation Phases

### Phase 1: Project Setup & Basic Infrastructure
**Duration**: 2-3 days

**Tasks**:
1. ✅ Create architecture documentation
2. ✅ Create implementation plan (this document)
3. ⏳ Update Gradle build files
   - Add `camviewer` module to `settings.gradle.kts`
   - Create `camviewer/build.gradle.kts`
   - Add dependencies (Ktor client, OkHttp, Coroutines, etc.)
4. ⏳ Create basic project structure
   - Package structure
   - MainActivity skeleton
   - AndroidManifest.xml
5. ⏳ Set up resources
   - Strings, colors, themes
   - Layout files (activity_main.xml)
6. ⏳ Create shared protocol models
   - Move command/telemetry classes to common module (optional)
   - Or duplicate in camviewer package

**Deliverables**:
- Buildable camviewer module
- Basic app structure
- Empty MainActivity launches

**Success Criteria**:
- App builds successfully
- App installs on device
- No conflicts with CamControl app

### Phase 2: WebSocket Client & Video Streaming
**Duration**: 4-5 days

**Tasks**:
1. ⏳ Implement WebSocket client
   - Connection management
   - Message parsing (binary vs text)
   - Reconnection logic
   - Connection state monitoring
2. ⏳ Implement video decoder
   - MediaCodec setup for H.264/H.265
   - Input buffer queue management
   - Output surface configuration
3. ⏳ Implement video renderer
   - SurfaceView setup
   - Aspect ratio handling
   - Fullscreen support
4. ⏳ Wire decoder to WebSocket
   - Feed binary frames to decoder
   - Handle SPS/PPS/IDR frames
   - Error handling and recovery
5. ⏳ Basic UI
   - Video display
   - Connection button
   - Status indicator

**Key Classes**:
```kotlin
// network/CamControlClient.kt
class CamControlClient(
    private val host: String,
    private val port: Int,
    private val onVideoFrame: (ByteArray) -> Unit,
    private val onTelemetry: (Telemetry) -> Unit,
    private val onConnectionChange: (Boolean) -> Unit
) {
    suspend fun connect()
    suspend fun disconnect()
    suspend fun sendCommand(command: ControlCommand)
}

// video/VideoDecoder.kt
class VideoDecoder(
    private val surface: Surface,
    private val codec: String = "h265"
) {
    fun start()
    fun stop()
    fun queueFrame(frame: ByteArray)
}

// video/VideoRenderer.kt
class VideoRenderer(
    private val surfaceView: SurfaceView
) {
    fun onSurfaceCreated(surface: Surface)
    fun setVideoSize(width: Int, height: Int)
}
```

**Deliverables**:
- Live video streaming from CamControl to CamViewer
- Connection management UI
- Stable 30fps playback

**Success Criteria**:
- Video displays with < 200ms latency
- No frame drops or artifacts
- Reconnects automatically on network issues
- Handles CamControl app restart gracefully

### Phase 3: Target Selection UI
**Duration**: 3-4 days

**Tasks**:
1. ⏳ Implement bounding box selector
   - Touch event handling (ACTION_DOWN, ACTION_MOVE, ACTION_UP)
   - Draw bounding box on overlay
   - Visual feedback (color, thickness)
2. ⏳ Implement long-press handler
   - Detect long press
   - Create default-sized bbox at touch point
   - Ensure bbox stays within video bounds
3. ⏳ Implement coordinate converter
   - Screen coordinates → video frame coordinates
   - Handle aspect ratio differences
   - Handle video scaling/letterboxing
4. ⏳ Create overlay view
   - Transparent view over video
   - Draw bbox with Canvas
   - Animation for selection confirmation
5. ⏳ UI controls
   - "Select Target" button
   - Clear selection button
   - Visual mode indicator

**Key Classes**:
```kotlin
// selection/BoundingBoxSelector.kt
class BoundingBoxSelector(
    private val view: View,
    private val onBBoxSelected: (BoundingBox) -> Unit
) {
    private var selectionMode = false
    private var startPoint: PointF? = null
    private var currentBox: RectF? = null
    
    fun enableSelection()
    fun disableSelection()
    fun handleTouch(event: MotionEvent): Boolean
}

// selection/CoordinateConverter.kt
object CoordinateConverter {
    fun screenToFrame(
        screenX: Float,
        screenY: Float,
        viewWidth: Int,
        viewHeight: Int,
        frameWidth: Int,
        frameHeight: Int
    ): PointF
    
    fun normalizeBBox(bbox: RectF, frameWidth: Int, frameHeight: Int): NormalizedBBox
}

// ui/TargetSelectionView.kt
class TargetSelectionView @JvmOverloads constructor(
    context: Context,
    attrs: AttributeSet? = null
) : View(context, attrs) {
    var currentBBox: RectF? = null
        set(value) {
            field = value
            invalidate()
        }
    
    override fun onDraw(canvas: Canvas)
}
```

**Deliverables**:
- Working bounding box selection
- Visual feedback during selection
- Accurate coordinate transformation

**Success Criteria**:
- User can draw bbox smoothly
- Long-press creates default bbox
- Coordinates accurately map to video frame
- Visual feedback is clear and responsive

### Phase 4: Orin Integration
**Duration**: 2-3 days

**Tasks**:
1. ⏳ Define target selection protocol (JSON schema)
2. ⏳ Implement Orin HTTP client
   - POST request to target API
   - Error handling
   - Timeout management
3. ⏳ Create target selection message builder
   - Build JSON payload
   - Include frame metadata
   - Add timestamp
4. ⏳ Implement Orin target API server (Python)
   - Flask or FastAPI endpoint
   - Receive target selection
   - Publish to ROS2 topic
5. ⏳ Testing and validation
   - End-to-end target selection flow
   - Coordinate accuracy verification
   - Network error scenarios

**Orin Server** (`orin/target_api_server.py`):
```python
from fastapi import FastAPI, HTTPException
from pydantic import BaseModel
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json

class TargetROI(BaseModel):
    bbox: dict
    frame_resolution: dict
    normalized: dict
    timestamp: int

app = FastAPI()

class TargetPublisher(Node):
    def __init__(self):
        super().__init__('target_publisher')
        self.publisher = self.create_publisher(String, '/target_roi', 10)
    
    def publish_target(self, roi: TargetROI):
        msg = String()
        msg.data = roi.json()
        self.publisher.publish(msg)

@app.post("/target")
async def receive_target(roi: TargetROI):
    # Publish to ROS2
    target_pub.publish_target(roi)
    return {"status": "success"}
```

**CamViewer Client** (`network/OrinClient.kt`):
```kotlin
class OrinClient(private val baseUrl: String) {
    private val client = HttpClient(CIO) {
        install(ContentNegotiation) {
            json()
        }
    }
    
    suspend fun sendTargetSelection(
        bbox: BoundingBox,
        frameResolution: Resolution,
        timestamp: Long
    ): Result<Unit> = runCatching {
        client.post("$baseUrl/target") {
            contentType(ContentType.Application.Json)
            setBody(TargetSelectionPayload(bbox, frameResolution, timestamp))
        }
    }
}
```

**Deliverables**:
- Working target selection API
- ROS2 integration on Orin
- End-to-end testing

**Success Criteria**:
- Target selections reach Orin successfully
- ROS2 topic receives correct data
- < 100ms latency for target selection
- Handles network errors gracefully

### Phase 5: Developer Mode
**Duration**: 3-4 days

**Tasks**:
1. ⏳ Create developer mode UI
   - Bottom sheet or side panel
   - Control widgets (sliders, buttons, spinners)
   - Telemetry display
2. ⏳ Implement camera controls
   - Zoom control (slider)
   - Camera switch (front/back)
   - Video profile selection (dropdown)
   - Bitrate/codec controls
3. ⏳ Implement telemetry display
   - Real-time updates
   - Value formatting
   - Connection status
4. ⏳ Wire controls to WebSocket
   - Send control commands
   - Receive telemetry updates
   - Two-way state synchronization
5. ⏳ Settings persistence
   - SharedPreferences for connection details
   - Remember dev mode state
   - Save last used settings

**UI Layout** (Bottom Sheet):
```xml
<!-- fragment_developer_mode.xml -->
<ScrollView>
    <LinearLayout orientation="vertical">
        <!-- Connection Settings -->
        <TextView text="Connection"/>
        <EditText hint="CamControl IP" />
        <EditText hint="Port" />
        <Button text="Connect"/>
        
        <!-- Camera Controls -->
        <TextView text="Camera Controls"/>
        <SeekBar hint="Zoom" />
        <TextView text="Zoom: 2.5x"/>
        <RadioGroup>
            <RadioButton text="Back Camera"/>
            <RadioButton text="Front Camera"/>
        </RadioGroup>
        
        <Spinner hint="Video Profile"/>
        <Spinner hint="Codec"/>
        <EditText hint="Bitrate (Mbps)"/>
        
        <!-- Telemetry -->
        <TextView text="Telemetry"/>
        <TextView text="AF: FOCUSED_LOCKED"/>
        <TextView text="AE: CONVERGED"/>
        <TextView text="ISO: 250"/>
        <TextView text="Exposure: 16.7 ms"/>
        <TextView text="FPS: 29.8"/>
    </LinearLayout>
</ScrollView>
```

**Deliverables**:
- Functional developer mode
- All controls working
- Telemetry display updating

**Success Criteria**:
- Can control camera from CamViewer
- Telemetry updates in real-time
- UI is responsive and intuitive
- Settings persist across restarts

### Phase 6: Polish & Testing
**Duration**: 3-4 days

**Tasks**:
1. ⏳ UI/UX improvements
   - Material Design 3 components
   - Dark mode support
   - Responsive layouts
   - Loading indicators
   - Error messages
2. ⏳ Performance optimization
   - Memory leak fixes
   - Frame dropping analysis
   - Battery usage optimization
3. ⏳ Error handling
   - Network disconnection
   - Invalid video data
   - Decoder errors
   - Graceful degradation
4. ⏳ Testing
   - Unit tests for key components
   - Integration tests for network layer
   - UI tests for target selection
   - End-to-end testing
5. ⏳ Documentation
   - User guide
   - Developer documentation
   - API documentation

**Deliverables**:
- Polished, production-ready app
- Test coverage > 60%
- User documentation

**Success Criteria**:
- No crashes during 1-hour continuous use
- Battery drain < 5% per hour
- All error scenarios handled gracefully
- Documentation complete

## Development Roadmap

### Week 1
- ✅ Architecture & Planning
- ⏳ Phase 1: Project Setup (days 1-3)
- ⏳ Phase 2: Start WebSocket client (days 4-5)

### Week 2
- ⏳ Phase 2: Complete video streaming (days 1-3)
- ⏳ Phase 3: Target selection UI (days 4-5)

### Week 3
- ⏳ Phase 3: Complete target selection (days 1-2)
- ⏳ Phase 4: Orin integration (days 3-5)

### Week 4
- ⏳ Phase 5: Developer mode (days 1-4)
- ⏳ Phase 6: Start polish & testing (day 5)

### Week 5
- ⏳ Phase 6: Complete polish & testing (days 1-3)
- ⏳ Final integration testing (days 4-5)

**Total Duration**: ~5 weeks (part-time) or ~3 weeks (full-time)

## Dependencies & Prerequisites

### Development Environment
- Android Studio Hedgehog (2023.1.1) or later
- Kotlin 1.9+
- Gradle 8.0+
- Android SDK 26+ (target 34)
- Physical Android device (for testing)

### Libraries (CamViewer)
```kotlin
// Network
implementation("io.ktor:ktor-client-core:2.3.12")
implementation("io.ktor:ktor-client-cio:2.3.12")
implementation("io.ktor:ktor-client-content-negotiation:2.3.12")
implementation("io.ktor:ktor-serialization-kotlinx-json:2.3.12")
implementation("com.squareup.okhttp3:okhttp:4.12.0")

// Coroutines
implementation("org.jetbrains.kotlinx:kotlinx-coroutines-android:1.8.1")
implementation("org.jetbrains.kotlinx:kotlinx-serialization-json:1.6.3")

// AndroidX
implementation("androidx.appcompat:appcompat:1.7.0")
implementation("androidx.core:core-ktx:1.13.1")
implementation("androidx.activity:activity-ktx:1.9.2")
implementation("androidx.fragment:fragment-ktx:1.8.4")
implementation("androidx.constraintlayout:constraintlayout:2.1.4")
implementation("androidx.lifecycle:lifecycle-viewmodel-ktx:2.8.6")
implementation("androidx.lifecycle:lifecycle-runtime-ktx:2.8.6")
implementation("com.google.android.material:material:1.12.0")

// Testing
testImplementation("junit:junit:4.13.2")
androidTestImplementation("androidx.test.ext:junit:1.2.1")
androidTestImplementation("androidx.test.espresso:espresso-core:3.6.1")
```

### Orin Dependencies
```bash
# Python packages
pip install fastapi uvicorn pydantic

# ROS2 (already installed)
source /opt/ros/humble/setup.bash
```

## Risk Assessment & Mitigation

### Risk 1: Video Decoding Latency
**Impact**: High  
**Probability**: Medium  
**Mitigation**:
- Use hardware decoding (MediaCodec)
- Optimize buffer sizes
- Profile and benchmark
- Consider dropping frames if latency exceeds threshold

### Risk 2: Network Reliability
**Impact**: High  
**Probability**: Medium  
**Mitigation**:
- Implement reconnection logic
- Add connection monitoring
- Handle partial frame drops
- Show clear connection status to user

### Risk 3: Coordinate Transformation Errors
**Impact**: Medium  
**Probability**: Low  
**Mitigation**:
- Extensive testing with different resolutions
- Visual debugging mode (show coordinate overlays)
- Unit tests for transformation logic
- Validation on Orin side

### Risk 4: Package Conflicts
**Impact**: Low  
**Probability**: Very Low  
**Mitigation**:
- Separate package names
- No shared resources
- Independent build configurations
- Test both apps running simultaneously

### Risk 5: Performance on Lower-End Devices
**Impact**: Medium  
**Probability**: Medium  
**Mitigation**:
- Target min SDK 26 (wide device support)
- Optimize rendering pipeline
- Add quality settings (resolution/bitrate)
- Profile on various devices

## Testing Strategy

### Unit Tests
- Coordinate transformation logic
- Protocol message serialization
- State management
- Network retry logic

### Integration Tests
- WebSocket client connection
- Video decoder with mock frames
- Orin API client
- Target selection flow

### UI Tests (Espresso)
- Video display
- Bounding box drawing
- Developer mode controls
- Connection management

### Manual Testing
- End-to-end video streaming
- Target selection accuracy
- Network interruption recovery
- Multi-device scenarios

### Performance Testing
- Frame rate measurement
- Latency profiling
- Memory usage monitoring
- Battery drain analysis

## Success Metrics

### Functional
- ✅ Video streams from CamControl to CamViewer
- ✅ User can select targets via bbox
- ✅ Target selections reach Orin
- ✅ Developer mode provides full control
- ✅ No conflicts with CamControl app

### Performance
- Video latency: < 200ms end-to-end
- Frame rate: 30 fps sustained
- Target selection latency: < 100ms
- Memory usage: < 200MB
- Battery drain: < 5% per hour

### Quality
- No crashes during 1-hour continuous use
- Handles all error scenarios gracefully
- UI is responsive (< 16ms per frame)
- Code coverage > 60%

## Next Steps

1. **Immediate** (Today):
   - ✅ Review and approve architecture
   - ✅ Create SYSTEM_ARCHITECTURE.md
   - ✅ Create this implementation plan

2. **Day 1**:
   - Update Gradle build files
   - Create camviewer module structure
   - Set up basic UI skeleton

3. **Day 2-3**:
   - Implement WebSocket client
   - Start video decoder implementation

4. **Week 1 Review**:
   - Demo basic video streaming
   - Validate approach
   - Adjust plan if needed

## Resources & References

- [Android MediaCodec Documentation](https://developer.android.com/reference/android/media/MediaCodec)
- [WebSocket RFC 6455](https://datatracker.ietf.org/doc/html/rfc6455)
- [OkHttp WebSocket](https://square.github.io/okhttp/features/websockets/)
- [Ktor Client](https://ktor.io/docs/getting-started-ktor-client.html)
- [Material Design 3](https://m3.material.io/)
- [CamControl existing code](../app/src/main/)
- [SYSTEM_ARCHITECTURE.md](SYSTEM_ARCHITECTURE.md)

## Appendix: File Checklist

### To Create
- [ ] `camviewer/build.gradle.kts`
- [ ] `camviewer/src/main/AndroidManifest.xml`
- [ ] `camviewer/src/main/java/com/example/camviewer/MainActivity.kt`
- [ ] `camviewer/src/main/java/com/example/camviewer/network/CamControlClient.kt`
- [ ] `camviewer/src/main/java/com/example/camviewer/network/OrinTargetClient.kt`
- [ ] `camviewer/src/main/java/com/example/camviewer/network/OrinMediaClient.kt` **(NEW)**
- [ ] `camviewer/src/main/java/com/example/camviewer/network/ProtocolModels.kt`
- [ ] `camviewer/src/main/java/com/example/camviewer/video/VideoDecoder.kt`
- [ ] `camviewer/src/main/java/com/example/camviewer/video/VideoRenderer.kt`
- [ ] `camviewer/src/main/java/com/example/camviewer/media/MediaRepository.kt` **(NEW)**
- [ ] `camviewer/src/main/java/com/example/camviewer/media/MediaDownloader.kt` **(NEW)**
- [ ] `camviewer/src/main/java/com/example/camviewer/media/ThumbnailCache.kt` **(NEW)**
- [ ] `camviewer/src/main/java/com/example/camviewer/selection/BoundingBoxSelector.kt`
- [ ] `camviewer/src/main/java/com/example/camviewer/selection/CoordinateConverter.kt`
- [ ] `camviewer/src/main/java/com/example/camviewer/ui/VideoDisplayFragment.kt`
- [ ] `camviewer/src/main/java/com/example/camviewer/ui/MediaGalleryFragment.kt` **(NEW)**
- [ ] `camviewer/src/main/java/com/example/camviewer/ui/MediaDetailFragment.kt` **(NEW)**
- [ ] `camviewer/src/main/java/com/example/camviewer/ui/DeveloperModeFragment.kt`
- [ ] `camviewer/src/main/java/com/example/camviewer/ui/TargetSelectionView.kt`
- [ ] `camviewer/src/main/res/layout/activity_main.xml`
- [ ] `camviewer/src/main/res/layout/fragment_video_display.xml`
- [ ] `camviewer/src/main/res/layout/fragment_media_gallery.xml` **(NEW)**
- [ ] `camviewer/src/main/res/layout/fragment_media_detail.xml` **(NEW)**
- [ ] `camviewer/src/main/res/layout/item_media_thumbnail.xml` **(NEW)**
- [ ] `camviewer/src/main/res/layout/fragment_developer_mode.xml`
- [ ] `orin/target_api_server.py`
- [ ] `orin/media_api_server.py` **(NEW)**
- [ ] `orin/ros2_camcontrol/msg/TargetROI.msg`
- [ ] `orin/ros2_camcontrol/msg/TrackingResult.msg`
- [ ] `orin/ros2_camcontrol/ros2_camcontrol/target_tracker_node.py`

### To Modify
- [ ] `settings.gradle.kts` (add camviewer module)
- [ ] `README.md` (add CamViewer overview)
- [ ] `README_zh.md` (add CamViewer overview in Chinese)
- [ ] `orin/ARCHITECTURE.md` (update with target selection flow)

---

**Document Version**: 1.0  
**Last Updated**: November 11, 2025  
**Status**: Draft - Awaiting Approval
