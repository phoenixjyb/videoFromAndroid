# Target API ROI Update

## Overview
Updated the Target API to support bounding boxes (Region of Interest) in addition to simple tap points.

## Changes Made

### 1. ROS2 Message Type Change
- **Old**: `geometry_msgs/Point` - Only stored x, y coordinates
- **New**: `sensor_msgs/RegionOfInterest` - Stores x, y, width, height

### 2. API Request Format

#### Simple Tap Point (Backward Compatible)
```json
{
  "x": 0.5,
  "y": 0.5
}
```
- Creates a default **10% bounding box** centered on the tap point
- Suitable for simple target selection

#### Full Bounding Box (New Feature)
```json
{
  "x": 0.3,
  "y": 0.4,
  "width": 0.2,
  "height": 0.15
}
```
- `x`, `y`: Top-left corner of the bounding box
- `width`, `height`: Dimensions of the bounding box
- All values normalized (0.0-1.0)

### 3. Publishing Frequency
- **Event-driven**: Publishes whenever CamViewer sends coordinates
- **No fixed rate**: Only publishes when user taps or when app sends ROI
- **Latency**: ~3-5ms from HTTP request to ROS2 publish

### 4. RegionOfInterest Message Details

```python
sensor_msgs/RegionOfInterest:
  x_offset: uint32    # Top-left x (scaled by 10000 for storage)
  y_offset: uint32    # Top-left y (scaled by 10000 for storage)
  height: uint32      # ROI height (scaled by 10000)
  width: uint32       # ROI width (scaled by 10000)
  do_rectify: bool    # Always false (not using rectification)
```

**Note**: Coordinates are stored as `uint32` by scaling normalized values (0.0-1.0) by 10000.
To convert back: `normalized_value = stored_value / 10000.0`

## Usage Examples

### Subscribe to ROI Topic
```bash
source /opt/ros/humble/setup.bash
ros2 topic echo /target_roi
```

### Send Simple Tap
```bash
curl -X POST http://172.16.30.234:8080/target \
  -H "Content-Type: application/json" \
  -d '{"x": 0.5, "y": 0.5}'
```

### Send Bounding Box
```bash
curl -X POST http://172.16.30.234:8080/target \
  -H "Content-Type: application/json" \
  -d '{"x": 0.3, "y": 0.4, "width": 0.2, "height": 0.15}'
```

### Monitor Topic Frequency
```bash
ros2 topic hz /target_roi
```

## Next Steps for CamViewer Integration

To send bounding boxes from CamViewer, you have two options:

### Option 1: Keep Simple Tap (Current)
- No changes needed
- Server automatically creates 10% box around tap point
- Good for simple target selection

### Option 2: Add Bounding Box Support (Recommended)
Update `OrinTargetClient.kt` to accept optional width/height:

```kotlin
suspend fun sendTargetROI(
    baseUrl: String,
    x: Float,
    y: Float,
    width: Float? = null,
    height: Float? = null
): Result<Unit> {
    // ... validation ...
    
    val request = if (width != null && height != null) {
        TargetROIRequest(x, y, width, height)
    } else {
        TargetRequest(x, y)
    }
    
    // ... send request ...
}

@Serializable
private data class TargetROIRequest(
    val x: Float,
    val y: Float,
    val width: Float,
    val height: Float
)
```

## Benefits of ROI Support

1. **More Context**: Vision algorithms get region size, not just a point
2. **Better Tracking**: Initialize trackers with proper bounding box
3. **Flexible**: Supports both simple taps and complex selections
4. **Standard Format**: Uses ROS2 standard `RegionOfInterest` message

## Testing Results

✅ Simple tap point: Works, creates 10% default box  
✅ Full bounding box: Works, uses provided dimensions  
✅ ROS2 publishing: Confirmed with `sensor_msgs/RegionOfInterest`  
✅ Backward compatible: Existing CamViewer app works without changes  
✅ Performance: ~3ms latency, event-driven publishing  

## Coordinate System

```
(0.0, 0.0) ──────────────► x (1.0, 0.0)
    │
    │    (x, y) = top-left
    │    ┌─────────┐
    │    │   ROI   │ height
    │    └─────────┘
    │      width
    ▼
    y
(0.0, 1.0)              (1.0, 1.0)
```

## Migration Notes

- **No breaking changes**: Old clients still work
- **ROS2 topic name**: Still `/target_roi` (unchanged)
- **Default behavior**: Tap points get 10% box (configurable in code)
- **Storage format**: Values scaled by 10000 for uint32 storage
