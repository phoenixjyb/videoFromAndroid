# Tests

Test scripts and validation tools for the videoFromAndroid project.

## Test Scripts

### Connection Tests
- **`test_websocket.py`** - Basic WebSocket connection test
- **`test_ws_connection.py`** - WebSocket connectivity validation

### Command Tests
- **`test_commands.py`** - Test camera control commands
- **`test_webui_commands.py`** - WebUI command interface testing
- **`test_camera_switch.py`** - Camera switching functionality test

### Telemetry Tests
- **`test_telemetry_ws.py`** - WebSocket telemetry stream validation

### ROS2 Integration Tests
- **`test_recomo_rgb_ros2.sh`** - Complete ROS2 integration test script
  - Tests image publishing, camera control, telemetry
  - Located at root for convenience, but logically part of tests

### Browser Tests
- **`webcodecs-selftest.html`** - WebCodecs API self-test page
  - Browser-based video decoding validation
  - Open in Chrome/Edge to test WebCodecs support

## Running Tests

### Python Tests
```bash
python3 tests/test_websocket.py
python3 tests/test_commands.py
```

### ROS2 Tests
```bash
./test_recomo_rgb_ros2.sh
# Or with subscriber validation:
./scripts/test_with_subscriber.sh
```

### Browser Tests
```bash
# Start a local server or open directly:
firefox tests/webcodecs-selftest.html
```

## Test Output

Test outputs (saved images, videos) are stored in:
- `/saved_images/` - Image capture test results
- `/saved_videos/` - Video recording test results

Both directories are ignored by git.

## See Also

- `/scripts/test_with_subscriber.sh` - ROS2 subscriber validation
- `/orin/ros2_camcontrol/` - ROS2 test tools
- `/scripts/stream_diagnostics.sh` - Stream health monitoring
