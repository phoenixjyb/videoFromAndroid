# Tools

Utility scripts for development and testing.

## Quick Logs Script

### `quick_logs.sh`
Monitors Android logcat for application-specific logs.

**Usage:**
```bash
./tools/quick_logs.sh
```

**Note:** The phone-to-ROS2 bridge script has been moved to `orin/start_phone_ros2_bridge.sh`.

## See Also

- `/tests/` - Test scripts and validation tools
- `/scripts/` - Production-ready recording and streaming scripts
- `/orin/` - ROS2 integration tools and service management
  - `start_all_services.sh` - Start all Orin services (target API, media API, camera relay)
  - `stop_all_services.sh` - Stop all Orin services
  - `start_phone_ros2_bridge.sh` - Bridge phone video/controls to ROS2 topics (with ADB forwarding)
