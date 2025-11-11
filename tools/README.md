# Tools

Utility scripts for development and testing.

## Quick Start Scripts

### `quick_start.sh`
Main development workflow script that handles:
- Building the Android APK
- Installing to connected device
- Setting up ADB port forwarding
- Starting the camera service

**Usage:**
```bash
./tools/quick_start.sh
```

### `quick_logs.sh`
Monitors Android logcat for application-specific logs.

**Usage:**
```bash
./tools/quick_logs.sh
```

## See Also

- `/tests/` - Test scripts and validation tools
- `/scripts/` - Production-ready recording and streaming scripts
- `/orin/` - ROS2 integration tools
