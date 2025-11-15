# Orin Service Control Feature

## Overview

Remote control feature for starting/stopping Orin services directly from the CamViewer tablet app.

## Architecture

### Backend (Orin)
- **service_control_api.py**: FastAPI server exposing REST endpoints
- **start_all_services.sh / stop_all_services.sh**: Manage Target API, Media API, Camera Relay
- **start_ros_bridge.sh / stop_ros_bridge.sh**: Manage the ROS video bridge (ADB + ws_to_image)
- **Port**: 8083
- **Endpoints**:
  - `GET /api/services/status` - Get current service status
  - `POST /api/services/start` - Start all services (now includes ROS bridge by default)
  - `POST /api/services/stop` - Stop all services
  - `GET /api/services/logs/{service_id}` - Get recent log lines

### Frontend (CamViewer)
- **OrinServiceRepository**: HTTP client using Ktor
- **OrinControlViewModel**: State management with StateFlow
- **OrinControlScreen**: Compose UI with service cards
- **Navigation**: 4th tab in NavigationRail ("Orin" with Computer icon)

## Deployment

### On Orin

1. Start the service control API:
```bash
cd /path/to/camControl/orin
./start_service_control_api.sh
```

2. Verify it's running:
```bash
curl http://localhost:8083/api/services/status
```

> **ADB requirement**: The ROS bridge relies on the Android phone being plugged into the Orin over USB with developer mode enabled. If ADB is unavailable, Target/Media APIs will still start, but the bridge status will show as stopped with log details in `ros_bridge.log`.

### On Tablet

1. Configure network in Settings screen:
   - Select ZeroTier or T8Space preset
   - Or set custom Orin IP

2. Navigate to "Orin" tab (4th tab with computer icon)

3. Use controls:
   - **Start All**: Launch Target API + Media API
   - **Stop All**: Stop both services
   - **Refresh**: Manual status check
   - **Auto-refresh toggle**: Enable/disable 5s auto-refresh

## Features

### Service Status Cards
- Green/Red indicator (running/stopped)
- Service name (Target API, Media API, Camera Relay, ROS Bridge)
- Port number (if applicable)
- PID (when running)
- Uptime (formatted as hours/minutes/seconds)
- Recent log lines (expandable)

### Control Buttons
- **Start All**: Runs `start_all_services.sh` (exports `START_ROS_BRIDGE=1` by default)
- **Stop All**: Runs `stop_all_services.sh` (only enabled when services running)
- Loading indicator during operations
- Error messages with dismiss button

### Auto-refresh
- Toggle in top bar
- Refreshes every 5 seconds when enabled
- Icon changes to show state

## API Response Format

### Service Status
```json
{
  "target_api": {
    "name": "Target API",
    "running": true,
    "pid": 12345,
    "uptime_seconds": 120.5,
    "port": 8082,
    "last_log_lines": [
      "INFO: Server started",
      "INFO: Listening on port 8082"
    ]
  },
  "media_api": {
    "name": "Media API",
    "running": false,
    "pid": null,
    "uptime_seconds": null,
    "port": 8081,
    "last_log_lines": []
  }
}
```

### Service Control Response
```json
{
  "success": true,
  "message": "Services started successfully",
  "services": {
    "target_api": { /* ServiceStatus object */ },
    "media_api": { /* ServiceStatus object */ }
  }
}
```

## Files

### Orin Backend
- `orin/service_control_api.py` - FastAPI server (285 lines)
- `orin/start_service_control_api.sh` - Startup script

### CamViewer Frontend
- `camviewer/src/main/java/com/example/camviewer/data/model/ServiceModels.kt` - Data models
- `camviewer/src/main/java/com/example/camviewer/data/repository/OrinServiceRepository.kt` - API client
- `camviewer/src/main/java/com/example/camviewer/ui/orin/OrinControlViewModel.kt` - ViewModel
- `camviewer/src/main/java/com/example/camviewer/ui/orin/OrinControlScreen.kt` - UI screen
- `camviewer/src/main/java/com/example/camviewer/ui/navigation/Screen.kt` - Navigation route
- `camviewer/src/main/java/com/example/camviewer/ui/navigation/CamViewerNavHost.kt` - Navigation setup

### Network Configuration

The service control API URL is derived from the Orin Target URL:
- Extract IP from `orinTargetUrl` (e.g., `192.168.100.150` from `http://192.168.100.150:8082`)
- Append port 8083: `http://192.168.100.150:8083`

### Presets
- **ZeroTier**: `http://192.168.100.150:8083`
- **T8Space**: `http://172.16.30.234:8083`
- **Custom**: Based on user-configured Orin IP

### Environment flags

- `SERVICE_CONTROL_START_ROS_BRIDGE=0` (host env) disables automatic ROS bridge start when CamViewer presses **Start All**.
- `START_ROS_BRIDGE=1 ./start_all_services.sh` (manual shell) forces the ROS bridge to be launched alongside the APIs.

## Testing

### Manual Testing

1. **Backend Only**:
```bash
# On Orin
./start_service_control_api.sh

# Check status
curl http://localhost:8083/api/services/status

# Start services
curl -X POST http://localhost:8083/api/services/start

# Stop services
curl -X POST http://localhost:8083/api/services/stop
```

2. **From Tablet**:
```bash
# Via adb shell from tablet
curl http://192.168.100.150:8083/api/services/status
```

3. **Full Integration**:
   - Build and install CamViewer APK
   - Navigate to Orin tab
   - Try Start All / Stop All buttons
   - Verify service status updates
   - Check log lines display

### Troubleshooting

**Error: Connection refused**
- Check if service_control_api.py is running: `ps aux | grep service_control_api`
- Verify port 8083 is listening: `netstat -tlnp | grep 8083`
- Check firewall: `sudo ufw status`

**Error: 404 Not Found**
- Verify API endpoints: `curl http://localhost:8083/`
- Check API logs: `tail -f service_control_api.log`

**Services not starting**
- Check PID files: `ls -la .*.pid`
- Verify shell scripts exist and are executable
- Check individual service logs: `tail -f target_api.log media_api.log`

**CamViewer can't connect**
- Verify network connectivity: ping Orin IP
- Check Orin IP in Settings matches actual IP
- Ensure both devices on same network (ZeroTier or T8Space)

## Future Enhancements

- [ ] Add individual service start/stop buttons
- [ ] Service health metrics (CPU, memory usage)
- [ ] Log streaming (WebSocket)
- [ ] Service restart capability
- [ ] Configuration editor
- [ ] Service startup order control
- [ ] Dependency checking
- [ ] Notification when services crash
