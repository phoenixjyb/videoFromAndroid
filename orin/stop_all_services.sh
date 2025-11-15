#!/bin/bash
# Stop script for all Orin API services

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

echo "=========================================="
echo "Stopping Orin API Services..."
echo "=========================================="
echo

# Function to stop process by port
stop_by_port() {
    local port=$1
    local service_name=$2
    
    # Find PID listening on the port
    local pid=$(lsof -ti :$port 2>/dev/null)
    
    if [ -n "$pid" ]; then
        echo "Found $service_name on port $port (PID: $pid)"
        kill $pid 2>/dev/null
        sleep 0.5
        
        # Force kill if still running
        if ps -p $pid > /dev/null 2>&1; then
            echo "Force stopping $service_name..."
            kill -9 $pid 2>/dev/null
        fi
        echo "✓ $service_name stopped"
        return 0
    fi
    return 1
}

# Stop Target API
TARGET_STOPPED=false
if [ -f .target_api.pid ]; then
    TARGET_PID=$(cat .target_api.pid)
    if ps -p $TARGET_PID > /dev/null 2>&1; then
        echo "Stopping Target API (PID: $TARGET_PID from PID file)..."
        kill $TARGET_PID
        sleep 0.5
        # Force kill if still running
        if ps -p $TARGET_PID > /dev/null 2>&1; then
            kill -9 $TARGET_PID 2>/dev/null
        fi
        echo "✓ Target API stopped"
        TARGET_STOPPED=true
    fi
    rm -f .target_api.pid
fi

# If not stopped by PID file, try by port
if [ "$TARGET_STOPPED" = false ]; then
    if ! stop_by_port 8082 "Target API"; then
        echo "⚠ Target API not running"
    fi
fi

echo

# Stop Media API
MEDIA_STOPPED=false
if [ -f .media_api.pid ]; then
    MEDIA_PID=$(cat .media_api.pid)
    if ps -p $MEDIA_PID > /dev/null 2>&1; then
        echo "Stopping Media API (PID: $MEDIA_PID from PID file)..."
        kill $MEDIA_PID
        sleep 0.5
        # Force kill if still running
        if ps -p $MEDIA_PID > /dev/null 2>&1; then
            kill -9 $MEDIA_PID 2>/dev/null
        fi
        echo "✓ Media API stopped"
        MEDIA_STOPPED=true
    fi
    rm -f .media_api.pid
fi

# If not stopped by PID file, try by port
if [ "$MEDIA_STOPPED" = false ]; then
    if ! stop_by_port 8081 "Media API"; then
        echo "⚠ Media API not running"
    fi
fi

echo

# Stop Camera Relay
RELAY_STOPPED=false
if [ -f .camera_relay.pid ]; then
    RELAY_PID=$(cat .camera_relay.pid)
    if ps -p $RELAY_PID > /dev/null 2>&1; then
        echo "Stopping Camera Relay (PID: $RELAY_PID from PID file)..."
        kill $RELAY_PID
        sleep 0.5
        # Force kill if still running
        if ps -p $RELAY_PID > /dev/null 2>&1; then
            kill -9 $RELAY_PID 2>/dev/null
        fi
        echo "✓ Camera Relay stopped"
        RELAY_STOPPED=true
    fi
    rm -f .camera_relay.pid
fi

# If not stopped by PID file, try by process name
if [ "$RELAY_STOPPED" = false ]; then
    RELAY_PID=$(pgrep -f "camera_control_relay.py" 2>/dev/null)
    if [ -n "$RELAY_PID" ]; then
        echo "Found Camera Relay (PID: $RELAY_PID)"
        kill $RELAY_PID 2>/dev/null
        sleep 0.5
        if ps -p $RELAY_PID > /dev/null 2>&1; then
            kill -9 $RELAY_PID 2>/dev/null
        fi
        echo "✓ Camera Relay stopped"
    else
        echo "⚠ Camera Relay not running"
    fi
fi

echo

# Stop ROS bridge if running
if [ -f .ros_bridge.pid ] || pgrep -f "ros2_camcontrol\.ws_to_image" >/dev/null 2>&1; then
    echo "Stopping ROS video bridge..."
    ./stop_ros_bridge.sh || true
else
    echo "ROS video bridge not running"
fi

echo
echo "=========================================="
echo "✓ All services stopped"
echo "=========================================="
