#!/bin/bash
# Start Orin Service Control API
#
# Usage:
#   ./start_service_control_api.sh
#   SERVICE_CONTROL_PIN=1234 ./start_service_control_api.sh  # With PIN protection
#
# Environment variables:
#   SERVICE_CONTROL_PORT - API port (default: 8083)
#   SERVICE_CONTROL_PIN  - PIN for start/stop operations (optional)

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

PORT="${SERVICE_CONTROL_PORT:-8083}"
PIN="${SERVICE_CONTROL_PIN:-}"

echo "=========================================="
echo "Orin Service Control API - Startup"
echo "=========================================="
echo

# Check if already running
if [ -f ".service_control_api.pid" ]; then
    OLD_PID=$(cat .service_control_api.pid)
    if ps -p $OLD_PID > /dev/null 2>&1; then
        echo "⚠ Service Control API already running (PID: $OLD_PID)"
        echo "  Stop it first with: kill $OLD_PID"
        exit 1
    fi
    rm -f .service_control_api.pid
fi

# Install dependencies
echo "Installing/updating dependencies..."
python3 -m pip install --user -q --upgrade pip
python3 -m pip install --user -q fastapi uvicorn pydantic
echo "✓ Dependencies installed"
echo

# Start API server in background
echo "Starting Service Control API..."
echo "  URL: http://0.0.0.0:$PORT"
echo "  Logs: service_control_api.log"
if [ -n "$PIN" ]; then
    echo "  Security: PIN protection enabled"
fi

# Export PIN for the API process
export SERVICE_CONTROL_PIN="$PIN"

nohup python3 service_control_api.py --port $PORT > service_control_api.log 2>&1 &
API_PID=$!

# Save PID
echo $API_PID > .service_control_api.pid

echo "  PID: $API_PID"
echo "✓ Service Control API started"
echo

# Test if server is responding
sleep 2
if curl -s http://localhost:$PORT/ > /dev/null; then
    echo "✓ Server is responding"
else
    echo "⚠ Server may not be responding yet, check logs"
fi

echo
echo "=========================================="
echo "Access the API at:"
echo "  http://<orin-ip>:$PORT/docs"
echo "=========================================="
