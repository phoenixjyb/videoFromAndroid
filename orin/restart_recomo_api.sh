#!/bin/bash
# Quick restart script for Recomo Service Control API

echo "=========================================="
echo "Restarting Recomo Service Control API"
echo "=========================================="
echo

# Stop any process on port 8083 (including manual starts)
echo "Stopping any processes on port 8083..."
EXISTING_PID=$(sudo lsof -ti :8083 2>/dev/null)
if [ -n "$EXISTING_PID" ]; then
    echo "Found process (PID: $EXISTING_PID) - stopping it..."
    sudo kill -9 $EXISTING_PID 2>/dev/null || true
    sleep 1
    echo "✓ Process stopped"
else
    echo "✓ No process on port 8083"
fi
echo

# Restart systemd service
echo "Restarting systemd service..."
sudo systemctl restart recomo_service_control.service
sleep 2
echo

# Show status
echo "Service status:"
sudo systemctl status recomo_service_control.service --no-pager -l
echo

# Test if it's responding
echo "Testing API..."
if curl -s http://localhost:8083/ > /dev/null 2>&1; then
    echo "✓ API is responding"
else
    echo "⚠ API not responding - check logs with: sudo journalctl -u recomo_service_control -f"
fi

echo
echo "=========================================="
echo "Done!"
echo "=========================================="
