#!/bin/bash
# Setup script for Recomo Service Control API systemd service

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SERVICE_FILE="recomo_service_control.service"
SYSTEMD_PATH="/etc/systemd/system/$SERVICE_FILE"

echo "=========================================="
echo "Setting up Recomo Service Control API as systemd service"
echo "=========================================="
echo

# Check if running as root or with sudo
if [ "$EUID" -ne 0 ]; then 
    echo "❌ This script must be run with sudo"
    echo "Usage: sudo ./setup_service_control_systemd.sh"
    exit 1
fi

# Install Python dependencies
echo "Installing Python dependencies..."
pip3 install fastapi uvicorn pydantic psutil 2>&1 | grep -v "already satisfied" || true
echo "✓ Dependencies installed"
echo

# Stop any existing processes on port 8083
echo "Checking for existing services on port 8083..."
EXISTING_PID=$(lsof -ti :8083 2>/dev/null)
if [ -n "$EXISTING_PID" ]; then
    echo "Found existing process (PID: $EXISTING_PID) - stopping it..."
    kill -9 $EXISTING_PID 2>/dev/null || true
    sleep 1
    echo "✓ Existing process stopped"
else
    echo "✓ Port 8083 is free"
fi
echo

# Stop existing systemd service if running
if systemctl is-active --quiet recomo_service_control.service 2>/dev/null; then
    echo "Stopping existing systemd service..."
    systemctl stop recomo_service_control.service
    echo "✓ Existing service stopped"
fi
echo

# Copy service file to systemd
echo "Installing systemd service..."
cp "$SCRIPT_DIR/$SERVICE_FILE" "$SYSTEMD_PATH"
echo "✓ Service file copied to $SYSTEMD_PATH"
echo

# Reload systemd
echo "Reloading systemd daemon..."
systemctl daemon-reload
echo "✓ Systemd reloaded"
echo

# Enable service to start on boot
echo "Enabling service to start on boot..."
systemctl enable recomo_service_control.service
echo "✓ Service enabled"
echo

# Start the service
echo "Starting service..."
systemctl start recomo_service_control.service
sleep 2
echo

# Check status
echo "Service status:"
systemctl status recomo_service_control.service --no-pager -l
echo

echo "=========================================="
echo "✓ Setup complete!"
echo "=========================================="
echo
echo "Useful commands:"
echo "  Check status:  sudo systemctl status recomo_service_control"
echo "  View logs:     sudo journalctl -u recomo_service_control -f"
echo "  Stop service:  sudo systemctl stop recomo_service_control"
echo "  Start service: sudo systemctl start recomo_service_control"
echo "  Restart:       sudo systemctl restart recomo_service_control"
echo "  Disable boot:  sudo systemctl disable recomo_service_control"
echo
