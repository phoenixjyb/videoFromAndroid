#!/bin/bash
# Quick start script for Orin Media API

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

echo "=================================="
echo "Orin Media API - Quick Start"
echo "=================================="
echo

# Install dependencies if needed
echo "Installing/updating dependencies..."
python3 -m pip install --user -q --upgrade pip
python3 -m pip install --user -q -r requirements.txt
echo "✓ Dependencies installed"

echo
echo "=================================="
echo "Starting Media API Server"
echo "=================================="
echo "Listening on: http://0.0.0.0:8081"
echo "Media directory: ./media/"
echo
echo "API endpoints:"
echo "  GET  /media - List all media files"
echo "  GET  /media/{filename} - Download file"
echo "  POST /upload - Upload new media"
echo "  DELETE /media/{filename} - Delete file"
echo
echo "Press Ctrl+C to stop"
echo "=================================="
echo

# Start the server
python3 media_api.py
