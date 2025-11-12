#!/bin/bash
# Stop script for all Orin API services

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

echo "=========================================="
echo "Stopping Orin API Services..."
echo "=========================================="
echo

# Stop Target API
if [ -f .target_api.pid ]; then
    TARGET_PID=$(cat .target_api.pid)
    if ps -p $TARGET_PID > /dev/null 2>&1; then
        echo "Stopping Target API (PID: $TARGET_PID)..."
        kill $TARGET_PID
        echo "✓ Target API stopped"
    else
        echo "⚠ Target API not running"
    fi
    rm -f .target_api.pid
else
    echo "⚠ No Target API PID file found"
fi

echo

# Stop Media API
if [ -f .media_api.pid ]; then
    MEDIA_PID=$(cat .media_api.pid)
    if ps -p $MEDIA_PID > /dev/null 2>&1; then
        echo "Stopping Media API (PID: $MEDIA_PID)..."
        kill $MEDIA_PID
        echo "✓ Media API stopped"
    else
        echo "⚠ Media API not running"
    fi
    rm -f .media_api.pid
else
    echo "⚠ No Media API PID file found"
fi

echo
echo "=========================================="
echo "✓ Services stopped"
echo "=========================================="
