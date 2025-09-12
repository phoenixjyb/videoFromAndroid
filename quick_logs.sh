#!/bin/bash
# Simple log monitor that exits after a few seconds
echo "📱 Monitoring app logs for 10 seconds..."
timeout 10 adb logcat -v time -s MainActivity:I ControlServer:I VideoEncoder:I Camera2Controller:I || \
    adb logcat -d -v time -s MainActivity:I ControlServer:I VideoEncoder:I Camera2Controller:I | tail -20
