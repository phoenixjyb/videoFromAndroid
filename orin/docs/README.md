# Orin Documentation Index

Complete documentation for the Orin camera control system.

## 📖 Getting Started

Start here if you're new to the system:

1. **[Quick Start Guide](guides/QUICKSTART.md)** - 5-minute setup
2. **[Setup Guide](guides/SETUP.md)** - Detailed installation
3. **[Camera Control Usage](guides/CAMERA_CONTROL.md)** - Control the phone camera

## 🔌 API Documentation

REST API and WebSocket endpoint references:

- **[Target API](api/TARGET_API.md)** - POST target detection data (port 8082)
- **[Media API](api/MEDIA_API.md)** - Stream video to desktop (port 8081)
- **[Camera Relay](api/CAMERA_RELAY.md)** - ROS2 ↔ WebSocket bridge

## 📚 User Guides

Step-by-step guides for common tasks:

- **[ROI Updates](guides/ROI_UPDATE.md)** - Working with region of interest data
- **[Listener Guide](guides/LISTENER.md)** - Subscribing to ROS2 topics
- **[Camera Control](guides/CAMERA_CONTROL.md)** - Zoom, switch, locks, bitrate

## 📋 Technical Reference

Design documents and specifications:

- **[Architecture](reference/ARCHITECTURE.md)** - System design overview
- **[WebSocket Format](reference/WS_FORMAT.md)** - Message protocols
- **[Directory Structure](reference/DIRECTORY.md)** - File organization
- **[Test Results](reference/TEST_RESULTS.md)** - Camera control testing

## 🔍 Quick Links

**Most Common Tasks:**
- Start all services → [Quick Start](guides/QUICKSTART.md#starting-services)
- Control camera from ROS2 → [Camera Control](guides/CAMERA_CONTROL.md#ros2-control)
- Send detection data → [Target API](api/TARGET_API.md#post-target)
- Stream video → [Media API](api/MEDIA_API.md#get-stream)
- Debug issues → [Setup Guide](guides/SETUP.md#troubleshooting)

**Network Configuration:**
- ZeroTier: Orin `192.168.100.150`, Phone `192.168.100.156`
- T8Space: Orin `172.16.30.234`, Phone `172.16.30.28`
- See `../config/` for network configuration

**ROS2 Topics:**
- `/target_roi` (published) - Detection results
- `/phone_camera/zoom` (subscribed) - Camera zoom
- `/phone_camera/switch` (subscribed) - Camera selection

---

## Document Organization

```
docs/
├── api/              # API references
│   ├── TARGET_API.md
│   ├── MEDIA_API.md
│   └── CAMERA_RELAY.md
├── guides/           # User guides
│   ├── QUICKSTART.md
│   ├── SETUP.md
│   ├── CAMERA_CONTROL.md
│   ├── LISTENER.md
│   └── ROI_UPDATE.md
└── reference/        # Technical specs
    ├── ARCHITECTURE.md
    ├── WS_FORMAT.md
    ├── DIRECTORY.md
    └── TEST_RESULTS.md
```

---

Return to [Orin README](../README.md) | Project [Root README](../../README.md)
