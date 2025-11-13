# Documentation Archive

**Created**: November 13, 2025  
**Purpose**: Historical documents from project development phases

## Archive Organization

### `/camviewer_development/` - CamViewer Planning & Implementation

These documents were created during CamViewer development (November 11-12, 2025) and are now superseded by the operational system documentation.

| File | Date | Purpose | Status |
|------|------|---------|--------|
| `CAMVIEWER_CLARIFICATIONS.md` | Nov 11 | Early architecture clarifications | ✅ Incorporated into SYSTEM_ARCHITECTURE.md |
| `CAMVIEWER_IMPLEMENTATION_PLAN.md` | Nov 11 | Detailed implementation plan (714 lines) | ✅ All phases complete |
| `CAMVIEWER_SUMMARY.md` | Nov 11 | Quick overview of CamViewer features | ✅ Superseded by main docs |
| `PHASE4_MEDIA_RETRIEVAL_SUMMARY.md` | Nov 12 | Phase 4 media gallery implementation | ✅ Feature complete and documented |

**Why Archived**: CamViewer is now fully operational. All architectural decisions and implementation details have been incorporated into:
- `SYSTEM_ARCHITECTURE.md` - High-level overview
- `THREE_WAY_CAMERA_CONTROL.md` - Control implementation
- `WEBSOCKET_ARCHITECTURE.md` - Protocol details

### `/project_history/` - Project Status & Deployment History

Historical documents tracking project evolution and resolved issues.

| File | Date | Purpose | Status |
|------|------|---------|--------|
| `DEPLOYMENT_CHECKLIST.md` | Nov 12 | Port configuration issues (8080 vs 9090) | ✅ Issues resolved, port 9090 standardized |
| `PROJECT_STATUS_SUMMARY.md` | Sep 12 | Project status with blockers | ✅ All blockers cleared |
| `ARCHITECTURE_DIAGRAMS.md` | Oct 2025 | Old architecture diagrams | ✅ Superseded by consolidated docs |
| `ARCHITECTURE_UPDATES_2025-11-11.md` | Nov 11 | Temporary update notes | ✅ Changes incorporated |
| `ARCHITECTURE_VALIDATION.md` | Nov 11 | Architecture validation notes | ✅ System validated |

**Why Archived**: These documents tracked issues that have been resolved:
- Port standardization complete (9090 for phone, 8081/8082 for Orin)
- Three-way control operational (WebUI, CamViewer, ROS2)
- All deployment blockers cleared

## Active Documentation

For current system information, see:

### Core Architecture
- **[SYSTEM_ARCHITECTURE.md](../SYSTEM_ARCHITECTURE.md)** - Master overview
- **[WEBSOCKET_ARCHITECTURE.md](../WEBSOCKET_ARCHITECTURE.md)** - WebSocket protocol specs
- **[ROS2_TOPICS_REFERENCE.md](../ROS2_TOPICS_REFERENCE.md)** - ROS2 topics documentation
- **[NETWORK_PROTOCOLS.md](../NETWORK_PROTOCOLS.md)** - Network protocols and APIs

### Implementation Guides
- **[THREE_WAY_CAMERA_CONTROL.md](../THREE_WAY_CAMERA_CONTROL.md)** - Three-way control implementation
- **[WIFI_ACCESS.md](../WIFI_ACCESS.md)** - WiFi setup guide
- **[ORIN_TARGET_API.md](../ORIN_TARGET_API.md)** - Target API specification

### Reference
- **[DIARY.md](../DIARY.md)** - Development journal
- **[ProjectSetup.md](../ProjectSetup.md)** - Android Camera2 setup guide

## Historical Value

These archived documents are preserved for:
1. **Development History**: Understanding design evolution and decisions
2. **Implementation Reference**: Detailed phase-by-phase implementation notes
3. **Issue Tracking**: Record of problems encountered and solutions
4. **Future Development**: Reference for similar features or refactoring

---

**Note**: These documents are not maintained. For current information, always refer to the active documentation listed above.
