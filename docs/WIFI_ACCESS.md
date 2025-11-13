# Network Access Guide

## Network Configurations

The CamControl system supports two network presets:

### 1. ZeroTier VPN Network (Default)
Private VPN network for remote access over internet:
- **Orin**: `192.168.100.150`
- **Phone (S23+)**: `192.168.100.156`
- **Tablet**: `192.168.100.159`

### 2. T8Space WiFi Hotspot  
Local WiFi network (phone hotspot):
- **Orin**: `172.16.30.234`
- **Phone (S23+)**: `172.16.30.28`
- **Tablet**: `172.16.30.199`

## Quick Start

### On CamViewer (Android Tablet)

1. Open **Settings** in CamViewer
2. Select **Network Preset**:
   - Choose "ZeroTier" for VPN access
   - Choose "T8Space" for local WiFi
3. URLs are automatically configured
4. Tap **Save Settings**

### On Orin (Jetson)

```bash
# Use ZeroTier (default)
./start_all_services.sh

# Or use T8Space
NETWORK_CONFIG=t8space ./start_all_services.sh
```

### On Desktop (WebUI Access)

**ZeroTier Network:**
```
http://192.168.100.156:9090
```

**T8Space Network:**
```
http://172.16.30.28:9090
```

## Step 1: Find Your Phone's IP Address

### Method A: Using ADB
```bash
adb shell ip addr show wlan0 | grep 'inet ' | awk '{print $2}' | cut -d/ -f1
```

### Method B: On Android Phone
1. Open **Settings** → **About phone** → **Status**
2. Look for **IP address** (e.g., `172.16.30.28`)

**OR**

1. Open **Settings** → **Network & Internet** → **Wi-Fi**
2. Tap on your connected network
3. Find **IP address**

### Method C: Quick Command
```bash
# Get phone IP automatically
adb shell "ip -o -4 addr show wlan0 | awk '{print \$4}' | cut -d/ -f1"
```

## Step 2: Access WebUI Wirelessly

Once you have the phone's IP (e.g., `172.16.30.28`):

### In Web Browser
```
http://172.16.30.28:9090
```

### WebSocket Endpoints
- **WebUI**: `http://172.16.30.28:9090/` (main page)
- **Video Stream**: `ws://172.16.30.28:9090/` (WebSocket binary)
- **Control Commands**: `ws://172.16.30.28:9090/control` (WebSocket JSON)

### Update Host in WebUI
1. Open `http://172.16.30.28:9090`
2. In the top bar, the **Host** field shows the current IP
3. WebUI automatically detects and uses the correct IP

## Step 3: Test Connection

```bash
# From your computer, test if port 9090 is reachable
curl http://172.16.30.28:9090/

# Or test WebSocket connectivity
wscat -c ws://172.16.30.28:9090/control
```

## Troubleshooting

### Cannot Connect
1. **Verify same WiFi**: Both devices on same network?
   ```bash
   # Check phone WiFi
   adb shell dumpsys wifi | grep "mNetworkInfo"
   ```

2. **Check phone IP**:
   ```bash
   adb shell ip addr show wlan0
   ```

3. **Test port accessibility**:
   ```bash
   # From computer
   nc -zv 172.16.30.28 9090
   ```

4. **Firewall**: Android may block incoming connections
   - Some phones have built-in firewall
   - Check phone's security settings

### VPN Interference
If you have VPN running:

**On Computer:**
```bash
# Temporarily disable VPN or add exception for local network
# For macOS/Linux, you can bypass proxy:
export NO_PROXY="172.16.0.0/12,192.168.0.0/16,127.0.0.1,localhost"

# Or disable VPN entirely for testing
```

**On Phone:**
- Disable VPN in phone settings during testing
- VPN can block local network access

### Port Forwarding Note
**Do NOT use ADB port forwarding for WiFi access**
```bash
# This is only for USB connection:
# adb forward tcp:9090 tcp:9090  ← NOT needed for WiFi

# For WiFi, access phone IP directly
```

## Quick Reference

| Connection Type | URL | ADB Forward Needed? |
|----------------|-----|---------------------|
| USB (ADB) | `http://localhost:9090` | ✅ Yes |
| WiFi | `http://PHONE_IP:9090` | ❌ No |

## Example Session

```bash
# 1. Get phone IP
PHONE_IP=$(adb shell "ip -o -4 addr show wlan0 | awk '{print \$4}' | cut -d/ -f1")
echo "Phone IP: $PHONE_IP"

# 2. Test connectivity
curl http://$PHONE_IP:9090/

# 3. Open in browser
open http://$PHONE_IP:9090  # macOS
# or
xdg-open http://$PHONE_IP:9090  # Linux
# or manually open in browser on Windows
```

## Network Requirements

- **WiFi**: 2.4GHz or 5GHz (5GHz recommended for better bandwidth)
- **Bandwidth**: 
  - H.264 1080p30: ~10-15 Mbps
  - H.265 1080p30: ~10-15 Mbps (better quality)
  - H.264 4K30: ~30-50 Mbps
- **Latency**: < 50ms on local network (typical: 1-5ms)

## Security Note

The WebSocket server has **NO authentication**. Only use on trusted WiFi networks.

For production use, consider adding:
- Authentication tokens
- HTTPS/WSS (TLS encryption)
- Network ACL (IP whitelisting)
