# Network Configuration Quick Reference

## Active Network IPs

### ZeroTier VPN (Default)
```
Phone:  192.168.100.156:9090  (WebSocket)
Orin:   192.168.100.150:8082  (Target API)
        192.168.100.150:8081  (Media API)
Tablet: 192.168.100.159       (CamViewer)
```

### T8Space WiFi Hotspot
```
Phone:  172.16.30.28:9090     (WebSocket)
Orin:   172.16.30.234:8082    (Target API)
        172.16.30.234:8081    (Media API)
Tablet: 172.16.30.199         (CamViewer)
```

## Switch Network

### On CamViewer (Tablet)
1. Open **Settings**
2. Select **Network Preset** dropdown
3. Choose **ZeroTier** or **T8Space**
4. Tap **Save Settings**
5. Return to main screen and reconnect

### On Orin (Jetson)
```bash
# ZeroTier (default)
cd orin && ./start_all_services.sh

# T8Space
cd orin && NETWORK_CONFIG=t8space ./start_all_services.sh

# Or set permanently
export NETWORK_CONFIG=t8space
./start_all_services.sh
```

### On Desktop (WebUI)
Just use the appropriate URL based on active network:

**ZeroTier:**
- Video: `http://192.168.100.156:9090`
- Control: `ws://192.168.100.156:9090/control`

**T8Space:**
- Video: `http://172.16.30.28:9090`
- Control: `ws://172.16.30.28:9090/control`

## Test Configuration

### Check which network is active
```bash
# On Orin
source config/load_network_config.sh
echo "Active network: $NETWORK_NAME"
echo "Phone IP: $PHONE_IP"
```

### Test connectivity
```bash
# Ping phone
ping -c 3 $PHONE_IP

# Test WebSocket (requires phone app running)
curl http://$PHONE_IP:9090/

# Test Orin services
curl http://localhost:8082/health     # Target API
curl http://localhost:8081/media/     # Media API
```

## Python Scripts

```python
from config.network_config import load_config

# Load default (ZeroTier)
config = load_config()
print(f"Phone: {config.phone_ip}")

# Load specific network
config = load_config("t8space")
print(f"Phone video: {config.phone_video_url}")
print(f"Orin target: {config.orin_target_url}")
```

## Environment Variables

After sourcing `load_network_config.sh`:
```bash
$NETWORK_NAME      # "ZeroTier" or "T8Space"
$PHONE_IP          # Phone IP address
$ORIN_IP           # Orin IP address
$TABLET_IP         # Tablet IP address
$PHONE_WS_PORT     # 9090
$ORIN_TARGET_PORT  # 8082
$ORIN_MEDIA_PORT   # 8081
```

## Troubleshooting

### CamViewer not connecting
1. Check Network Preset in Settings matches active network
2. Verify phone app is running
3. Test: `ping <phone-ip>` from tablet

### Orin services using wrong IPs
1. Check `NETWORK_CONFIG` environment variable
2. Re-source config: `source ../config/load_network_config.sh`
3. Restart services: `./stop_all_services.sh && ./start_all_services.sh`

### Mixed network (some devices on each)
**Won't work!** All devices must be on the same network:
- All on ZeroTier VPN, OR
- All on T8Space WiFi hotspot

Cannot mix devices across networks.
