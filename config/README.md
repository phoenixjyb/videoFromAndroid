# Network Configuration

This directory contains network configuration presets for different network environments.

## Available Networks

### ZeroTier (default)
Private VPN network for remote access
- Orin: `192.168.100.150`
- Phone (S23+): `192.168.100.156`
- Tablet: `192.168.100.159`

### T8Space
Local WiFi hotspot network
- Orin: `172.16.30.234`
- Phone (S23+): `172.16.30.28`
- Tablet: `172.16.30.199`

## Configuration Files

- `zerotier.env` - ZeroTier network configuration
- `t8space.env` - T8Space network configuration
- `network.env` - Active configuration (symlink or copy)

## Usage

### On Orin (Python scripts)

```bash
# Set network environment
export NETWORK_CONFIG=zerotier  # or t8space

# Scripts will automatically load the correct IPs
./start_all_services.sh
```

### On Android (CamViewer)

Use the network selector in Settings screen to switch between presets.

### In Documentation

Examples use ZeroTier IPs by default, with T8Space IPs noted as alternatives.
