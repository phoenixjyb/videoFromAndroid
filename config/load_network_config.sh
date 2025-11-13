#!/bin/bash
# Network Configuration Helper for Orin Services
#
# Usage:
#   source load_network_config.sh [network_name]
#   
# Sets environment variables based on network preset:
#   - zerotier (default)
#   - t8space
#
# Example:
#   source load_network_config.sh zerotier
#   echo "Phone IP: $PHONE_IP"

# Get network name from argument or environment variable, default to zerotier
NETWORK_CONFIG="${1:-${NETWORK_CONFIG:-zerotier}}"

# Define network presets
case "$NETWORK_CONFIG" in
    zerotier)
        export NETWORK_NAME="ZeroTier"
        export ORIN_IP="192.168.100.150"
        export PHONE_IP="192.168.100.156"
        export TABLET_IP="192.168.100.159"
        ;;
    t8space)
        export NETWORK_NAME="T8Space"
        export ORIN_IP="172.16.30.234"
        export PHONE_IP="172.16.30.28"
        export TABLET_IP="172.16.30.199"
        ;;
    *)
        echo "Error: Unknown network '$NETWORK_CONFIG'"
        echo "Available networks: zerotier, t8space"
        return 1
        ;;
esac

# Service ports (same across all networks)
export PHONE_WS_PORT=9090
export ORIN_TARGET_PORT=8082
export ORIN_MEDIA_PORT=8081

# Derived URLs
export PHONE_VIDEO_URL="ws://${PHONE_IP}:${PHONE_WS_PORT}/"
export PHONE_CONTROL_URL="ws://${PHONE_IP}:${PHONE_WS_PORT}/control"
export ORIN_TARGET_URL="http://${ORIN_IP}:${ORIN_TARGET_PORT}"
export ORIN_MEDIA_URL="http://${ORIN_IP}:${ORIN_MEDIA_PORT}"

# Export NETWORK_CONFIG for downstream scripts
export NETWORK_CONFIG

# Print loaded configuration (if not sourced silently)
if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
    echo "Network Configuration: $NETWORK_NAME ($NETWORK_CONFIG)"
    echo "  Orin:   $ORIN_IP"
    echo "  Phone:  $PHONE_IP (WebSocket: $PHONE_WS_PORT)"
    echo "  Tablet: $TABLET_IP"
    echo ""
    echo "URLs:"
    echo "  Phone Video:   $PHONE_VIDEO_URL"
    echo "  Phone Control: $PHONE_CONTROL_URL"
    echo "  Orin Target:   $ORIN_TARGET_URL"
    echo "  Orin Media:    $ORIN_MEDIA_URL"
fi
