#!/usr/bin/env python3
"""
Network configuration loader for CamControl system.
Supports multiple network presets (zerotier, t8space).
"""

import os
from typing import Dict, Optional
from pathlib import Path


class NetworkConfig:
    """Network configuration for CamControl system"""
    
    # Default to ZeroTier
    DEFAULT_NETWORK = "zerotier"
    
    # Network presets
    PRESETS = {
        "zerotier": {
            "name": "ZeroTier",
            "orin_ip": "192.168.100.150",
            "phone_ip": "192.168.100.156",
            "tablet_ip": "192.168.100.159",
        },
        "t8space": {
            "name": "T8Space",
            "orin_ip": "172.16.30.234",
            "phone_ip": "172.16.30.28",
            "tablet_ip": "172.16.30.199",
        }
    }
    
    # Service ports (same across all networks)
    PHONE_WS_PORT = 9090
    ORIN_TARGET_PORT = 8082
    ORIN_MEDIA_PORT = 8081
    
    def __init__(self, network: Optional[str] = None):
        """
        Initialize network configuration.
        
        Args:
            network: Network preset name ("zerotier" or "t8space").
                    If None, reads from NETWORK_CONFIG env var or uses default.
        """
        if network is None:
            network = os.getenv("NETWORK_CONFIG", self.DEFAULT_NETWORK)
        
        if network not in self.PRESETS:
            raise ValueError(f"Unknown network: {network}. Available: {list(self.PRESETS.keys())}")
        
        self.network = network
        self._preset = self.PRESETS[network]
    
    @property
    def network_name(self) -> str:
        """Human-readable network name"""
        return self._preset["name"]
    
    @property
    def orin_ip(self) -> str:
        """Orin device IP address"""
        return self._preset["orin_ip"]
    
    @property
    def phone_ip(self) -> str:
        """Phone (S23+) IP address"""
        return self._preset["phone_ip"]
    
    @property
    def tablet_ip(self) -> str:
        """Tablet IP address"""
        return self._preset["tablet_ip"]
    
    @property
    def phone_video_url(self) -> str:
        """Phone WebSocket video stream URL"""
        return f"ws://{self.phone_ip}:{self.PHONE_WS_PORT}/"
    
    @property
    def phone_control_url(self) -> str:
        """Phone WebSocket control URL"""
        return f"ws://{self.phone_ip}:{self.PHONE_WS_PORT}/control"
    
    @property
    def orin_target_url(self) -> str:
        """Orin target API URL"""
        return f"http://{self.orin_ip}:{self.ORIN_TARGET_PORT}"
    
    @property
    def orin_media_url(self) -> str:
        """Orin media API URL"""
        return f"http://{self.orin_ip}:{self.ORIN_MEDIA_PORT}"
    
    def to_dict(self) -> Dict[str, str]:
        """Export configuration as dictionary"""
        return {
            "network": self.network,
            "network_name": self.network_name,
            "orin_ip": self.orin_ip,
            "phone_ip": self.phone_ip,
            "tablet_ip": self.tablet_ip,
            "phone_ws_port": str(self.PHONE_WS_PORT),
            "orin_target_port": str(self.ORIN_TARGET_PORT),
            "orin_media_port": str(self.ORIN_MEDIA_PORT),
            "phone_video_url": self.phone_video_url,
            "phone_control_url": self.phone_control_url,
            "orin_target_url": self.orin_target_url,
            "orin_media_url": self.orin_media_url,
        }
    
    def __str__(self) -> str:
        """String representation"""
        return (
            f"Network: {self.network_name} ({self.network})\n"
            f"  Orin:   {self.orin_ip}\n"
            f"  Phone:  {self.phone_ip}\n"
            f"  Tablet: {self.tablet_ip}\n"
            f"  Phone WS:     {self.phone_video_url}\n"
            f"  Orin Target:  {self.orin_target_url}\n"
            f"  Orin Media:   {self.orin_media_url}"
        )


def load_config(network: Optional[str] = None) -> NetworkConfig:
    """
    Load network configuration.
    
    Args:
        network: Network preset name. If None, uses NETWORK_CONFIG env var or default.
    
    Returns:
        NetworkConfig instance
    
    Example:
        >>> config = load_config("zerotier")
        >>> print(config.phone_ip)
        192.168.100.156
        
        >>> config = load_config()  # Uses NETWORK_CONFIG env or default
        >>> print(config.orin_target_url)
        http://192.168.100.150:8082
    """
    return NetworkConfig(network)


if __name__ == "__main__":
    import sys
    
    # CLI for testing configuration
    if len(sys.argv) > 1:
        network = sys.argv[1]
    else:
        network = os.getenv("NETWORK_CONFIG", NetworkConfig.DEFAULT_NETWORK)
    
    try:
        config = load_config(network)
        print(config)
        print("\nEnvironment variables:")
        for key, value in config.to_dict().items():
            print(f"  {key.upper()}={value}")
    except ValueError as e:
        print(f"Error: {e}", file=sys.stderr)
        print(f"Available networks: {', '.join(NetworkConfig.PRESETS.keys())}", file=sys.stderr)
        sys.exit(1)
