#!/usr/bin/env python3
"""
ROS2 Camera Control Relay

Subscribes to ROS2 camera control topics and forwards commands to the phone's
camControl app via WebSocket. This allows downstream ROS2 nodes to control
the camera by publishing to standard ROS2 topics.

Control Topics:
- /recomo/film/zoom (std_msgs/Float32): Zoom ratio (1.0 - 10.0)
- /recomo/film/ae_lock (std_msgs/Bool): Auto Exposure lock
- /recomo/film/awb_lock (std_msgs/Bool): Auto White Balance lock
- /recomo/film/switch (std_msgs/String): Camera facing ("back" or "front")
- /recomo/film/bitrate (std_msgs/Int32): Bitrate in bits per second
- /recomo/film/codec (std_msgs/String): Codec ("h264" or "h265")
- /recomo/film/key_frame (std_msgs/Empty): Request key frame

Usage:
    python3 camera_control_relay.py --phone-host 172.16.30.28
"""

import argparse
import asyncio
import json
import logging
import signal
import sys
import threading
from queue import Queue
from typing import Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Bool, String, Int32, Empty
import websockets

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)


class CameraControlRelay(Node):
    """
    ROS2 node that relays camera control commands from ROS2 topics
    to the phone's camControl app via WebSocket
    """
    
    def __init__(self, phone_host: str, phone_port: int = 9090):
        super().__init__('camera_control_relay')
        
        self.phone_host = phone_host
        self.phone_port = phone_port
        self.ws_uri = f'ws://{phone_host}:{phone_port}/control'
        
        # Command queue for thread-safe communication
        self.command_queue = Queue()
        
        # Start async worker thread
        self.async_thread = threading.Thread(target=self._async_worker, daemon=True)
        self.async_thread.start()
        
        self.get_logger().info(f'Camera Control Relay initialized')
        self.get_logger().info(f'Phone WebSocket: {self.ws_uri}')
        
        # Create subscriptions for camera control topics
        self.zoom_sub = self.create_subscription(
            Float32,
            '/recomo/film/zoom',
            self._zoom_callback,
            10
        )
        
        self.ae_lock_sub = self.create_subscription(
            Bool,
            '/recomo/film/ae_lock',
            self._ae_lock_callback,
            10
        )
        
        self.awb_lock_sub = self.create_subscription(
            Bool,
            '/recomo/film/awb_lock',
            self._awb_lock_callback,
            10
        )
        
        self.switch_sub = self.create_subscription(
            String,
            '/recomo/film/switch',
            self._switch_callback,
            10
        )
        
        self.bitrate_sub = self.create_subscription(
            Int32,
            '/recomo/film/bitrate',
            self._bitrate_callback,
            10
        )
        
        self.codec_sub = self.create_subscription(
            String,
            '/recomo/film/codec',
            self._codec_callback,
            10
        )
        
        self.key_frame_sub = self.create_subscription(
            Empty,
            '/recomo/film/key_frame',
            self._key_frame_callback,
            10
        )
        
        self.get_logger().info('✅ All camera control topic subscriptions created')
    
    def _async_worker(self):
        """Worker thread that runs async event loop"""
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        loop.run_until_complete(self._process_commands())
    
    async def _process_commands(self):
        """Process commands from queue and send via WebSocket"""
        while True:
            # Wait for command from queue
            await asyncio.sleep(0.01)  # Small delay to prevent busy waiting
            
            if not self.command_queue.empty():
                command = self.command_queue.get()
                await self._send_command(command)
    
    async def _send_command(self, command: dict):
        """Send command to phone via WebSocket"""
        try:
            async with websockets.connect(self.ws_uri, ping_interval=None) as websocket:
                command_json = json.dumps(command)
                await websocket.send(command_json)
                self.get_logger().info(f'📤 Sent command: {command_json}')
        except Exception as e:
            self.get_logger().error(f'Failed to send command: {e}')
    
    def _zoom_callback(self, msg: Float32):
        """Handle zoom command"""
        self.get_logger().info(f'🔍 Zoom: {msg.data}x')
        command = {
            'cmd': 'setZoomRatio',
            'value': float(msg.data)
        }
        self.command_queue.put(command)
    
    def _ae_lock_callback(self, msg: Bool):
        """Handle AE lock command"""
        self.get_logger().info(f'☀️ AE Lock: {msg.data}')
        command = {
            'cmd': 'setAeLock',
            'value': bool(msg.data)
        }
        self.command_queue.put(command)
    
    def _awb_lock_callback(self, msg: Bool):
        """Handle AWB lock command"""
        self.get_logger().info(f'📷 AWB Lock: {msg.data}')
        command = {
            'cmd': 'setAwbLock',
            'value': bool(msg.data)
        }
        self.command_queue.put(command)
    
    def _switch_callback(self, msg: String):
        """Handle camera switch command"""
        self.get_logger().info(f'🔄 Switch camera: {msg.data}')
        command = {
            'cmd': 'switchCamera',
            'facing': msg.data
        }
        self.command_queue.put(command)
    
    def _bitrate_callback(self, msg: Int32):
        """Handle bitrate command"""
        self.get_logger().info(f'📊 Bitrate: {msg.data / 1000000}Mbps')
        command = {
            'cmd': 'setBitrate',
            'bitrate': int(msg.data)
        }
        self.command_queue.put(command)
    
    def _codec_callback(self, msg: String):
        """Handle codec command"""
        self.get_logger().info(f'🎬 Codec: {msg.data}')
        command = {
            'cmd': 'setCodec',
            'codec': msg.data
        }
        self.command_queue.put(command)
    
    def _key_frame_callback(self, msg: Empty):
        """Handle key frame request"""
        self.get_logger().info(f'🔑 Request key frame')
        command = {
            'cmd': 'requestKeyFrame'
        }
        self.command_queue.put(command)


def main():
    parser = argparse.ArgumentParser(description='ROS2 Camera Control Relay')
    parser.add_argument('--phone-host', type=str, required=True,
                       help='Phone IP address (e.g., 172.16.30.28)')
    parser.add_argument('--phone-port', type=int, default=9090,
                       help='Phone WebSocket port (default: 9090)')
    args = parser.parse_args()
    
    # Initialize ROS2
    rclpy.init()
    
    # Create node
    node = CameraControlRelay(args.phone_host, args.phone_port)
    
    # Handle shutdown gracefully
    def signal_handler(sig, frame):
        logger.info('Shutting down camera control relay...')
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(0)
    
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    logger.info('🚀 Camera Control Relay started')
    logger.info('📡 Listening for camera control commands on ROS2 topics...')
    logger.info('   /recomo/film/zoom (Float32)')
    logger.info('   /recomo/film/ae_lock (Bool)')
    logger.info('   /recomo/film/awb_lock (Bool)')
    logger.info('   /recomo/film/switch (String)')
    logger.info('   /recomo/film/bitrate (Int32)')
    logger.info('   /recomo/film/codec (String)')
    logger.info('   /recomo/film/key_frame (Empty)')
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
