#!/usr/bin/env python3

import asyncio
import websockets
import json
import os

async def test_commands():
    try:
        # Bypass proxy for localhost
        # Bypass proxies for localhost
        os.environ['no_proxy'] = 'localhost,127.0.0.1,*'
        os.environ['NO_PROXY'] = 'localhost,127.0.0.1,*'
        for k in ['http_proxy','https_proxy','HTTP_PROXY','HTTPS_PROXY','all_proxy','ALL_PROXY']:
            if k in os.environ:
                del os.environ[k]
        
        uri = "ws://localhost:9090/control"
        async with websockets.connect(uri) as websocket:
            print("✅ Connected to WebSocket")
            
            # Test zoom command
            zoom_command = {
                "cmd": "setZoomRatio",
                "value": 2.0
            }
            print(f"📤 Sending zoom command: {zoom_command}")
            await websocket.send(json.dumps(zoom_command))
            
            # Wait a bit
            await asyncio.sleep(1)
            
            # Test camera switch command
            camera_command = {
                "cmd": "switchCamera",
                "facing": "front"
            }
            print(f"📤 Sending camera switch command: {camera_command}")
            await websocket.send(json.dumps(camera_command))
            
            # Wait a bit more
            await asyncio.sleep(1)
            print("✅ Commands sent successfully")
            
    except Exception as e:
        print(f"❌ Error: {e}")

if __name__ == "__main__":
    asyncio.run(test_commands())
