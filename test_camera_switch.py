#!/usr/bin/env python3

import asyncio
import websockets
import json
import os

async def test_camera_switch():
    try:
        # Bypass proxy for localhost
        os.environ['no_proxy'] = 'localhost,127.0.0.1,*'
        os.environ['NO_PROXY'] = 'localhost,127.0.0.1,*'
        for k in ['http_proxy','https_proxy','HTTP_PROXY','HTTPS_PROXY','all_proxy','ALL_PROXY']:
            if k in os.environ:
                del os.environ[k]
        
        uri = "ws://localhost:9090/control"
        async with websockets.connect(uri) as websocket:
            print("✅ Connected to WebSocket")
            
            # Test camera switch to front
            camera_command = {
                "cmd": "switchCamera",
                "facing": "front"
            }
            print(f"📤 Sending camera switch to front: {camera_command}")
            await websocket.send(json.dumps(camera_command))
            
            # Wait for processing
            await asyncio.sleep(3)
            
            # Test camera switch back to back
            camera_command = {
                "cmd": "switchCamera", 
                "facing": "back"
            }
            print(f"📤 Sending camera switch to back: {camera_command}")
            await websocket.send(json.dumps(camera_command))
            
            # Wait a bit more
            await asyncio.sleep(3)
            print("✅ Camera switch commands sent successfully")
            
    except Exception as e:
        print(f"❌ Error: {e}")

if __name__ == "__main__":
    asyncio.run(test_camera_switch())
