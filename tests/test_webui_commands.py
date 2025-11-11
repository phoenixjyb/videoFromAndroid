#!/usr/bin/env python3
"""
Test script to verify WebUI commands are working
"""
import asyncio
import websockets
import json
import time

async def test_zoom_command():
    """Test zoom command"""
    try:
        # Connect to WebSocket
        uri = "ws://localhost:9090/control"
        print(f"🔌 Connecting to {uri}...")
        
        # Set up no proxy for websockets (both lower/upper case)
        import os
        old_http_proxy = os.environ.get('http_proxy')
        old_https_proxy = os.environ.get('https_proxy')
        old_HTTP_PROXY = os.environ.get('HTTP_PROXY')
        old_HTTPS_PROXY = os.environ.get('HTTPS_PROXY')
        old_all_proxy = os.environ.get('all_proxy')
        old_ALL_PROXY = os.environ.get('ALL_PROXY')
        old_no_proxy = os.environ.get('no_proxy')
        old_NO_PROXY = os.environ.get('NO_PROXY')

        for k in ['http_proxy','https_proxy','HTTP_PROXY','HTTPS_PROXY','all_proxy','ALL_PROXY']:
            if k in os.environ:
                del os.environ[k]
        os.environ['no_proxy'] = '*'
        os.environ['NO_PROXY'] = '*'
        
        try:
            async with websockets.connect(uri, open_timeout=5) as websocket:
                print("✅ Connected!")
                
                # Test zoom command
                zoom_command = {
                    "cmd": "setZoomRatio",
                    "value": 3.0
                }
                
                print(f"📤 Sending zoom command: {zoom_command}")
                await websocket.send(json.dumps(zoom_command))
                print("✅ Zoom command sent")
                
                # Wait a bit
                await asyncio.sleep(2)
                
                # Test camera switch command
                switch_command = {
                    "cmd": "switchCamera",
                    "facing": "front"
                }
                
                print(f"📤 Sending camera switch command: {switch_command}")
                await websocket.send(json.dumps(switch_command))
                print("✅ Camera switch command sent")
                
                # Wait a bit more
                await asyncio.sleep(2)
                
                # Test another zoom
                zoom_command2 = {
                    "cmd": "setZoomRatio", 
                    "value": 1.5
                }
                
                print(f"📤 Sending second zoom command: {zoom_command2}")
                await websocket.send(json.dumps(zoom_command2))
                print("✅ Second zoom command sent")
                
                # Switch back to rear camera
                switch_command2 = {
                    "cmd": "switchCamera",
                    "facing": "back"
                }
                
                print(f"📤 Sending switch back command: {switch_command2}")
                await websocket.send(json.dumps(switch_command2))
                print("✅ Switch back command sent")
                
                await asyncio.sleep(1)
                print("🎯 All commands sent successfully!")
        finally:
            # Restore proxy settings
            if old_http_proxy:
                os.environ['http_proxy'] = old_http_proxy
            if old_https_proxy:
                os.environ['https_proxy'] = old_https_proxy
            if old_HTTP_PROXY:
                os.environ['HTTP_PROXY'] = old_HTTP_PROXY
            if old_HTTPS_PROXY:
                os.environ['HTTPS_PROXY'] = old_HTTPS_PROXY
            if old_all_proxy:
                os.environ['all_proxy'] = old_all_proxy
            if old_ALL_PROXY:
                os.environ['ALL_PROXY'] = old_ALL_PROXY
            if old_no_proxy is not None:
                os.environ['no_proxy'] = old_no_proxy
            else:
                os.environ.pop('no_proxy', None)
            if old_NO_PROXY is not None:
                os.environ['NO_PROXY'] = old_NO_PROXY
            else:
                os.environ.pop('NO_PROXY', None)
            
    except Exception as e:
        print(f"❌ Error: {e}")

if __name__ == "__main__":
    print("🧪 Testing WebUI commands...")
    asyncio.run(test_zoom_command())
