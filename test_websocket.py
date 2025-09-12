#!/usr/bin/env python3
"""
Simple WebSocket client to test the Android app's WebSocket server
"""
import asyncio
import websockets
import json
import os

async def test_connection():
    # Bypass proxies for localhost
    os.environ['no_proxy'] = 'localhost,127.0.0.1,*'
    os.environ['NO_PROXY'] = 'localhost,127.0.0.1,*'
    for k in ['http_proxy','https_proxy','HTTP_PROXY','HTTPS_PROXY','all_proxy','ALL_PROXY']:
        if k in os.environ:
            del os.environ[k]

    uri = "ws://127.0.0.1:9090/control"
    try:
        print(f"Connecting to {uri}...")
        async with websockets.connect(uri) as websocket:
            print("✅ Connected successfully!")
            
            # Send a test command
            test_command = {"cmd": "setZoomRatio", "value": 1.5}
            await websocket.send(json.dumps(test_command))
            print(f"📤 Sent: {test_command}")
            
            # Listen for responses for 5 seconds
            try:
                while True:
                    response = await asyncio.wait_for(websocket.recv(), timeout=5.0)
                    if isinstance(response, bytes):
                        print(f"📥 Received binary data: {len(response)} bytes")
                    else:
                        print(f"📥 Received text: {response}")
            except asyncio.TimeoutError:
                print("⏰ Timeout waiting for messages")
                
    except Exception as e:
        print(f"❌ Connection failed: {e}")

if __name__ == "__main__":
    asyncio.run(test_connection())
