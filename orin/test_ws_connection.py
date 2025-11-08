#!/usr/bin/env python3
"""
Quick test to see if WebSocket is receiving video frames
"""
import asyncio
import websockets
import sys

async def test_connection():
    url = "ws://127.0.0.1:9100/control"
    print(f"Connecting to {url}...")
    
    text_count = 0
    binary_count = 0
    total_bytes = 0
    
    try:
        async with websockets.connect(url, open_timeout=5) as ws:
            print("✓ Connected!")
            print("Listening for messages (10 seconds)...\n")
            
            for i in range(20):  # 20 iterations, 0.5s each = 10 seconds
                try:
                    msg = await asyncio.wait_for(ws.recv(), timeout=0.5)
                    
                    if isinstance(msg, (bytes, bytearray)):
                        binary_count += 1
                        total_bytes += len(msg)
                        print(f"  Binary frame #{binary_count}: {len(msg)} bytes")
                    else:
                        text_count += 1
                        print(f"  Text message #{text_count}: {msg[:100]}")
                        
                except asyncio.TimeoutError:
                    print("  .", end="", flush=True)
                    continue
                    
    except Exception as e:
        print(f"\n✗ Error: {e}")
        return False
    
    print(f"\n\n=== Summary ===")
    print(f"Text messages: {text_count}")
    print(f"Binary frames: {binary_count}")
    print(f"Total video data: {total_bytes / 1024:.1f} KB")
    
    if binary_count == 0:
        print("\n⚠ WARNING: No video frames received!")
        print("The camera might not be streaming.")
        return False
    else:
        print(f"\n✓ Video is streaming ({binary_count} frames in 10 seconds)")
        return True

if __name__ == "__main__":
    result = asyncio.run(test_connection())
    sys.exit(0 if result else 1)
