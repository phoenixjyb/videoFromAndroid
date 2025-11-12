#!/usr/bin/env python3
"""
✅ RECOMMENDED: Record video using Android's MediaMuxer (accurate timestamps).

This script triggers on-device MP4 recording using MediaMuxer, which produces videos
with ACCURATE duration and timestamps (5s recording → ~4.3s video, accounting for
start/stop latency). Automatically configures encoder settings and pulls files.

Features:
    - Accurate timestamps (no compression issues)
    - Automatic file retrieval from device
    - Configurable encoder (codec, bitrate, resolution, fps, zoom)
    - Timestamped filenames with metadata
    - Video info display (duration, fps, frames)

Usage:
    python scripts/record_on_device.py -d 5                          # Quick 5s recording
    python scripts/record_on_device.py -d 10 -c h265 -b 8000000      # Custom quality
    python scripts/record_on_device.py -d 30 --profile 3840x2160@30 -z 2.0  # 4K with zoom

Why use this instead of WebSocket streaming?
    - WebSocket streaming has timestamp compression (5s → ~1-2s video)
    - MediaMuxer produces correct timestamps directly from encoder
    - More reliable and predictable results

See also:
    - archive/record_on_device_simple.py: Simpler version without encoder config/auto-pull
    - record_video.py: WebSocket streaming (not recommended)
    - scripts/README.md: Detailed comparison
"""

import asyncio
import websockets
import json
import argparse
import sys
import subprocess
import os
from pathlib import Path
from datetime import datetime
from typing import Optional

# ANSI colors
GREEN = '\033[92m'
YELLOW = '\033[93m'
RED = '\033[91m'
BLUE = '\033[94m'
CYAN = '\033[96m'
RESET = '\033[0m'

def disable_proxies():
    """Disable proxy settings for localhost connections"""
    os.environ['no_proxy'] = 'localhost,127.0.0.1,*'
    os.environ['NO_PROXY'] = 'localhost,127.0.0.1,*'
    for k in ['http_proxy','https_proxy','HTTP_PROXY','HTTPS_PROXY','all_proxy','ALL_PROXY']:
        os.environ.pop(k, None)

class MuxerRecorder:
    def __init__(self, host, port, output_dir):
        self.host = host
        self.port = port
        self.output_dir = Path(output_dir)
        self.output_dir.mkdir(exist_ok=True)
        self.ws = None
        
    async def connect(self):
        """Connect to WebSocket server"""
        uri = f"ws://{self.host}:{self.port}/control"
        print(f"{BLUE}🔌 Connecting to {uri}...{RESET}")
        disable_proxies()
        self.ws = await websockets.connect(uri, open_timeout=10)
        print(f"{GREEN}✅ Connected{RESET}\n")
        
    async def send_command(self, cmd, **kwargs):
        """Send JSON command to server"""
        msg = {"cmd": cmd, **kwargs}
        await self.ws.send(json.dumps(msg))
        print(f"{CYAN}📤 {cmd}: {kwargs}{RESET}")
        
    async def configure_encoder(self, codec, bitrate, profile, zoom):
        """Configure video encoder settings"""
        print(f"{YELLOW}⚙️  Configuring encoder...{RESET}")
        
        # Set zoom first (before encoder restart)
        if zoom:
            await self.send_command("setZoomRatio", value=zoom)
            await asyncio.sleep(0.3)
        
        # Set codec
        await self.send_command("setCodec", codec=codec)
        await asyncio.sleep(0.5)
        
        # Set video profile (parse: "1920x1080@30")
        if profile:
            try:
                resolution, fps_str = profile.split('@')
                width, height = map(int, resolution.split('x'))
                fps_val = int(fps_str)
                await self.send_command("setVideoProfile", width=width, height=height, fps=fps_val)
                await asyncio.sleep(0.5)
            except ValueError:
                print(f"{RED}❌ Invalid profile format: {profile}{RESET}")
                print(f"{YELLOW}Expected format: WIDTHxHEIGHT@FPS (e.g., 1920x1080@30){RESET}")
                raise
        
        # Set bitrate
        if bitrate:
            await self.send_command("setBitrate", bitrate=bitrate)
            await asyncio.sleep(0.5)
        
        # Wait for encoder to stabilize
        print(f"{YELLOW}⏳ Waiting for encoder to stabilize (3 seconds)...{RESET}")
        await asyncio.sleep(3.0)
        
        print(f"{GREEN}✅ Encoder ready: {codec.upper()}, {profile}, {bitrate // 1000000}Mbps{', Zoom: ' + str(zoom) + 'x' if zoom else ''}{RESET}\n")
        
    def generate_filename(self, codec, bitrate, profile, duration):
        """Generate timestamped filename matching record_video.py format"""
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        profile_clean = profile.replace('@', '_') + "fps" if profile else "default"
        bitrate_mb = f"{bitrate // 1000000}Mbps" if bitrate else "auto"
        filename = f"{timestamp}_{codec}_{profile_clean}_{bitrate_mb}_{duration}s"
        return filename
        
    async def record(self, duration, filename_base):
        """Start recording on device, wait, then stop"""
        print(f"{YELLOW}🎥 Starting recording: {filename_base}{RESET}")
        
        # Start recording on device
        await self.send_command("startRecording", name=filename_base)
        
        # Wait for specified duration
        print(f"{CYAN}⏱️  Recording for {duration} seconds...{RESET}")
        await asyncio.sleep(duration)
        
        # Stop recording
        await self.send_command("stopRecording")
        print(f"{GREEN}✅ Recording stopped{RESET}\n")
        
def pull_file(filename_base: str, destination: Path) -> Optional[Path]:
    """Pull the recorded file from device using adb."""
    remote_dir = "/sdcard/Android/data/com.example.camcontrol/files/Movies/"
    
    # Sanitize filename same way as VideoRecorder.kt: replace non-alphanumeric (except _ and -) with _
    import re
    safe_name = re.sub(r'[^a-zA-Z0-9_-]', '_', filename_base)
    
    # Find the file on device (it will have timestamp suffix)
    result = subprocess.run(
        ["adb", "shell", f"ls {remote_dir}{safe_name}-*.mp4"],
        capture_output=True,
        text=True
    )
    
    if result.returncode != 0:
        print(f"{RED}❌ Could not find recorded file on device{RESET}")
        print(f"{BLUE}💡 Check: adb shell ls {remote_dir}{RESET}")
        return None
    
    device_path = result.stdout.strip()
    local_path = destination / Path(device_path).name
    
    # Pull the file
    result = subprocess.run(
        ['adb', 'pull', device_path, str(local_path)],
        capture_output=True,
        text=True
    )
    
    if result.returncode != 0:
        print(f"{RED}❌ Failed to pull file: {result.stderr}{RESET}")
        return None
        
    print(f"{GREEN}✅ File saved: {local_path.name}{RESET}")
    return local_path

def check_video_info(video_path):
    """Display video information using ffprobe"""
    print(f"\n{BLUE}📊 Video Information:{RESET}")
    
    result = subprocess.run(
        ['ffprobe', '-v', 'error', 
         '-show_entries', 'format=duration:stream=r_frame_rate,avg_frame_rate,nb_frames,width,height',
         '-of', 'default=noprint_wrappers=1',
         str(video_path)],
        capture_output=True,
        text=True
    )
    
    if result.returncode == 0:
        for line in result.stdout.strip().split('\n'):
            if '=' in line:
                key, value = line.split('=', 1)
                if key == 'duration':
                    print(f"   Duration: {float(value):.2f}s")
                elif key == 'r_frame_rate':
                    num, den = value.split('/')
                    fps = int(num) / int(den)
                    print(f"   Frame rate: {fps:.1f} fps")
                elif key == 'nb_frames':
                    print(f"   Total frames: {value}")
                elif key == 'width':
                    print(f"   Resolution: {value}", end='')
                elif key == 'height':
                    print(f"x{value}")
        else:
            print(f"{YELLOW}   (ffprobe not available or failed){RESET}")

async def main(args):
    recorder = MuxerRecorder(args.host, args.port, args.output)
    
    try:
        # Connect to WebSocket
        await recorder.connect()
        
        # Configure encoder if not skipped
        if not args.no_config:
            await recorder.configure_encoder(
                args.codec,
                args.bitrate,
                args.profile,
                args.zoom
            )
        else:
            print(f"{YELLOW}⚠️  Skipping encoder reconfiguration (using current settings){RESET}\n")
            if args.zoom:
                await recorder.send_command("setZoomRatio", value=args.zoom)
                print(f"{CYAN}🔍 Zoom set to {args.zoom}x{RESET}\n")
        
        # Generate filename
        filename_base = recorder.generate_filename(
            args.codec,
            args.bitrate,
            args.profile,
            args.duration
        )
        
        # Record
        await recorder.record(args.duration, filename_base)
        
        # Wait a moment for file to be fully written
        await asyncio.sleep(0.5)
        
    except websockets.exceptions.WebSocketException as e:
        print(f"{RED}❌ WebSocket error: {e}{RESET}")
        print(f"{YELLOW}💡 Make sure the Android app is running and reachable at {args.host}:{args.port}{RESET}")
        sys.exit(1)
    except KeyboardInterrupt:
        print(f"\n{YELLOW}⚠️  Recording interrupted by user{RESET}")
        sys.exit(130)
    finally:
        if recorder.ws:
            await recorder.ws.close()
    
    # Pull file from device
    output_dir = Path(__file__).parent.parent / 'saved_videos'
    video_path = pull_file(filename_base, output_dir)
    
    if video_path and video_path.exists():
        # Show video info
        check_video_info(video_path)
        print(f"\n{GREEN}✅ Recording complete!{RESET}")
        print(f"{CYAN}   File: {video_path}{RESET}")
    else:
        print(f"\n{RED}❌ Recording may have failed{RESET}")
        sys.exit(1)

if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        description='Record video using Android MediaMuxer (produces correct timestamps)',
        formatter_class=argparse.RawDescriptionHelpFormatter
    )
    
    # Connection settings
    parser.add_argument('-H', '--host', default='localhost',
                        help='WebSocket host (default: localhost)')
    parser.add_argument('-p', '--port', type=int, default=9091,
                        help='WebSocket port (default: 9091)')
    
    # Recording settings
    parser.add_argument('-d', '--duration', type=float, default=5.0,
                        help='Recording duration in seconds (default: 5)')
    parser.add_argument('-o', '--output', default='saved_videos',
                        help='Output directory (default: saved_videos)')
    
    # Encoder settings
    parser.add_argument('-c', '--codec', choices=['h264', 'h265'], default='h265',
                        help='Video codec (default: h265)')
    parser.add_argument('-b', '--bitrate', type=int, default=5000000,
                        help='Bitrate in bps (default: 5000000 = 5Mbps)')
    parser.add_argument('--profile', default='1920x1080@30',
                        help='Video profile as WIDTHxHEIGHT@FPS (default: 1920x1080@30)')
    parser.add_argument('-z', '--zoom', type=float, default=None,
                        help='Zoom ratio (e.g., 2.0 for 2x zoom)')
    
    # Advanced options
    parser.add_argument('--no-config', action='store_true',
                        help='Skip encoder reconfiguration (use current settings)')
    
    args = parser.parse_args()
    
    try:
        asyncio.run(main(args))
    except KeyboardInterrupt:
        print(f"\n{YELLOW}⚠️  Interrupted{RESET}")
        sys.exit(130)
