#!/usr/bin/env python3
"""
Record video from Android camera with customizable quality settings.
Saves H.264/H.265 stream to local Mac with timestamped filename.

Usage:
    python scripts/record_video.py --duration 30 --codec h264 --bitrate 8000000
    python scripts/record_video.py -d 60 -c h265 -b 12000000 --profile 1920x1080@30
"""

import asyncio
import websockets
import json
import argparse
import sys
from pathlib import Path
from datetime import datetime
import signal

# ANSI colors
GREEN = '\033[92m'
YELLOW = '\033[93m'
RED = '\033[91m'
BLUE = '\033[94m'
CYAN = '\033[96m'
RESET = '\033[0m'

class VideoRecorder:
    def __init__(self, host, port, output_dir):
        self.host = host
        self.port = port
        self.output_dir = Path(output_dir)
        self.output_dir.mkdir(exist_ok=True)
        self.ws = None
        self.recording = False
        self.file_handle = None
        self.frame_count = 0
        self.byte_count = 0
        self.start_time = None
        self.got_sps_pps = False
        
    async def connect(self):
        """Connect to WebSocket server"""
        uri = f"ws://{self.host}:{self.port}/control"
        print(f"{BLUE}🔌 Connecting to {uri}...{RESET}")
        self.ws = await websockets.connect(uri, max_size=10 * 1024 * 1024)
        print(f"{GREEN}✅ Connected{RESET}")
        
    async def send_command(self, cmd, **kwargs):
        """Send JSON command to server"""
        msg = {"cmd": cmd, **kwargs}
        await self.ws.send(json.dumps(msg))
        print(f"{CYAN}📤 Sent: {msg}{RESET}")
        
    async def configure_encoder(self, codec, bitrate, profile, fps):
        """Configure video encoder settings"""
        print(f"\n{YELLOW}⚙️  Configuring encoder...{RESET}")
        
        # Set codec (field name: "codec")
        await self.send_command("setCodec", codec=codec)
        await asyncio.sleep(0.5)
        
        # Set video profile (fields: width, height, fps)
        if profile:
            # Parse profile: "1920x1080@30" -> width=1920, height=1080, fps=30
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
        
        # Set bitrate (field name: "bitrate")
        if bitrate:
            await self.send_command("setBitrate", bitrate=bitrate)
            await asyncio.sleep(0.5)
            
        print(f"{GREEN}✅ Encoder configured: {codec.upper()}, {profile}, {bitrate} bps{RESET}")
        
    def generate_filename(self, codec, bitrate, profile, duration):
        """Generate timestamped filename with recording parameters"""
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        # Clean profile string: 1920x1080@30 -> 1920x1080_30fps
        profile_clean = profile.replace('@', '_') + "fps" if profile else "default"
        bitrate_mb = f"{bitrate // 1000000}Mbps" if bitrate else "auto"
        
        ext = "h265" if codec.lower() == "h265" else "h264"
        filename = f"{timestamp}_{codec}_{profile_clean}_{bitrate_mb}_{duration}s.{ext}"
        return self.output_dir / filename
        
    async def record(self, duration, codec, bitrate, profile, fps):
        """Record video stream for specified duration"""
        output_file = self.generate_filename(codec, bitrate, profile, duration)
        
        print(f"\n{BLUE}🎥 Recording to: {output_file.name}{RESET}")
        print(f"{YELLOW}⏱️  Duration: {duration}s{RESET}")
        print(f"{CYAN}📊 Codec: {codec.upper()}, Bitrate: {bitrate} bps, Profile: {profile}{RESET}\n")
        
        self.file_handle = open(output_file, 'wb')
        self.recording = True
        self.start_time = datetime.now()
        self.frame_count = 0
        self.byte_count = 0
        self.got_sps_pps = False
        
        # Start receiving frames
        end_time = datetime.now().timestamp() + duration
        
        try:
            while datetime.now().timestamp() < end_time:
                msg = await asyncio.wait_for(self.ws.recv(), timeout=2.0)
                
                if isinstance(msg, str):
                    # Text message (telemetry/status)
                    continue
                    
                # Binary video frame
                if not self.got_sps_pps:
                    # Check for SPS (0x67) and PPS (0x68) or HEVC VPS/SPS/PPS
                    if codec.lower() == "h264":
                        # Look for SPS (0x00 0x00 0x00 0x01 0x67)
                        if b'\x00\x00\x00\x01\x67' in msg[:50]:
                            self.got_sps_pps = True
                            print(f"{GREEN}✅ Got SPS/PPS (H.264 config){RESET}")
                    else:  # h265
                        # Look for VPS (0x00 0x00 0x00 0x01 0x40)
                        if b'\x00\x00\x00\x01\x40' in msg[:50]:
                            self.got_sps_pps = True
                            print(f"{GREEN}✅ Got VPS/SPS/PPS (H.265 config){RESET}")
                
                if self.got_sps_pps:
                    self.file_handle.write(msg)
                    self.frame_count += 1
                    self.byte_count += len(msg)
                    
                    # Progress update every 2 seconds
                    if self.frame_count % 60 == 0:
                        elapsed = (datetime.now() - self.start_time).total_seconds()
                        remaining = duration - elapsed
                        mbytes = self.byte_count / (1024 * 1024)
                        fps_actual = self.frame_count / elapsed if elapsed > 0 else 0
                        print(f"{CYAN}📹 {self.frame_count} frames, {mbytes:.1f} MB, "
                              f"{fps_actual:.1f} fps, {remaining:.1f}s remaining{RESET}")
                        
        except asyncio.TimeoutError:
            pass
        except Exception as e:
            print(f"{RED}❌ Error during recording: {e}{RESET}")
        finally:
            self.recording = False
            if self.file_handle:
                self.file_handle.close()
                
        elapsed = (datetime.now() - self.start_time).total_seconds()
        mbytes = self.byte_count / (1024 * 1024)
        fps_actual = self.frame_count / elapsed if elapsed > 0 else 0
        
        print(f"\n{GREEN}✅ Recording complete!{RESET}")
        print(f"{CYAN}📊 Stats:{RESET}")
        print(f"   Frames: {self.frame_count}")
        print(f"   Size: {mbytes:.2f} MB")
        print(f"   Duration: {elapsed:.2f}s")
        print(f"   Avg FPS: {fps_actual:.1f}")
        print(f"   File: {output_file}{RESET}\n")
        
        # Convert to MP4 if possible
        await self.convert_to_mp4(output_file, codec)
        
    async def convert_to_mp4(self, raw_file, codec):
        """Convert raw H.264/H.265 to MP4 container using ffmpeg"""
        mp4_file = raw_file.with_suffix('.mp4')
        
        print(f"{YELLOW}🔄 Converting to MP4...{RESET}")
        
        codec_name = 'hevc' if codec.lower() == 'h265' else 'h264'
        cmd = [
            'ffmpeg', '-hide_banner', '-loglevel', 'error',
            '-f', codec_name, '-i', str(raw_file),
            '-c', 'copy', '-movflags', '+faststart',
            str(mp4_file)
        ]
        
        try:
            proc = await asyncio.create_subprocess_exec(
                *cmd,
                stdout=asyncio.subprocess.PIPE,
                stderr=asyncio.subprocess.PIPE
            )
            stdout, stderr = await proc.communicate()
            
            if proc.returncode == 0:
                print(f"{GREEN}✅ MP4 created: {mp4_file.name}{RESET}")
                print(f"{YELLOW}💡 You can delete the raw .{codec_name} file if not needed{RESET}")
            else:
                print(f"{RED}❌ ffmpeg failed: {stderr.decode()}{RESET}")
                print(f"{YELLOW}💡 Install ffmpeg: brew install ffmpeg{RESET}")
        except FileNotFoundError:
            print(f"{YELLOW}⚠️  ffmpeg not found. Install with: brew install ffmpeg{RESET}")
            print(f"{YELLOW}💡 Raw {codec_name} file saved: {raw_file.name}{RESET}")
        
    async def cleanup(self):
        """Cleanup resources"""
        if self.recording and self.file_handle:
            self.file_handle.close()
        if self.ws:
            await self.ws.close()

async def main():
    parser = argparse.ArgumentParser(
        description='Record video from Android camera with custom quality settings',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Record 30s H.264 video at 8 Mbps, 1920x1080@30fps
  python scripts/record_video.py -d 30 -c h264 -b 8000000 -p 1920x1080@30
  
  # Record 60s H.265 video at 12 Mbps, 3840x2160@30fps  
  python scripts/record_video.py -d 60 -c h265 -b 12000000 -p 3840x2160@30
  
  # Record with default settings
  python scripts/record_video.py -d 10

Video profiles available:
  1920x1080@60, 1920x1080@30, 3840x2160@30, 1280x720@60, etc.
  
Output:
  Files saved to: saved_videos/
  Format: YYYYMMDD_HHMMSS_codec_resolution_bitrate_duration.{h264|h265|mp4}
        """
    )
    
    parser.add_argument('-H', '--host', default='172.16.31.5',
                        help='Android device IP address (default: 172.16.31.5)')
    parser.add_argument('-P', '--port', type=int, default=9090,
                        help='WebSocket port (default: 9090)')
    parser.add_argument('-d', '--duration', type=int, required=True,
                        help='Recording duration in seconds')
    parser.add_argument('-c', '--codec', choices=['h264', 'h265'], default='h264',
                        help='Video codec (default: h264)')
    parser.add_argument('-b', '--bitrate', type=int,
                        help='Bitrate in bps (e.g., 8000000 for 8 Mbps)')
    parser.add_argument('-p', '--profile', 
                        help='Video profile: WIDTHxHEIGHT@FPS (e.g., 1920x1080@30)')
    parser.add_argument('-f', '--fps', type=int,
                        help='Frame rate (if not specified in profile)')
    parser.add_argument('-o', '--output', default='saved_videos',
                        help='Output directory (default: saved_videos)')
    
    args = parser.parse_args()
    
    # Validate profile format if provided
    if args.profile and '@' not in args.profile:
        print(f"{RED}❌ Invalid profile format. Use WIDTHxHEIGHT@FPS (e.g., 1920x1080@30){RESET}")
        sys.exit(1)
    
    # Set default bitrate based on resolution if not specified
    if not args.bitrate:
        if args.profile:
            if '3840x2160' in args.profile or '4K' in args.profile:
                args.bitrate = 20000000  # 20 Mbps for 4K
            elif '1920x1080' in args.profile or 'FHD' in args.profile:
                args.bitrate = 8000000   # 8 Mbps for 1080p
            else:
                args.bitrate = 4000000   # 4 Mbps for 720p
        else:
            args.bitrate = 8000000  # Default 8 Mbps
    
    recorder = VideoRecorder(args.host, args.port, args.output)
    
    # Handle Ctrl+C gracefully
    def signal_handler(sig, frame):
        print(f"\n{YELLOW}⚠️  Interrupted. Saving recording...{RESET}")
        asyncio.create_task(recorder.cleanup())
        sys.exit(0)
    
    signal.signal(signal.SIGINT, signal_handler)
    
    try:
        await recorder.connect()
        await recorder.configure_encoder(
            args.codec, 
            args.bitrate, 
            args.profile,
            args.fps
        )
        # Wait for encoder to restart
        await asyncio.sleep(2)
        
        await recorder.record(
            args.duration,
            args.codec,
            args.bitrate, 
            args.profile,
            args.fps
        )
        
    except websockets.exceptions.WebSocketException as e:
        print(f"{RED}❌ WebSocket error: {e}{RESET}")
        print(f"{YELLOW}💡 Make sure the Android app is running and reachable at {args.host}:{args.port}{RESET}")
        sys.exit(1)
    except Exception as e:
        print(f"{RED}❌ Error: {e}{RESET}")
        import traceback
        traceback.print_exc()
        sys.exit(1)
    finally:
        await recorder.cleanup()

if __name__ == '__main__':
    asyncio.run(main())
