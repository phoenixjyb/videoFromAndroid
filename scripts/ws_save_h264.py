#!/usr/bin/env python3
import asyncio
import argparse
import os
import sys
import time
import websockets


def disable_proxies():
    os.environ['no_proxy'] = 'localhost,127.0.0.1,*'
    os.environ['NO_PROXY'] = 'localhost,127.0.0.1,*'
    for k in ['http_proxy','https_proxy','HTTP_PROXY','HTTPS_PROXY','all_proxy','ALL_PROXY']:
        os.environ.pop(k, None)


def _has_sps_or_idr(buf: bytes) -> bool:
    # Look for Annex B start codes and NAL types 7 (SPS) or 5 (IDR)
    i = 0
    n = len(buf)
    while i + 4 <= n:
        # find start code (0x00000001 or 0x000001)
        if i + 4 <= n and buf[i:i+4] == b"\x00\x00\x00\x01":
            i += 4
        elif i + 3 <= n and buf[i:i+3] == b"\x00\x00\x01":
            i += 3
        else:
            i += 1
            continue
        if i >= n:
            break
        nal_header = buf[i]
        nal_type = nal_header & 0x1F
        if nal_type in (5, 7):
            return True
        # move to next start code; a simple scan continues
    return False


async def dump_h264(uri: str, out_path: str, seconds: float, wait_keyframe: bool):
    disable_proxies()
    started = time.time()
    count_bytes = 0
    count_frames = 0
    text_msgs = 0
    started_writing = not wait_keyframe
    print(f"Connecting to {uri} ...")
    async with websockets.connect(uri, open_timeout=5) as ws:
        print(f"Connected. Writing to {out_path} for ~{seconds}s … (Ctrl+C to stop)")
        with open(out_path, 'wb') as f:
            while True:
                try:
                    msg = await asyncio.wait_for(ws.recv(), timeout=5)
                except asyncio.TimeoutError:
                    # keep waiting while within time budget
                    if time.time() - started > seconds:
                        break
                    else:
                        continue
                if isinstance(msg, (bytes, bytearray)):
                    if not started_writing:
                        started_writing = _has_sps_or_idr(msg)
                        if not started_writing:
                            continue
                    f.write(msg)
                    count_bytes += len(msg)
                    count_frames += 1
                else:
                    text_msgs += 1
                if time.time() - started > seconds:
                    break
    print(f"Done. Frames: {count_frames}, Bytes: {count_bytes}. Text msgs: {text_msgs}")


def main():
    ap = argparse.ArgumentParser(description="Save WS H.264 AnnexB stream to file")
    ap.add_argument('--host', default='127.0.0.1', help='host (default: 127.0.0.1)')
    ap.add_argument('--port', type=int, default=9090, help='port (default: 9090)')
    ap.add_argument('--seconds', type=float, default=8.0, help='recording duration (default: 8s)')
    ap.add_argument('--out', default='capture.h264', help='output file (default: capture.h264)')
    ap.add_argument('--no-wait-keyframe', action='store_true', help='start writing immediately without waiting for SPS/IDR')
    args = ap.parse_args()
    uri = f"ws://{args.host}:{args.port}/control"
    try:
        asyncio.run(dump_h264(uri, args.out, args.seconds, not args.no_wait_keyframe))
    except KeyboardInterrupt:
        print("Interrupted.")
        sys.exit(130)


if __name__ == '__main__':
    main()
