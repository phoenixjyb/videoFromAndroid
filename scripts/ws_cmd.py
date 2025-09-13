#!/usr/bin/env python3
import argparse
import asyncio
import json
import os
import sys
import websockets


def disable_proxies():
    os.environ['no_proxy'] = 'localhost,127.0.0.1,*'
    os.environ['NO_PROXY'] = 'localhost,127.0.0.1,*'
    for k in ['http_proxy','https_proxy','HTTP_PROXY','HTTPS_PROXY','all_proxy','ALL_PROXY']:
        os.environ.pop(k, None)


async def send_cmd(uri: str, payload: dict):
    disable_proxies()
    async with websockets.connect(uri, open_timeout=5) as ws:
        await ws.send(json.dumps(payload))
        print(f"Sent: {payload}")


def main():
    ap = argparse.ArgumentParser(description='Send control commands over WS to Android app')
    ap.add_argument('--host', default='127.0.0.1')
    ap.add_argument('--port', type=int, default=9090)
    sub = ap.add_subparsers(dest='cmd')

    # Raw JSON
    raw = sub.add_parser('raw', help='Send raw JSON string')
    raw.add_argument('json', help='JSON payload')

    # Convenience commands
    zb = sub.add_parser('set-bitrate', help='Set encoder bitrate (bps)')
    zb.add_argument('bps', type=int)

    zp = sub.add_parser('set-profile', help='Set video profile WxHxFPSxHS')
    zp.add_argument('profile', help='e.g. 1920x1080x30x0')

    sc = sub.add_parser('switch-camera', help='Switch camera facing')
    sc.add_argument('facing', choices=['back', 'front'])

    zl = sub.add_parser('set-zoom', help='Set zoom ratio')
    zl.add_argument('ratio', type=float)

    args = ap.parse_args()
    uri = f"ws://{args.host}:{args.port}/control"

    if args.cmd == 'raw':
        try:
            payload = json.loads(args.json)
        except Exception as e:
            print(f"Invalid JSON: {e}", file=sys.stderr)
            sys.exit(2)
    elif args.cmd == 'set-bitrate':
        payload = {"cmd": "setBitrate", "bitrate": args.bps}
    elif args.cmd == 'set-profile':
        w,h,fps,hs = map(int, args.profile.split('x'))
        payload = {"cmd": "setVideoProfile", "width": w, "height": h, "fps": fps, "highSpeed": bool(hs)}
    elif args.cmd == 'switch-camera':
        payload = {"cmd": "switchCamera", "facing": args.facing}
    elif args.cmd == 'set-zoom':
        payload = {"cmd": "setZoomRatio", "value": args.ratio}
    else:
        ap.print_help()
        sys.exit(1)

    asyncio.run(send_cmd(uri, payload))


if __name__ == '__main__':
    main()

