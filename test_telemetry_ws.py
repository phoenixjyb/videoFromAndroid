#!/usr/bin/env python3
import asyncio
import json
import os
import websockets

def _disable_proxies():
    os.environ['no_proxy'] = 'localhost,127.0.0.1,*'
    os.environ['NO_PROXY'] = 'localhost,127.0.0.1,*'
    for k in ['http_proxy','https_proxy','HTTP_PROXY','HTTPS_PROXY','all_proxy','ALL_PROXY']:
        os.environ.pop(k, None)

async def main():
    _disable_proxies()
    uri = 'ws://127.0.0.1:9090/control'
    print('Connecting to', uri)
    async with websockets.connect(uri, open_timeout=5) as ws:
        print('Connected')
        # try receive telemetry
        for i in range(2):
            msg = await asyncio.wait_for(ws.recv(), timeout=5)
            if isinstance(msg, bytes):
                print('got binary', len(msg))
            else:
                print('got text', (msg[:120] + '...') if len(msg) > 120 else msg)
        await ws.send(json.dumps({'cmd': 'setZoomRatio', 'value': 2.3}))
        print('sent zoom')
        msg = await asyncio.wait_for(ws.recv(), timeout=5)
        if isinstance(msg, bytes):
            print('post-zoom binary', len(msg))
        else:
            print('post-zoom text', (msg[:120] + '...') if len(msg) > 120 else msg)

if __name__ == '__main__':
    asyncio.run(main())

