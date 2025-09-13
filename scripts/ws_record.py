#!/usr/bin/env python3
import asyncio, json, os, sys, time
import websockets

def disable_proxies():
    os.environ['no_proxy'] = 'localhost,127.0.0.1,*'
    os.environ['NO_PROXY'] = 'localhost,127.0.0.1,*'
    for k in ['http_proxy','https_proxy','HTTP_PROXY','HTTPS_PROXY','all_proxy','ALL_PROXY']:
        os.environ.pop(k, None)

async def main(host='127.0.0.1', port=9090, seconds=5, name=None):
    disable_proxies()
    uri = f'ws://{host}:{port}/control'
    print('Connecting to', uri)
    async with websockets.connect(uri, open_timeout=5) as ws:
        print('Connected')
        cmd = {'cmd': 'startRecording'}
        if name:
            cmd['name'] = name
        await ws.send(json.dumps(cmd))
        print('Started recording for ~', seconds, 's')
        await asyncio.sleep(seconds)
        await ws.send(json.dumps({'cmd': 'stopRecording'}))
        print('Stopped recording')

if __name__ == '__main__':
    host = sys.argv[1] if len(sys.argv) > 1 else '127.0.0.1'
    port = int(sys.argv[2]) if len(sys.argv) > 2 else 9090
    seconds = float(sys.argv[3]) if len(sys.argv) > 3 else 5
    name = sys.argv[4] if len(sys.argv) > 4 else None
    asyncio.run(main(host, port, seconds, name))

