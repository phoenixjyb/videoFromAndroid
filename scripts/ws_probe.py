import asyncio, os, websockets
os.environ['no_proxy']=os.environ['NO_PROXY']='localhost,127.0.0.1,*'
for k in ['http_proxy','https_proxy','HTTP_PROXY','HTTPS_PROXY','all_proxy','ALL_PROXY']:
    os.environ.pop(k, None)
async def main():
    uri='ws://127.0.0.1:9090/control'
    async with websockets.connect(uri, open_timeout=5) as ws:
        print('Connected')
        for _ in range(40):
            msg = await asyncio.wait_for(ws.recv(), timeout=5)
            if isinstance(msg, (bytes, bytearray)):
                print('got binary', len(msg))
                return
            else:
                print('got text', msg[:60])
        print('no binary frames observed')
if __name__ == '__main__':
    asyncio.run(main())
