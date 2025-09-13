#!/usr/bin/env python3
"""
Jetson Orin — Restream Android WS H.264 to RTSP using GStreamer

Publishes an RTSP mount point (rtsp://<orin-ip>:8554/cam) and feeds it with
the H.264 Annex-B stream received over WebSocket from the Android phone.

Pipeline inside RTSP factory:
  appsrc name=src is-live=true format=time do-timestamp=true caps=video/x-h264,stream-format=byte-stream,alignment=au
  ! h264parse config-interval=-1
  ! rtph264pay name=pay0 pt=96
"""
import argparse
import asyncio
import os
import queue
import threading

import websockets

import gi
gi.require_version('Gst', '1.0')
gi.require_version('GstRtspServer', '1.0')
from gi.repository import Gst, GstRtspServer, GLib


def disable_proxies():
    os.environ['no_proxy'] = 'localhost,127.0.0.1,*'
    os.environ['NO_PROXY'] = 'localhost,127.0.0.1,*'
    for k in ['http_proxy','https_proxy','HTTP_PROXY','HTTPS_PROXY','all_proxy','ALL_PROXY']:
        os.environ.pop(k, None)


class WSRTSPFactory(GstRtspServer.RTSPMediaFactory):
    def __init__(self):
        super().__init__()
        self.appsrc = None
        self.queue = queue.Queue(maxsize=512)
        self.pts = 0

    def do_configure(self, media):
        self.appsrc = media.get_element().get_child_by_name('src')
        self.appsrc.set_property('format', Gst.Format.TIME)
        self.appsrc.set_property('block', True)
        # drain queue periodically
        GLib.timeout_add(5, self._push_from_queue)

    def _push_from_queue(self):
        if not self.appsrc:
            return True
        for _ in range(32):
            try:
                data = self.queue.get_nowait()
            except queue.Empty:
                break
            buf = Gst.Buffer.new_allocate(None, len(data), None)
            buf.fill(0, data)
            dur = int(1e9/30)
            buf.pts = self.pts
            buf.dts = self.pts
            buf.duration = dur
            self.pts += dur
            self.appsrc.emit('push-buffer', buf)
        return True


async def ws_reader(url: str, factory: WSRTSPFactory):
    print(f"[WS] Connect {url}")
    async with websockets.connect(url, open_timeout=5) as ws:
        print("[WS] Connected")
        while True:
            try:
                msg = await ws.recv()
            except Exception as e:
                print(f"[WS] recv end: {e}")
                break
            if isinstance(msg, (bytes, bytearray)):
                try:
                    factory.queue.put_nowait(bytes(msg))
                except queue.Full:
                    pass
            # ignore text frames


def main():
    ap = argparse.ArgumentParser(description='WS->RTSP restream')
    ap.add_argument('--host', required=True, help='Android IP')
    ap.add_argument('--port', type=int, default=9090)
    ap.add_argument('--mount', default='/cam')
    ap.add_argument('--rtsp-port', type=int, default=8554)
    args = ap.parse_args()

    disable_proxies()
    Gst.init(None)

    # Build RTSP factory
    caps = 'video/x-h264,stream-format=byte-stream,alignment=au'
    launch = f"appsrc name=src is-live=true format=time do-timestamp=true caps={caps} ! h264parse config-interval=-1 ! rtph264pay name=pay0 pt=96"
    factory = WSRTSPFactory()
    factory.set_launch(launch)
    factory.set_latency(100)

    server = GstRtspServer.RTSPServer()
    server.props.service = str(args.rtsp_port)
    mounts = server.get_mount_points()
    mounts.add_factory(args.mount, factory)
    server.attach(None)
    print(f"[RTSP] rtsp://<orin-ip>:{args.rtsp_port}{args.mount}")

    # WS thread
    url = f"ws://{args.host}:{args.port}/control"
    t = threading.Thread(target=lambda: asyncio.run(ws_reader(url, factory)), daemon=True)
    t.start()

    # GLib loop
    loop = GLib.MainLoop()
    try:
        loop.run()
    finally:
        loop.quit()


if __name__ == '__main__':
    main()

