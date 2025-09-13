#!/usr/bin/env python3
"""
Jetson Orin H.264 over WebSocket receiver with GStreamer decode

Connects to ws://<host>:<port>/control, reads binary H.264 Annex-B frames,
and feeds them to a GStreamer pipeline using appsrc. Uses NVDEC (nvv4l2decoder)
when available, falling back to software decode.
"""
import argparse
import asyncio
import os
import queue
import signal
import sys
import threading
import time

import websockets

import gi
gi.require_version('Gst', '1.0')
gi.require_version('GObject', '2.0')
from gi.repository import Gst, GObject, GLib


def disable_proxies():
    os.environ['no_proxy'] = 'localhost,127.0.0.1,*'
    os.environ['NO_PROXY'] = 'localhost,127.0.0.1,*'
    for k in ['http_proxy','https_proxy','HTTP_PROXY','HTTPS_PROXY','all_proxy','ALL_PROXY']:
        os.environ.pop(k, None)


class WsH264Receiver:
    def __init__(self, host: str, port: int, sink: str, use_hw: bool):
        self.host = host
        self.port = port
        self.url = f"ws://{host}:{port}/control"
        self.use_hw = use_hw
        self.sink = sink
        self.pipeline = None
        self.appsrc = None
        self.loop = None
        self.ws_task = None
        self.queue = queue.Queue(maxsize=512)
        self._running = threading.Event()
        self._running.set()
        self._pts = 0

    def _has_factory(self, name: str) -> bool:
        return Gst.ElementFactory.find(name) is not None

    def _build_pipeline(self):
        # Caps for Annex-B byte stream with AU alignment (SPS/PPS+IDR are in stream)
        caps = 'video/x-h264,stream-format=byte-stream,alignment=au'

        # Choose decoder path
        hw_ok = self.use_hw and self._has_factory('nvv4l2decoder')
        if hw_ok:
            decode = 'h264parse config-interval=-1 ! nvv4l2decoder enable-max-performance=1 ! nvvidconv ! nvegltransform'
            sink = self.sink or 'nveglglessink sync=false'
        else:
            decode = 'h264parse config-interval=-1 ! avdec_h264 ! videoconvert'
            sink = self.sink or 'autovideosink sync=false'

        desc = f"appsrc name=src is-live=true format=time do-timestamp=true caps={caps} ! {decode} ! {sink}"
        self.pipeline = Gst.parse_launch(desc)
        self.appsrc = self.pipeline.get_by_name('src')
        assert self.appsrc is not None
        # Set reasonable blocksize/latency
        self.appsrc.set_property('block', True)
        self.appsrc.set_property('format', Gst.Format.TIME)

    def _on_bus(self, bus: Gst.Bus, msg: Gst.Message, loop: GLib.MainLoop):
        t = msg.type
        if t == Gst.MessageType.ERROR:
            err, dbg = msg.parse_error()
            print(f"[GStreamer] ERROR: {err} dbg={dbg}", file=sys.stderr)
            loop.quit()
        elif t == Gst.MessageType.EOS:
            print("[GStreamer] EOS")
            loop.quit()
        return True

    def _push_from_queue(self):
        # Called periodically in the GLib main loop
        pushed = 0
        while pushed < 32:
            try:
                data = self.queue.get_nowait()
            except queue.Empty:
                break
            buf = Gst.Buffer.new_allocate(None, len(data), None)
            buf.fill(0, data)
            # 30 fps timestamps (rough, decoder drives real pacing)
            duration = int(1e9/30)
            buf.pts = self._pts
            buf.dts = self._pts
            buf.duration = duration
            self._pts += duration
            ret = self.appsrc.emit('push-buffer', buf)
            if ret != Gst.FlowReturn.OK:
                print(f"[GStreamer] push-buffer returned {ret}")
                return True
            pushed += 1
        return True

    async def _ws_reader(self):
        print(f"[WS] Connecting {self.url}…")
        async with websockets.connect(self.url, open_timeout=5) as ws:
            print("[WS] Connected")
            while self._running.is_set():
                try:
                    msg = await asyncio.wait_for(ws.recv(), timeout=5)
                except asyncio.TimeoutError:
                    continue
                except Exception as e:
                    print(f"[WS] recv error: {e}")
                    break
                if isinstance(msg, (bytes, bytearray)):
                    try:
                        self.queue.put_nowait(bytes(msg))
                    except queue.Full:
                        # Drop on overflow to keep latency low
                        pass
                # Ignore text telemetry here; future: export metrics
        print("[WS] Disconnected")

    def start(self):
        # GStreamer setup
        Gst.init(None)
        self._build_pipeline()

        bus = self.pipeline.get_bus()
        bus.add_signal_watch()
        loop = GLib.MainLoop()
        bus.connect('message', self._on_bus, loop)

        # Periodic queue drain
        GLib.timeout_add(5, self._push_from_queue)

        # Start pipeline
        self.pipeline.set_state(Gst.State.PLAYING)

        # WebSocket thread (asyncio)
        def ws_thread():
            disable_proxies()
            asyncio.run(self._ws_reader())

        t = threading.Thread(target=ws_thread, daemon=True)
        t.start()

        # Run main loop until SIGINT or EOS/ERROR
        def handle_sigint(*_):
            print("[Main] SIGINT")
            self.stop()
            loop.quit()

        signal.signal(signal.SIGINT, handle_sigint)
        try:
            loop.run()
        finally:
            self.stop()
            t.join(timeout=1)

    def stop(self):
        if self._running.is_set():
            self._running.clear()
        try:
            if self.appsrc:
                self.appsrc.emit('end-of-stream')
        except Exception:
            pass
        if self.pipeline:
            self.pipeline.set_state(Gst.State.NULL)


def main():
    p = argparse.ArgumentParser(description='Jetson Orin WS H.264 receiver')
    p.add_argument('--host', required=True, help='Android phone IP or hostname')
    p.add_argument('--port', type=int, default=9090)
    p.add_argument('--sink', default='', help='GStreamer sink element (default nveglglessink or autovideosink)')
    p.add_argument('--no-hw', action='store_true', help='Disable hardware decoder (force software)')
    args = p.parse_args()

    r = WsH264Receiver(args.host, args.port, args.sink, use_hw=not args.no_hw)
    r.start()


if __name__ == '__main__':
    main()

