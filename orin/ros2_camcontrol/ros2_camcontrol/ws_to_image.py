#!/usr/bin/env python3
import os
import sys
import threading
import asyncio
import argparse
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

import websockets
import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst, GLib


def disable_proxies():
    os.environ['no_proxy'] = 'localhost,127.0.0.1,*'
    os.environ['NO_PROXY'] = 'localhost,127.0.0.1,*'
    for k in ['http_proxy','https_proxy','HTTP_PROXY','HTTPS_PROXY','all_proxy','ALL_PROXY']:
        os.environ.pop(k, None)


class WSH264ToImage(Node):
    def __init__(self, host: str, port: int, topic: str, use_hw: bool):
        super().__init__('ws_h264_to_image')
        self.host = host
        self.port = port
        self.url = f"ws://{host}:{port}/control"
        self.publisher = self.create_publisher(Image, topic, 10)
        self._pts = 0
        self._appsink = None

        Gst.init(None)

        caps = 'video/x-h264,stream-format=byte-stream,alignment=au'
        if use_hw and Gst.ElementFactory.find('nvv4l2decoder'):
            decode = 'h264parse config-interval=-1 ! nvv4l2decoder ! nvvidconv ! video/x-raw,format=RGB'
        else:
            decode = 'h264parse config-interval=-1 ! avdec_h264 ! videoconvert ! video/x-raw,format=RGB'

        desc = f"appsrc name=src is-live=true format=time do-timestamp=true caps={caps} ! {decode} ! appsink name=sink emit-signals=true max-buffers=1 drop=true"
        self.pipeline = Gst.parse_launch(desc)
        self.appsrc = self.pipeline.get_by_name('src')
        self._appsink = self.pipeline.get_by_name('sink')
        self._appsink.connect('new-sample', self.on_new_sample)
        self.appsrc.set_property('block', True)
        self.pipeline.set_state(Gst.State.PLAYING)

        # WS feeder thread
        self._ws_thread = threading.Thread(target=self._ws_main, daemon=True)
        self._ws_thread.start()

    def destroy_node(self):
        try:
            self.pipeline.set_state(Gst.State.NULL)
        except Exception:
            pass
        super().destroy_node()

    def on_new_sample(self, sink):
        sample = sink.emit('pull-sample')
        buf = sample.get_buffer()
        caps = sample.get_caps()
        s = caps.get_structure(0)
        width = s.get_value('width')
        height = s.get_value('height')
        success, mapinfo = buf.map(Gst.MapFlags.READ)
        if not success:
            return Gst.FlowReturn.OK
        try:
            arr = np.frombuffer(mapinfo.data, dtype=np.uint8)
            arr = arr.reshape((height, width, 3))
            msg = Image()
            msg.height = height
            msg.width = width
            msg.encoding = 'rgb8'
            msg.is_bigendian = 0
            msg.step = width * 3
            msg.data = arr.tobytes()
            self.publisher.publish(msg)
        finally:
            buf.unmap(mapinfo)
        return Gst.FlowReturn.OK

    def _ws_main(self):
        disable_proxies()

        async def reader():
            async with websockets.connect(self.url, open_timeout=5) as ws:
                while True:
                    try:
                        msg = await asyncio.wait_for(ws.recv(), timeout=5)
                    except asyncio.TimeoutError:
                        continue
                    except Exception:
                        break
                    if isinstance(msg, (bytes, bytearray)):
                        data = bytes(msg)
                        buf = Gst.Buffer.new_allocate(None, len(data), None)
                        buf.fill(0, data)
                        dur = int(1e9/30)
                        buf.pts = self._pts
                        buf.dts = self._pts
                        buf.duration = dur
                        self._pts += dur
                        self.appsrc.emit('push-buffer', buf)
        asyncio.run(reader())


def main(argv=None):
    parser = argparse.ArgumentParser(description='WS H.264 -> ROS2 Image')
    parser.add_argument('--host', required=True)
    parser.add_argument('--port', type=int, default=9090)
    parser.add_argument('--topic', default='/camera/image_rgb')
    parser.add_argument('--no-hw', action='store_true')
    args = parser.parse_args(argv)

    rclpy.init()
    node = WSH264ToImage(args.host, args.port, args.topic, use_hw=not args.no_hw)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main(sys.argv[1:])

