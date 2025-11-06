#!/usr/bin/env python3
import os
import sys
import json
import threading
import asyncio
import argparse
from pathlib import Path
from typing import Optional
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo

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
    def __init__(
        self,
        host: str,
        port: int,
        image_topic: str,
        info_topic: str,
        frame_id: str,
        use_hw: bool,
        camera_info_path: Optional[str],
        target_rate_hz: float,
        codec: str,
    ):
        super().__init__('ws_h264_to_image')
        self.host = host
        self.port = port
        self.url = f"ws://{host}:{port}/control"
        self.frame_id = frame_id
        self.publisher = self.create_publisher(Image, image_topic, 10)
        self.camera_info_pub = self.create_publisher(CameraInfo, info_topic, 10)
        self._camera_info_template = self._load_camera_info(camera_info_path, frame_id)
        self._pts = 0
        self._appsink = None
        self._publish_period_ns = 0
        self._last_pub_ns = 0
        self._codec = codec.lower()

        Gst.init(None)

        if self._codec in ('h265', 'hevc'):
            caps = 'video/x-h265,stream-format=byte-stream,alignment=au'
            parse = 'h265parse config-interval=-1'
            hw_decode = f'{parse} ! nvv4l2decoder ! nvvidconv ! video/x-raw,format=RGB'
            sw_decode = f'{parse} ! avdec_h265 ! videoconvert ! video/x-raw,format=RGB'
        else:
            caps = 'video/x-h264,stream-format=byte-stream,alignment=au'
            parse = 'h264parse config-interval=-1'
            hw_decode = f'{parse} ! nvv4l2decoder ! nvvidconv ! video/x-raw,format=RGB'
            sw_decode = f'{parse} ! avdec_h264 ! videoconvert ! video/x-raw,format=RGB'
            self._codec = 'h264'

        if use_hw and Gst.ElementFactory.find('nvv4l2decoder'):
            decode = hw_decode
        else:
            decode = sw_decode

        desc = f"appsrc name=src is-live=true format=time do-timestamp=true caps={caps} ! {decode} ! appsink name=sink emit-signals=true max-buffers=1 drop=true"
        self.pipeline = Gst.parse_launch(desc)
        self.appsrc = self.pipeline.get_by_name('src')
        self._appsink = self.pipeline.get_by_name('sink')
        self._appsink.connect('new-sample', self.on_new_sample)
        self.appsrc.set_property('block', True)
        self.pipeline.set_state(Gst.State.PLAYING)

        if target_rate_hz > 0:
            self._publish_period_ns = int(1e9 / target_rate_hz)

        # WS feeder thread
        self._ws_thread = threading.Thread(target=self._ws_main, daemon=True)
        self._ws_thread.start()

    def destroy_node(self):
        try:
            self.pipeline.set_state(Gst.State.NULL)
        except Exception:
            pass
        super().destroy_node()

    def _load_camera_info(self, maybe_path: Optional[str], frame_id: str) -> CameraInfo:
        msg = CameraInfo()
        msg.header.frame_id = frame_id
        if not maybe_path:
            return msg
        path = Path(maybe_path)
        try:
            text = path.read_text()
        except Exception as exc:
            self.get_logger().warning(f"Failed to read camera info file '{path}': {exc}")
            return msg

        data = None
        try:
            import yaml  # type: ignore

            data = yaml.safe_load(text)
        except Exception:
            try:
                data = json.loads(text)
            except Exception as exc:
                self.get_logger().error(f"Unable to parse camera info '{path}': {exc}")
                return msg

        if not isinstance(data, dict):
            self.get_logger().warning(f"Camera info file '{path}' is not a dict, ignoring.")
            return msg

        try:
            if 'width' in data:
                msg.width = int(data['width'])
            if 'height' in data:
                msg.height = int(data['height'])
            if 'distortion_model' in data:
                msg.distortion_model = str(data['distortion_model'])
            if 'd' in data:
                msg.d = [float(x) for x in data['d']]
            elif 'D' in data:
                msg.d = [float(x) for x in data['D']]
            if 'k' in data:
                vals = data['k']
            elif 'K' in data:
                vals = data['K']
            else:
                vals = None
            if vals:
                msg.k = [float(x) for x in vals][:9]
            if 'r' in data:
                vals = data['r']
            elif 'R' in data:
                vals = data['R']
            else:
                vals = None
            if vals:
                msg.r = [float(x) for x in vals][:9]
            if 'p' in data:
                vals = data['p']
            elif 'P' in data:
                vals = data['P']
            else:
                vals = None
            if vals:
                msg.p = [float(x) for x in vals][:12]
            if 'binning_x' in data:
                msg.binning_x = int(data['binning_x'])
            if 'binning_y' in data:
                msg.binning_y = int(data['binning_y'])
        except Exception as exc:
            self.get_logger().warning(f"Camera info file '{path}' had unexpected values: {exc}")
        return msg

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
            now_time = self.get_clock().now()
            now_ns = now_time.nanoseconds
            if self._publish_period_ns > 0 and self._last_pub_ns != 0:
                if (now_ns - self._last_pub_ns) < self._publish_period_ns:
                    return Gst.FlowReturn.OK
            self._last_pub_ns = now_ns

            arr = np.frombuffer(mapinfo.data, dtype=np.uint8)
            arr = arr.reshape((height, width, 3))
            msg = Image()
            now_msg = now_time.to_msg()
            msg.header.stamp = now_msg
            msg.header.frame_id = self.frame_id
            msg.height = height
            msg.width = width
            msg.encoding = 'rgb8'
            msg.is_bigendian = 0
            msg.step = width * 3
            msg.data = arr.tobytes()
            self.publisher.publish(msg)

            info = CameraInfo()
            template = self._camera_info_template
            info.header.stamp = now_msg
            info.header.frame_id = self.frame_id
            info.width = width if template.width == 0 else template.width
            info.height = height if template.height == 0 else template.height
            info.distortion_model = template.distortion_model
            info.d = list(template.d)
            info.k = list(template.k)
            info.r = list(template.r)
            info.p = list(template.p)
            info.binning_x = template.binning_x
            info.binning_y = template.binning_y
            info.roi = template.roi
            if info.width == 0:
                info.width = width
            if info.height == 0:
                info.height = height
            self.camera_info_pub.publish(info)
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
    parser = argparse.ArgumentParser(description='WS H.26x -> ROS2 Image')
    parser.add_argument('--host', required=True)
    parser.add_argument('--port', type=int, default=9090)
    parser.add_argument('--topic', default='/recomo/rgb')
    parser.add_argument('--camera-info-topic', default='')
    parser.add_argument('--camera-info-file', default='')
    parser.add_argument('--frame-id', default='camera_rgb')
    parser.add_argument('--rate', type=float, default=10.0, help='Target publish rate in Hz (0 to disable throttling)')
    parser.add_argument('--codec', default='h265', help='h264 or h265')
    parser.add_argument('--no-hw', action='store_true')
    args = parser.parse_args(argv)

    image_topic = args.topic
    info_topic = args.camera_info_topic.strip()
    if not info_topic:
        base = image_topic.rstrip('/')
        if not base:
            info_topic = '/camera_info'
        elif '/' in base:
            parent = base.rsplit('/', 1)[0]
            if not parent:
                parent = base.lstrip('/')
                info_topic = f"/{parent}/camera_info" if parent else '/camera_info'
            else:
                info_topic = f"{parent}/camera_info"
        else:
            info_topic = f"/{base}/camera_info" if not base.startswith('/') else f"{base}/camera_info"

    rclpy.init()
    node = WSH264ToImage(
        args.host,
        args.port,
        image_topic=image_topic,
        info_topic=info_topic,
        frame_id=args.frame_id,
        use_hw=not args.no_hw,
        camera_info_path=args.camera_info_file or None,
        target_rate_hz=args.rate,
        codec=args.codec,
    )
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main(sys.argv[1:])
