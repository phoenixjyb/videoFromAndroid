#!/usr/bin/env python3
import os
import sys
import json
import threading
import asyncio
import argparse
import inspect
import time
from pathlib import Path
from typing import Optional

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
        timing_sample_interval: int,
        dry_run_publish: bool,
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
        self._last_pub_monotonic_ns = 0
        self._codec = codec.lower()
        self._published_count = 0
        self._last_pub_log_ns = 0
        env_timing = os.environ.get('WS_TIMING_SAMPLE_INTERVAL')
        if timing_sample_interval <= 0 and env_timing:
            try:
                timing_sample_interval = int(env_timing)
            except ValueError:
                self.get_logger().warning(
                    f"Invalid WS_TIMING_SAMPLE_INTERVAL='{env_timing}', ignoring"
                )
        self._timing_sample_interval = max(0, timing_sample_interval)
        self._timing_enabled = self._timing_sample_interval > 0
        if self._timing_enabled:
            self.get_logger().info(
                f"Timing instrumentation enabled: sample every {self._timing_sample_interval} published frames"
            )
        self._dry_run_publish = dry_run_publish
        if self._dry_run_publish:
            self.get_logger().warning("Dry-run publish enabled: skipping publisher.publish calls")

        Gst.init(None)

        if self._codec in ('h265', 'hevc'):
            caps = 'video/x-h265,stream-format=byte-stream,alignment=au'
            parse = 'h265parse config-interval=-1'
            hw_decode = (
                f"{parse} ! queue leaky=downstream max-size-buffers=1 max-size-time=0 max-size-bytes=0 "
                "! nvv4l2decoder disable-dpb=true enable-max-performance=true "
                "! nvvidconv ! video/x-raw,format=RGBA "
                "! videoconvert ! video/x-raw,format=RGB"
            )
            sw_decode = f"{parse} ! avdec_h265 ! videoconvert ! video/x-raw,format=RGB"
        else:
            caps = 'video/x-h264,stream-format=byte-stream,alignment=au'
            parse = 'h264parse config-interval=-1'
            hw_decode = (
                f"{parse} ! queue leaky=downstream max-size-buffers=1 max-size-time=0 max-size-bytes=0 "
                "! nvv4l2decoder disable-dpb=true enable-max-performance=true "
                "! nvvidconv ! video/x-raw,format=RGBA "
                "! videoconvert ! video/x-raw,format=RGB"
            )
            sw_decode = f"{parse} ! avdec_h264 ! videoconvert ! video/x-raw,format=RGB"
            self._codec = 'h264'

        if use_hw and Gst.ElementFactory.find('nvv4l2decoder'):
            decode = hw_decode
        else:
            decode = sw_decode
        # Log which decode path was selected for diagnostics
        try:
            self.get_logger().info(f"Decoder selected: {'hw' if decode == hw_decode else 'sw'}, codec={self._codec}")
        except Exception:
            pass

        desc = (
            "appsrc name=src is-live=true format=time do-timestamp=true "
            f"caps={caps} ! {decode} ! "
            "appsink name=sink emit-signals=true max-buffers=1 drop=true sync=false"
        )
        self.pipeline = Gst.parse_launch(desc)
        self.appsrc = self.pipeline.get_by_name('src')
        self._appsink = self.pipeline.get_by_name('sink')
        self._appsink.connect('new-sample', self.on_new_sample)
        self.appsrc.set_property('block', True)
        self.appsrc.set_property('format', Gst.Format.TIME)
        self.appsrc.set_property('do-timestamp', True)
        self.appsrc.set_property('is-live', True)
        self.appsrc.set_property('max-bytes', 1 << 20)  # 1 MB buffer
        self.pipeline.set_state(Gst.State.PLAYING)

        if target_rate_hz > 0:
            self._publish_period_ns = int(1e9 / target_rate_hz)
        try:
            self.get_logger().info(f"Target rate: {target_rate_hz} Hz -> publish_period_ns={self._publish_period_ns}")
        except Exception:
            pass

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
        timing_enabled = self._timing_enabled
        timing_start_ns = time.perf_counter_ns() if timing_enabled else None
        map_done_ns = None
        pack_done_ns = None
        publish_done_ns = None
        sample_timing = False
        delta_publish_ns = None
        clock_done_ns = None
        payload_done_ns = None
        info_done_ns = None
        
        # Debug: Log that we received a frame
        if not hasattr(self, '_frame_count'):
            self._frame_count = 0
            self.get_logger().info(f"First decoded frame received: {width}x{height}")
        self._frame_count += 1
        if self._frame_count % 30 == 0:
            self.get_logger().info(f"Decoded {self._frame_count} frames")
        
        success, mapinfo = buf.map(Gst.MapFlags.READ)
        if not success:
            return Gst.FlowReturn.OK
        if timing_enabled:
            map_done_ns = time.perf_counter_ns()
        try:
            now_monotonic_ns = time.monotonic_ns()
            if self._publish_period_ns > 0 and self._last_pub_monotonic_ns != 0:
                delta_publish_ns = now_monotonic_ns - self._last_pub_monotonic_ns
                if delta_publish_ns < self._publish_period_ns:
                    return Gst.FlowReturn.OK
            elif self._last_pub_monotonic_ns != 0:
                delta_publish_ns = now_monotonic_ns - self._last_pub_monotonic_ns
            self._last_pub_monotonic_ns = now_monotonic_ns

            sample_index = self._published_count + 1
            if timing_enabled and self._timing_sample_interval > 0:
                sample_timing = (sample_index % self._timing_sample_interval) == 0
            now_time = self.get_clock().now()
            if sample_timing:
                clock_done_ns = time.perf_counter_ns()
            payload = bytes(mapinfo.data)
            if sample_timing:
                payload_done_ns = time.perf_counter_ns()
            # GStreamer buffers may include row padding (stride). If so, extract
            # the first width*3 bytes of each row into a packed RGB buffer.
            expected_row = width * 3
            total = len(payload)
            if height > 0 and total % height == 0:
                rowstride = total // height
            else:
                rowstride = expected_row

            if rowstride == expected_row:
                packed = payload
            else:
                # Extract per-row slices
                out = bytearray(expected_row * height)
                for r in range(height):
                    start = r * rowstride
                    out[r * expected_row:(r + 1) * expected_row] = payload[start:start + expected_row]
                packed = bytes(out)
            if sample_timing:
                pack_done_ns = time.perf_counter_ns()

            msg = Image()
            now_msg = now_time.to_msg()
            msg.header.stamp = now_msg
            msg.header.frame_id = self.frame_id
            msg.height = height
            msg.width = width
            msg.encoding = 'rgb8'
            msg.is_bigendian = 0
            msg.step = expected_row
            msg.data = packed
            if not self._dry_run_publish:
                try:
                    self.publisher.publish(msg)
                except Exception as e:
                    self.get_logger().error(f"Failed to publish image: {e}")
            if sample_timing:
                publish_done_ns = time.perf_counter_ns()

            info = CameraInfo()
            template = self._camera_info_template
            self._published_count += 1

            if (
                sample_timing
                and timing_start_ns is not None
                and map_done_ns is not None
                and pack_done_ns is not None
                and publish_done_ns is not None
            ):
                map_ms = (map_done_ns - timing_start_ns) / 1e6
                pack_ms = (pack_done_ns - map_done_ns) / 1e6
                publish_ms = (publish_done_ns - pack_done_ns) / 1e6
                total_ms = (publish_done_ns - timing_start_ns) / 1e6
                period_ms = (delta_publish_ns / 1e6) if delta_publish_ns is not None else float('nan')
                clock_ms = ((clock_done_ns - map_done_ns) / 1e6) if clock_done_ns else float('nan')
                payload_ms = ((payload_done_ns - clock_done_ns) / 1e6) if payload_done_ns and clock_done_ns else float('nan')
                info_ms = ((info_done_ns - publish_done_ns) / 1e6) if info_done_ns else float('nan')
                self.get_logger().info(
                    "Timing sample (pub #{0}): map={1:.3f}ms pack={2:.3f}ms publish={3:.3f}ms total={4:.3f}ms period={5:.3f}ms clock={6:.3f}ms payload={7:.3f}ms info={8:.3f}ms".format(
                        sample_index,
                        map_ms,
                        pack_ms,
                        publish_ms,
                        total_ms,
                        period_ms,
                        clock_ms,
                        payload_ms,
                        info_ms,
                    )
                )

            if self._published_count % 30 == 0:
                if self._last_pub_log_ns == 0:
                    self._last_pub_log_ns = now_monotonic_ns
                else:
                    delta_ns = now_monotonic_ns - self._last_pub_log_ns
                    if delta_ns > 0:
                        hz = 30 * 1e9 / delta_ns
                        self.get_logger().info(
                            f"Published {self._published_count} frames (approx {hz:.1f} Hz)"
                        )
                    self._last_pub_log_ns = now_monotonic_ns
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
            if not self._dry_run_publish:
                self.camera_info_pub.publish(info)
            if sample_timing:
                info_done_ns = time.perf_counter_ns()
        finally:
            buf.unmap(mapinfo)
        return Gst.FlowReturn.OK

    def _ws_main(self):
        disable_proxies()
        
        frame_count = [0]  # Use list to allow modification in nested function
        push_timing_interval = 30

        async def reader():
            connect_kwargs = {}
            try:
                sig = inspect.signature(websockets.connect)
                params = sig.parameters
                if 'open_timeout' in params:
                    connect_kwargs['open_timeout'] = 5
                if 'loop' in params:
                    loop = asyncio.get_running_loop()
                    connect_kwargs['loop'] = loop
                    # Some very old websockets also expect timeouts on the same loop
                    if 'ping_timeout' in params and params['ping_timeout'].default is inspect._empty:
                        connect_kwargs['ping_timeout'] = None
            except (ValueError, TypeError):
                self.get_logger().warning(
                    "Unable to inspect websockets.connect() signature; using default arguments"
                )

            async with websockets.connect(self.url, **connect_kwargs) as ws:
                self.get_logger().info(f"WebSocket connected to {self.url}")
                while True:
                    try:
                        msg = await asyncio.wait_for(ws.recv(), timeout=5)
                    except asyncio.TimeoutError:
                        continue
                    except Exception as e:
                        self.get_logger().error(f"WebSocket error: {e}")
                        break
                    if isinstance(msg, (bytes, bytearray)):
                        frame_count[0] += 1
                        if frame_count[0] == 1:
                            self.get_logger().info(f"First video frame received: {len(msg)} bytes")
                        if frame_count[0] % 30 == 0:
                            self.get_logger().info(f"Received {frame_count[0]} video frames from WebSocket")
                        data = bytes(msg)
                        buf = Gst.Buffer.new_allocate(None, len(data), None)
                        buf.fill(0, data)
                        dur = int(1e9/30)
                        buf.pts = self._pts
                        buf.dts = self._pts
                        buf.duration = dur
                        self._pts += dur
                        push_start = time.perf_counter_ns()
                        ret = self.appsrc.emit('push-buffer', buf)
                        if frame_count[0] % push_timing_interval == 0:
                            push_ms = (time.perf_counter_ns() - push_start) / 1e6
                            self.get_logger().info(
                                f"push-buffer ret={ret} count={frame_count[0]} took {push_ms:.2f}ms"
                            )
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
    parser.add_argument(
        '--timing-sample-interval',
        type=int,
        default=0,
        help='Collect timing stats every N published frames (0 to disable)'
    )
    parser.add_argument(
        '--dry-run-publish',
        action='store_true',
        help='Skip publisher.publish calls to measure pipeline throughput without ROS backpressure'
    )
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
        timing_sample_interval=args.timing_sample_interval,
        dry_run_publish=args.dry_run_publish,
    )
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main(sys.argv[1:])
