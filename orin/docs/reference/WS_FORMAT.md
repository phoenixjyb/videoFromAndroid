# WebSocket Video Format — Integration Notes (Android → Orin)

This document describes the WebSocket protocol and payload format produced by the Android CamControl app so native apps on Jetson Orin (or elsewhere) can ingest it reliably.

## Endpoint
- URL: `ws://<android-ip>:9090/control`
- Transport: plain WebSocket (no subprotocol)
- Mixed traffic on the same connection:
  - Text frames (UTF‑8 JSON) for control and telemetry
  - Binary frames (H.264 Annex‑B) for video access units (AUs)

The Android device also serves the web UI at `http://<android-ip>:9090/` from the same Ktor server.

## Text (JSON) frames
- Control examples:
  - `{ "cmd": "setZoomRatio", "value": 2.5 }`
  - `{ "cmd": "switchCamera", "facing": "front" }`
  - `{ "cmd": "setAeLock", "value": true }`
  - `{ "cmd": "setVideoProfile", "width": 1920, "height": 1080, "fps": 30, "highSpeed": false }`
- Telemetry example (sent from Android to clients):
  - `{ "af": 1, "ae": 1, "iso": 320, "expNs": 19982648, "zoom": 1.0, "fps": 29.9 }`

Orin ingesters can ignore text frames or parse telemetry for monitoring.

## Binary frames (video)
- Codec: H.264/AVC
- Bytestream format: Annex‑B (aka "byte-stream") with 3/4‑byte start codes
- Alignment: access-unit (AU). Each WebSocket binary message contains one AU (one or more NAL units).
- Keyframes: IDR AUs are prefixed with SPS/PPS (Android prepends csd-0/csd-1 to IDR) so decoders can lock on at any point.

GStreamer caps to use:
```
video/x-h264, stream-format=byte-stream, alignment=au
```

## Minimal ingester (Python + GStreamer)
See `orin/ws_h264_gst.py` for a complete implementation. Core idea:

```python
url = f"ws://{HOST}:{PORT}/control"
async with websockets.connect(url) as ws:
    while True:
        msg = await ws.recv()
        if isinstance(msg, (bytes, bytearray)):
            data = bytes(msg)  # one AU in Annex-B format
            # push to appsrc with caps:
            # video/x-h264,stream-format=byte-stream,alignment=au
```

Recommended pipeline on Jetson (NVDEC):
```
appsrc is-live=true format=time do-timestamp=true caps=video/x-h264,stream-format=byte-stream,alignment=au \
  ! h264parse config-interval=-1 \
  ! nvv4l2decoder enable-max-performance=1 \
  ! nvvidconv \
  ! nveglglessink sync=false
```

Software fallback:
```
appsrc ! h264parse config-interval=-1 ! avdec_h264 ! videoconvert ! autovideosink
```

## Notes
- No additional handshake: connect and start receiving. Control/telemetry is optional.
- If connecting over Wi‑Fi, ensure the LAN doesn’t enforce client isolation; open `http://<android-ip>:9090/` to verify reachability.
- For restreaming, you can feed the AUs into `rtph264pay` (see `orin/ws_h264_rtsp_server.py`).

