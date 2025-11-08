# CamControl — 安卓相机流系统（Android + Mac 浏览器 + Orin）

中文 | [English](README.md)

## 概览
- 目标：远程控制安卓手机摄像头，在 Mac 浏览器预览，同时在 Jetson Orin 原生解码/处理或转发。
- 传输：Android 端单一 Ktor WebSocket（端口 `9090`，路径 `/control`），文本帧承载控制与遥测，二进制帧承载视频（H.264 Annex‑B）。
- 当前状态：Mac 端浏览器预览与控制已打通；Orin 端接入/RTSP/ROS2 节点已完成脚手架。

## 架构（运行时）
```
Mac（浏览器） ←→ Android CamControl（Ktor WS 9090） → Orin（WS 接入）
  │  HTML/JS          └─ Camera2 + 编码（H.264）        ├─ GStreamer NVDEC 解码显示
  │  WebCodecs              ↑ 广播控制命令               ├─ RTSP 转发（rtsp://:8554/cam）
  └─ 控制 JSON              │ 遥测 JSON                  └─ ROS2 节点发布 sensor_msgs/Image
```

- 浏览器：连接 `ws://<host>:9090/control`，优先使用 WebCodecs 解码（Annex‑B→AVCC），回退到 Broadway（JS 解码）。
- Android：Ktor 提供 Web 页面（`/`）和 WebSocket（`/control`）；MediaCodec 输出 H.264；Service 将控制转发给 Activity；广播遥测。
- Orin：原生 Python 通过 WebSocket 接收视频帧并注入 GStreamer（`appsrc → h264parse → nvv4l2decoder`）；支持 RTSP 服务与 ROS2 发布。

## 软件结构
```
app/（Android 应用）
  CamControlService.kt     # 前台 Service：WS 服务、编码器、遥测转发
  MainActivity.kt          # 相机流水线（预览 + 挂接编码 Surface）
  camera/Camera2Controller.kt
  encode/VideoEncoder.kt   # MediaCodec H.264（Annex‑B）
  transport/ControlServer.kt（Ktor HTTP+WS）/ ControlCommand.kt / Telemetry.kt
  assets/index.html        # Web UI（WebCodecs 解码 + 控件）

scripts/
  ws_probe.py / ws_save_h264.py / record_on_device.py  # 调试与抓流
  log.sh / webui.sh                             # 开发辅助（端口 9090）

orin/
  ws_h264_gst.py             # WS 接入 → GStreamer 解码显示（NVDEC）
  ws_h264_rtsp_server.py     # WS → RTSP（rtsp://<orin-ip>:8554/cam）
  ros2_camcontrol/           # ROS2 Humble 包（WS → sensor_msgs/Image）
  README.md / ARCHITECTURE.md

README.md / README_zh.md     # 双语总览
DIARY.md / PROJECT_STATUS_SUMMARY.md
```

## 协议
- WebSocket：`ws://<phone-ip>:9090/control`
  - 文本帧（JSON）：
    - `{\"cmd\":\"setZoomRatio\",\"value\":2.5}`
    - `{\"cmd\":\"switchCamera\",\"facing\":\"front\"}`
    - `{\"cmd\":\"setAeLock\",\"value\":true}` / `setAwbLock` / `startRecording` / `stopRecording` / `setVideoProfile`
  - 遥测（JSON）：`{af, ae, iso, expNs, zoom, fps}`
  - 二进制帧：H.264 Annex‑B（IDR 前含 SPS/PPS）

## 快速上手
### Android（构建与安装）
- USB 连接手机（开发者模式）。
- 构建安装：`./gradlew installDebug`
- 端口转发：`adb forward tcp:9090 tcp:9090`

### Mac 浏览器预览
- 打开 `http://localhost:9090`（Edge/Chrome）。
- 若首屏空白，刷新一次（等待 SPS/PPS 关键帧）。
- 排障：保持 App 前台；确认 `adb forward`；关闭本机代理或将 `localhost` 直连；`scripts/log.sh --both --forward` 查看日志。

### 本地抓流（Mac）
- `source .venv/bin/activate && python scripts/ws_save_h264.py --seconds 10 --out capture.h264`
- Remux：`ffmpeg -y -f h264 -i capture.h264 -c copy capture.mp4`

### Orin — 显示/解码
- 安装依赖：`python3-gi`、`gir1.2-gstreamer-1.0`、GStreamer 插件、`nvidia-l4t-gstreamer`、`websockets`。
- 运行：`python3 orin/ws_h264_gst.py --host <android-ip> --codec h265`（默认手机推送 HEVC，如需 H.264 请加 `--codec h264`）

### Orin — RTSP 转发
- `python3 orin/ws_h264_rtsp_server.py --host <android-ip>`
- 播放：`rtsp://<orin-ip>:8554/cam`

### Orin — ROS2 图像发布（快速启动）
- 前置条件：`android-tools-adb`、ROS2 Humble、GStreamer 依赖，并已在 `orin/ros2_camcontrol` 下执行 `colcon build --symlink-install`。
- 将安卓手机通过 USB 连接并打开 CamControl App 后，运行：
  ```bash
  ./quick_start.sh
  ```
  脚本会检测 ADB、建立 `localhost:9100 → device:9090` 转发、source ROS2 环境，并以 10 Hz（HEVC）启动 `ros2_camcontrol.ws_to_image`，输出主题 `/recomo/rgb` 与 `/recomo/camera_info`。
- 如需一次性收集 CPU 占用、话题频率、最近日志，可使用诊断脚本：
  ```bash
  ./scripts/stream_diagnostics.sh
  ./scripts/stream_diagnostics.sh --dry-run-publish  # 跳过发布，仅测 pipeline 延迟
  ```
- 节点运行期间，可在新终端查看 ROS2 信息：
  ```bash
  source /opt/ros/humble/setup.bash
  source /home/nvidia/videoFromAndroid/orin/ros2_camcontrol/install/setup.bash
  ros2 topic hz /recomo/rgb --window 50
  ros2 topic echo /recomo/rgb --once
  ```
  默认解码分辨率为 1920×1080，并发布 RGB 图像帧。

### Orin — ROS2 图像发布
- ROS2 Humble：`source /opt/ros/humble/setup.bash`
- 构建：`cd orin/ros2_camcontrol && colcon build --symlink-install && source install/setup.bash`
- 运行：`ros2 run ros2_camcontrol ws_to_image --host <android-ip> --topic /recomo/rgb --rate 10 --codec h265`

## 已完成
- 统一端口 9090；单 WS 同时承载控制/遥测/视频。
- Android：Service 管 WS+编码；Activity 管相机；显式广播命令。
- Web UI：WebCodecs 解码（Annex‑B→AVCC）；错误重置；Broadway 兜底。
- 控制（变焦、切换摄像头）与遥测链路验证通过。
- 浏览器播放与本地抓流均已验证；二进制帧正确输出。
- Orin：完成 WS 接入 + NVDEC 解码、RTSP 转发、ROS2 图像发布脚手架。

## 下一步
- Android：
  - 采集音频并封装至 MP4；配置切换握手；健壮性/退避重试。
  - 手动曝光/ISO、AF 控件；参数持久化。
- Orin：
  - `tee` 一次解码多路扇出（显示 + CUDA/DeepStream + RTSP）；重连与指标上报。
  - ROS2 走 NVMM 零拷贝路径以适配 CUDA。
- Web：
  - 显示解码器状态（WebCodecs/Broadway）、FPS、重连按钮；可选 WebRTC/MSE。

## 常见问题
- ADB 转发：`adb forward --list` 应包含 `tcp:9090 tcp:9090`。
- 代理：`localhost` 需直连；确保未被 `http_proxy`/`https_proxy` 劫持。
- 日志：`adb logcat -v time -s ControlServer:D CamControlService:D MainActivity:D`。
- WS 探测：`python scripts/ws_probe.py` 能看到二进制帧即表示在推流。
