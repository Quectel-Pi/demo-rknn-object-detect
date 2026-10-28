# demo-rknn-object-detect

RK3576 NPU 实时目标检测 + 多目标跟踪 demo。基于 Rockchip rknpu2 examples 的 rknn_yolov5_demo 改造，USB 摄像头实时画面 → YOLOv5s 检测 → IoUTracker 跟踪 → 屏幕显示 + 结构化事件输出。

## 平台

- SoC：Rockchip RK3576
- NPU runtime：librknnrt 2.3.2
- 模型：yolov5s-640-640.rknn（INT8 量化，COCO 80 类）
- OS：Debian 13 (trixie) / kernel 6.1

## 即用即验（不编译，直接运行预编译版本）

仓库已带预编译好的二进制和所有 .so，push 到板子直接跑。

### 1. 找到 USB 摄像头设备节点和桌面用户

```bash
adb shell 'v4l2-ctl --list-devices | grep -A1 "USB Camera"'
# 找到类似 /dev/video73 的节点

adb shell 'ls /run/user/'  # 找到桌面用户的 uid（如 1000）
# 或
adb shell 'whoami'  # 如果 adb shell 已是桌面用户
```

### 2. 推送到板子并运行

```bash
adb push install/rknn_yolov5_demo_Linux /data/local/tmp/detect/
adb shell 'chmod +x /data/local/tmp/detect/rknn_yolov5_video_demo'

# 以桌面用户身份运行（替换 <uid> 和 <device>）
adb shell "su - \$(ls /run/user | head -1) -c '
cd /data/local/tmp/detect &&
export XDG_RUNTIME_DIR=/run/user/\$(ls /run/user | head -1)
export WAYLAND_DISPLAY=wayland-0 DISPLAY=:0
export GST_PLUGIN_PATH=/usr/lib/aarch64-linux-gnu/gstreamer-1.0
export LD_LIBRARY_PATH=/data/local/tmp/detect/lib
./rknn_yolov5_video_demo model/RK3576/yolov5s-640-640.rknn --v4l2 <device> --display
'"
```

示例（摄像头 /dev/video73，桌面用户 uid=1000）：

```bash
adb shell 'su - $(id -u 1000) -c "
cd /data/local/tmp/detect &&
export XDG_RUNTIME_DIR=/run/user/1000 WAYLAND_DISPLAY=wayland-0 DISPLAY=:0
export GST_PLUGIN_PATH=/usr/lib/aarch64-linux-gnu/gstreamer-1.0
export LD_LIBRARY_PATH=/data/local/tmp/detect/lib
./rknn_yolov5_video_demo model/RK3576/yolov5s-640-640.rknn --v4l2 /dev/video73 --display
"'
```

屏幕上会看到：摄像头实时画面 + 检测框（活物红框/死物蓝框 + 类别名#track_id）+ 右侧统计面板。

## 从源码编译

需要 RK3576 SDK 的交叉编译工具链（gcc-arm-10.3 aarch64）和 buildroot sysroot（gstreamer dev）。

```bash
cd demo-rknn-object-detect
export GCC_COMPILER=<SDK>/prebuilts/gcc/linux-x86/aarch64/gcc-arm-10.3-2021.07-x86_64-aarch64-none-linux-gnu/bin/aarch64-none-linux-gnu
export PKG_CONFIG_PATH=<SDK>/buildroot/output/rockchip_rk3576/host/aarch64-buildroot-linux-gnu/sysroot/usr/lib/pkgconfig
export PKG_CONFIG_SYSROOT_DIR=<SDK>/buildroot/output/rockchip_rk3576/host/aarch64-buildroot-linux-gnu/sysroot
bash build-linux.sh -t rk3576 -a aarch64
```

产物：`install/rknn_yolov5_demo_Linux/rknn_yolov5_video_demo`

## 数据链路

```
USB 摄像头 (/dev/videoN)
  ↓ V4L2 MMAP, MJPG 1280×720@30fps
MJPG 帧
  ↓ stb_image CPU 软解 → RGB888
  ↓ cv::resize letterbox 640×640
RGB888 640×640 (model input)
  ↓ RGA → rknn_inputs_set → rknn_run (~20ms) → post_process
检测结果
  ↓ IoUTracker.update() → 分配 track_id
  ↓ 画检测框（活物红/死物蓝 + 类别标签）
  ↓ gst appsrc → waylandsink 全屏显示
屏幕实时显示
```

## 参数

| 参数 | 说明 |
|------|------|
| `<model.rknn>` | RKNN 模型路径 |
| `--v4l2 /dev/videoNN` | USB 摄像头设备节点 |
| `--display` | 实时屏幕显示（默认开） |
| `--noshow` | 关闭屏幕显示 |
| `--disp-size WxH` | 窗口尺寸（默认 1280×800 全屏） |

## 输出

### 屏幕

- 左侧：摄像头画面 + 检测框（红=活物 person/动物，蓝=死物）+ 类别名#track_id
- 右侧：半透明统计面板（TRACKS 在线数 / ONLINE COUNTS 各类计数 / TRACK LIST）

### stdout JSON 事件流

```
{"event":"frame","frame":1234,"w":1280,"h":720,"fps":15.4,"tracks":[{"id":5,"cls":"person","score":0.82,"x":120,"y":80,"w":45,"h":180}]}
{"event":"appear","id":5,"cls":"person"}
{"event":"counts","counts":{"person":3,"tv":2,"keyboard":1}}
```

- `frame`：每帧当前所有 track 列表
- `appear`：新目标出现
- `counts`：每 30 帧累计计数

## 目录结构

```
demo-rknn-object-detect/
├── src/
│   ├── main_video.cc          # 主程序：V4L2采集→解码→推理→跟踪→显示
│   ├── v4l2_capture.cpp/.h    # V4L2 MJPG 采集
│   ├── tracker.cpp/.h         # IoUTracker 跨帧跟踪+计数
│   ├── event_logger.cpp/.h    # JSON 结构化事件输出
│   ├── postprocess.cc         # YOLOv5 后处理（NMS+decode）
│   └── preprocess.cc
├── utils/
│   ├── drawing.cpp/.h         # YUV 画框
│   ├── mpp_decoder.cpp/.h     # MPP 解码器封装（H.264/HEVC，JPEG 未用）
│   └── mpp_encoder.cpp/.h     # MPP 编码器封装
├── include/                   # rknn_api.h RgaUtils.h postprocess.h 等
├── 3rdparty/                  # 自包含依赖（无需 SDK 即可编译）
│   ├── stb/                   # stb_image.h 单头 JPEG 解码
│   ├── rga/                   # RGA 头文件 + librga.so
│   ├── mpp/                   # MPP 头文件 + librockchip_mpp.so
│   ├── opencv/                # OpenCV 3.4.5 aarch64
│   ├── librknn_api/           # rknn_api.h + librknnrt.so
│   └── gst-board/             # gstreamer .so（板端运行时）
├── model/
│   ├── RK3576/yolov5s-640-640.rknn
│   ├── coco_80_labels_list.txt
│   └── bus.jpg
├── install/                   # 预编译产物（可直接 push 运行）
├── CMakeLists.txt
├── build-linux.sh
└── README.md
```

## 性能

- 采集：MJPG 1280×720@30fps
- NPU 推理：~20ms/帧
- 整体帧率：~10-15 FPS（stb_image CPU 软解是瓶颈）
- 检测目标：每帧 8-16 个

## 遗留问题

1. **stb_image CPU 软解瓶颈**：1280×720 MJPG 软解约 30-40ms/帧，限制了 FPS。优化方向：libjpeg-turbo（交叉编译 glibc 冲突）或 MPP 硬解（MppDecoder 封装不兼容 JPEG advanced 模式）。
2. **MppDecoder JPEG 路径未调通**：试了 6 种配置组合全部失败，而 RK 官方 mpi_dec_test 可解码（794 FPS）。MppDecoder 封装与 JPEG advanced 模式不兼容。

## License

Apache License 2.0（与 Rockchip rknpu2 examples 一致）
