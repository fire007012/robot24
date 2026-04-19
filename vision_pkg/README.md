# vision_pkg

视觉处理包，负责双目全景拼接、YOLOv8 检测和自定义检测消息定义。

## 包职责

- 订阅相机图像并生成全景图。
- 运行 YOLOv8 ONNX 推理并发布检测结果与标注图。
- 维护视觉模块自定义消息和第三方视觉依赖。

## 包结构

```text
vision_pkg/
|-- include/vision_pkg/
|-- launch/
|   `-- vision.launch
|-- msg/
|   `-- Detection.msg
|-- src/
|   |-- panorama*.cpp
|   |-- yolov8*.cpp
|   |-- camerainit*.cpp
|   `-- realsense*.cpp
|-- test/
`-- third_party/
```

## 快速开始

先编译包：

```bash
cd ~/robot24_ws
catkin_make --pkg vision_pkg
source devel/setup.bash
```

启动全景节点：

```bash
rosrun vision_pkg panorama_node \
  _cam1_topic:=/forward_camera/image_raw \
  _cam2_topic:=/back_camera/image_raw
```

启动 YOLOv8 节点：

```bash
rosrun vision_pkg yolov8_node \
  _model_path:=/absolute/path/best.onnx \
  _image_topic:=/forward_camera/image_raw
```

## 常用命令

```bash
# 使用包内 launch（启动前先核对设备号、RealSense 序列号和模型路径）
roslaunch vision_pkg vision.launch

# 查看检测消息
rostopic echo /yolov8/detections

# 查看标注图
rosrun rqt_image_view rqt_image_view /yolov8/detection_image

# 查看全景图
rosrun rqt_image_view rqt_image_view /panorama/panorama_image
```

## 节点与 API 摘要

- `panorama_node`
  - 订阅：相机 1、相机 2、`/joy`
  - 发布：`/panorama/panorama_image`
- `yolov8_node`
  - 订阅：`image_topic` 指定的图像
  - 发布：`/yolov8/detections`
  - 发布：`/yolov8/detection_image`
- `msg/Detection.msg`
  - 字段：`id`、`confidence`、`x`、`y`、`width`、`height`、`class_name`
- 第三方资源
  - `third_party/lib/opencv4100lib`
  - `third_party/include/librealsense2`
  - `third_party/include/glm-0.9.9.8`

## 使用说明

- `launch/vision.launch` 当前默认使用 `/dev/video0/2/4/6`，并写死了一个 RealSense 序列号；上机前先改成现场设备。
- `launch/vision.launch` 默认引用 `$(find vision_pkg)/model/best.onnx`，但仓库里当前没有 `model/` 目录；请传入实际模型路径，或先补齐该目录。
- `src/camerainit_node.cpp` 和 `src/realsense_node.cpp` 目前仍是源码资源，未在 `CMakeLists.txt` 中注册为可执行文件。

## 详细文档索引

- `README.md`：当前维护入口。
- `docs/README.md`：包内文档索引。
