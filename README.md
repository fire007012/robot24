# vision_pkg

越障车视觉模块（从Qt迁移到ROS Noetic）

## 依赖安装

```bash
# 系统依赖
sudo apt-get update
sudo apt-get install -y \
    ros-noetic-usb-cam \
    ros-noetic-realsense2-camera \
    ros-noetic-cv-bridge \
    ros-noetic-image-transport \
    libopencv-dev \
    libegl1-mesa-dev \
    libgl-dev

# 或者用 rosdep 一键安装 ROS 依赖
cd ~/catkin_ws
rosdep install --from-paths src --ignore-src -r -y
```

## 编译与运行

```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
roslaunch vision_pkg vision.launch
```

## 节点说明

| 节点 | 说明 |
|------|------|
| forward_camera / back_camera / hand_camera / arm_camera | USB 摄像头（usb_cam） |
| paw_camera / behind_camera | RealSense 深度相机 |
| panorama_node | 全景拼接（EGL 离屏渲染） |
| yolov8_node | YOLOv8 目标检测（需要 model/best.onnx） |

## 注意事项

- USB 摄像头设备号（/dev/video0 等）需根据实际硬件在 launch 文件中修改
- RealSense 相机序列号同样需要在 launch 文件中修改
- YOLO 模型文件 `model/best.onnx` 需单独提供
