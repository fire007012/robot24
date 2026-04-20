# vision_pkg —— 越障车视觉模块（ROS1 Noetic）

## 项目简介

vision_pkg 是一个运行在 **Ubuntu 20.04 + ROS1 Noetic** 上的视觉处理模块，为越障车提供以下功能：

- **综合视觉显示**：一个节点集成 YOLOv8 检测 + 障碍物预警 + 距离显示，细白线辅助框设计
- **YOLOv8 目标检测**：基于 OpenCV DNN 加载 ONNX 模型，检测置信度阈值 0.7
- **物体 3D 位姿估计**：结合检测结果与深度图，计算物体 3D 位置（含深度滤波 + EMA 平滑）
- **障碍物预警**：深度图三区域（左/中/右）最近距离检测，细白线分割，距离文字显示
- **中心距离显示**：细白线十字准星，实时显示画面中心深度距离
- **双鱼眼全景拼接**：前后鱼眼相机校正并拼合为等距柱状投影全景图
- **球面全景渲染**：EGL 离屏渲染，支持手柄交互控制视角
- **RealSense D405**：paw_camera 提供彩色图、深度图和点云数据
- **Behind Camera**：支持 Astra+ 或 RealSense D435i（可通过 launch 参数或环境变量指定；也可用外部脚本自动检测后传参）
- **多路 USB 摄像头**：支持 4 路 USB 摄像头采集
- **热成像相机**：Xtherm T2S+ 热成像（独立包 `thermal_camera`，默认随 vision.launch 启动）

---

## 项目结构

```
vision_pkg/
├── CMakeLists.txt                          # 构建配置
├── package.xml                             # ROS 包描述
├── README.md                               # 本文件
├── include/vision_pkg/
│   ├── camerainit.h                        # USB 摄像头采集类（V4L2）
│   ├── realsense.h                         # RealSense 深度相机采集类
│   ├── fisheye.h                           # 鱼眼畸变校正与全景转换
│   ├── panorama.h                          # 全景处理管线（鱼眼校正 → 球面渲染）
│   ├── panoramasimulator.h                 # EGL 离屏球面全景渲染器
│   ├── yolov8.h                            # YOLOv8 目标检测封装
│   └── yolov8_utils.h                      # 检测结果数据结构与绘图工具
├── src/
│   ├── yolov8.cpp                          # YOLOv8 检测实现（LetterBox + NMS）
│   ├── yolov8_node.cpp                     # YOLOv8 ROS 节点
│   ├── object_pose_node.cpp                # 物体 3D 位姿估计节点
│   ├── distance_display_node.cpp           # 中心距离显示节点
│   ├── obstacle_warning_node.cpp           # 前方障碍物预警节点
│   ├── fisheye.cpp                         # 鱼眼校正实现
│   ├── panorama.cpp                        # 全景管线实现
│   ├── panoramasimulator.cpp               # EGL/OpenGL 球面渲染实现
│   ├── panorama_node.cpp                   # 全景处理 ROS 节点
│   └── vision_display_node.cpp             # 综合视觉显示节点
├── scripts/
│   └── detect_behind_camera.py             # behind_camera 类型自动检测脚本
├── launch/
│   └── vision.launch                       # 一键启动所有节点
├── msg/
│   ├── Detection.msg                       # 自定义检测结果消息
│   └── ObstacleWarning.msg                 # 障碍物预警消息
├── model/
│   └── best.onnx                           # YOLOv8 ONNX 模型文件
└── third_party/                            # 第三方头文件库（需手动放置）
    └── include/
        ├── glm-0.9.9.8/                    # GLM 数学库（header-only）
        └── Eigen/                           # Eigen 矩阵库（header-only）

thermal_camera/                              # 热成像独立包
├── CMakeLists.txt
├── package.xml
├── setup.py
├── scripts/
│   └── thermal_camera_node.py              # ROS 热成像节点
├── launch/
│   └── thermal_camera.launch
└── src/                                     # IR-Py-Thermal 库源码
    ├── irpythermal.py
    ├── pyplot.py
    ├── opencv.py
    ├── display.py
    ├── utils.py
    └── example_simple.py
```

---

## 一键启动

```bash
source ~/catkin_ws/devel/setup.bash
roslaunch vision_pkg vision.launch
启动请检查behind_camera位置的相机是realsense还是astra，默认是realsense，需更改请参考下面的launch参数配置
```

### launch 参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `enable_usb_cams` | `false` | 启用 4 路 USB 摄像头 |
| `enable_paw_camera` | `true` | 启用 RealSense D405 |
| `enable_behind_camera` | `true` | 启用 behind_camera |
| `enable_thermal` | `true` | 启用热成像 |
 | `behind_camera_type` | `realsense` | Behind 相机类型：`astra` 或 `realsense`（可用环境变量 `BEHIND_CAMERA_TYPE` 覆盖） |
| `paw_serial` | `130322273001` | RealSense D405 序列号 |
| `behind_serial` | `219323070286` | Astra+ 序列号 |
| `behind_realsense_serial` | `""` | D435i 序列号（需手动填写） |

```bash
# 示例：关闭热成像
roslaunch vision_pkg vision.launch enable_thermal:=false

# 手动指定 behind_camera 为 Astra+
roslaunch vision_pkg vision.launch behind_camera_type:=astra

# 手动指定 behind_camera 为 RealSense D435i
roslaunch vision_pkg vision.launch behind_camera_type:=realsense behind_realsense_serial:=XXXXXX

# 用环境变量指定（适合两台工控机固定配置）
export BEHIND_CAMERA_TYPE=astra 或 export BEHIND_CAMERA_TYPE=realsense
roslaunch vision_pkg vision.launch
```

---

## 所有话题一览 & 订阅方法

### 驱动层话题

#### 1. RealSense D405 (paw_camera)

| 话题 | 消息类型 | 说明 |
|------|---------|------|
| `/paw_camera/color/image_raw` | `sensor_msgs/Image` | 彩色图像 |
| `/paw_camera/depth/image_rect_raw` | `sensor_msgs/Image` | 深度图像 |
| `/paw_camera/color/camera_info` | `sensor_msgs/CameraInfo` | 相机内参 |
| `/paw_camera/depth/color/points` | `sensor_msgs/PointCloud2` | 彩色点云 |

```bash
# 查看彩色图像
rqt_image_view /paw_camera/color/image_raw

# 查看深度图
rqt_image_view /paw_camera/depth/image_rect_raw

# RViz 查看点云
rviz   # 添加 PointCloud2，话题选 /paw_camera/depth/color/points
```

#### 2. Behind Camera (Astra+ 或 D435i，按配置选择)

**当 behind_camera 为 Astra+ 时：**

| 话题 | 消息类型 | 说明 |
|------|---------|------|
| `/behind_camera/color/image_raw` | `sensor_msgs/Image` | 彩色图像 |
| `/behind_camera/depth/image_raw` | `sensor_msgs/Image` | 深度图像 |
| `/behind_camera/depth/points` | `sensor_msgs/PointCloud2` | 点云 |

**当 behind_camera 为 RealSense D435i 时：**

| 话题 | 消息类型 | 说明 |
|------|---------|------|
| `/behind_camera/color/image_raw` | `sensor_msgs/Image` | 彩色图像 |
| `/behind_camera/depth/image_rect_raw` | `sensor_msgs/Image` | 深度图像 |
| `/behind_camera/depth/color/points` | `sensor_msgs/PointCloud2` | 彩色点云 |

```bash
# 查看 behind_camera 彩色图像
rqt_image_view /behind_camera/color/image_raw

# 查看深度图
rqt_image_view /behind_camera/depth/image_raw
```

#### 3. USB 摄像头（默认关闭，需 `enable_usb_cams:=true`）

| 话题 | 消息类型 | 说明 |
|------|---------|------|
| `/forward_camera/image_raw` | `sensor_msgs/Image` | 前方摄像头 |
| `/back_camera/image_raw` | `sensor_msgs/Image` | 后方摄像头 |
| `/hand_camera/image_raw` | `sensor_msgs/Image` | 手部摄像头 |
| `/arm_camera/image_raw` | `sensor_msgs/Image` | 机械臂摄像头 |

```bash
roslaunch vision_pkg vision.launch enable_usb_cams:=true
rqt_image_view /forward_camera/image_raw
```

#### 4. 热成像 Xtherm T2S+

| 话题 | 消息类型 | 说明 |
|------|---------|------|
| `/thermal_camera/image_raw` | `sensor_msgs/Image` | 伪彩色热成像图像（plasma colormap，1024x768） |

```bash
# 查看热成像画面
rqt_image_view /thermal_camera/image_raw

# 单独启动热成像
roslaunch thermal_camera thermal_camera.launch

# 热成像参数
#   ~device       : 设备路径（空=自动检测）
#   ~raw_mode     : RAW 模式（默认 true，T2S+ V2 必须开启）
#   ~frame_rate   : 帧率（默认 25）
#   ~temp_offset  : 温度偏移（默认 0.0）
#   ~upscale      : 放大倍数（默认 4，原始 256x192 → 1024x768）
#   ~colormap     : matplotlib colormap（默认 plasma）
```

### 功能层话题

#### 5. vision_display_node（综合视觉显示）

运行两个实例：`paw_vision`（paw_camera）和 `behind_vision`（behind_camera）。

**paw_vision：**

| 话题 | 消息类型 | 方向 | 说明 |
|------|---------|------|------|
| `/paw_camera/color/image_raw` | `sensor_msgs/Image` | 订阅 | 彩色图像输入 |
| `/paw_camera/depth/image_rect_raw` | `sensor_msgs/Image` | 订阅 | 深度图像输入 |
| `/paw_vision/vision_image` | `sensor_msgs/Image` | 发布 | YOLO标注 + 障碍物预警 + 距离显示 |
| `/paw_vision/detections` | `vision_pkg/Detection` | 发布 | 检测结果 |
| `/paw_vision/obstacle_warning` | `vision_pkg/ObstacleWarning` | 发布 | 障碍物预警 |

**behind_vision：**

| 话题 | 消息类型 | 方向 | 说明 |
|------|---------|------|------|
| `/behind_camera/color/image_raw` | `sensor_msgs/Image` | 订阅 | 彩色图像输入 |
| `/behind_camera/depth/image_raw` | `sensor_msgs/Image` | 订阅 | 深度图像输入 |
| `/behind_vision/vision_image` | `sensor_msgs/Image` | 发布 | YOLO标注 + 障碍物预警 + 距离显示 |
| `/behind_vision/detections` | `vision_pkg/Detection` | 发布 | 检测结果 |
| `/behind_vision/obstacle_warning` | `vision_pkg/ObstacleWarning` | 发布 | 障碍物预警 |

```bash
# 查看 paw_camera 综合视觉画面（YOLO + 障碍物预警 + 距离）
rqt_image_view /paw_vision/vision_image

# 查看 behind_camera 综合视觉画面
rqt_image_view /behind_vision/vision_image

# 订阅检测结果
rostopic echo /paw_vision/detections

# 订阅障碍物预警
rostopic echo /paw_vision/obstacle_warning
```

#### 6. object_pose_node（物体 3D 位姿估计）

| 话题 | 消息类型 | 方向 | 说明 |
|------|---------|------|------|
| `/paw_vision/detections` | `vision_pkg/Detection` | 订阅 | 检测结果输入 |
| `/paw_camera/depth/image_rect_raw` | `sensor_msgs/Image` | 订阅 | 深度图输入 |
| `/paw_camera/color/camera_info` | `sensor_msgs/CameraInfo` | 订阅 | 相机内参 |
| `/detected_object_pose` | `geometry_msgs/PoseStamped` | 发布 | 物体 3D 位姿（20Hz） |

```bash
# 订阅物体 3D 位姿
rostopic echo /detected_object_pose

# 在 RViz 中可视化位姿
rviz   # 添加 Pose 显示，话题选 /detected_object_pose
```

#### 7. panorama_node（全景拼接）

| 话题 | 消息类型 | 方向 | 说明 |
|------|---------|------|------|
| `/forward_camera/image_raw` | `sensor_msgs/Image` | 订阅 | 前方鱼眼图像 |
| `/back_camera/image_raw` | `sensor_msgs/Image` | 订阅 | 后方鱼眼图像 |
| `/joy` | `sensor_msgs/Joy` | 订阅 | 手柄输入（右摇杆控制视角） |
| `/panorama/panorama_image` | `sensor_msgs/Image` | 发布 | 球面渲染全景图 |

```bash
# 查看全景图（需先启用 USB 摄像头）
roslaunch vision_pkg vision.launch enable_usb_cams:=true
rqt_image_view /panorama/panorama_image
```

---

## 话题速查表

```bash
# ===== 驱动层 =====
/paw_camera/color/image_raw            # D405 彩色图
/paw_camera/depth/image_rect_raw       # D405 深度图
/paw_camera/depth/color/points         # D405 点云
/behind_camera/color/image_raw         # Astra+/D435i 彩色图
/behind_camera/depth/image_raw         # Astra+/D435i 深度图
/forward_camera/image_raw              # USB 前方（需 enable）
/back_camera/image_raw                 # USB 后方（需 enable）
/hand_camera/image_raw                 # USB 手部（需 enable）
/arm_camera/image_raw                  # USB 机臂（需 enable）
/thermal_camera/image_raw              # 热成像伪彩色

# ===== 功能层 =====
/paw_vision/vision_image               # paw 综合视觉画面
/paw_vision/detections                 # paw YOLO 检测结果
/paw_vision/obstacle_warning           # paw 障碍物预警
/behind_vision/vision_image            # behind 综合视觉画面
/behind_vision/detections              # behind YOLO 检测结果
/behind_vision/obstacle_warning        # behind 障碍物预警
/detected_object_pose                  # 物体 3D 位姿
/panorama/panorama_image               # 全景拼接画面
```

**快速查看所有话题：**
```bash
rostopic list
rostopic list | grep image    # 只看图像话题
rostopic list | grep points   # 只看点云话题
```

---

## 自定义消息

### Detection.msg

```
int32   id
float32 confidence
int32   x
int32   y
int32   width
int32   height
string  class_name
```

### ObstacleWarning.msg

```
float32 left_dist
float32 center_dist
float32 right_dist
bool    left_warn
bool    center_warn
bool    right_warn
```

---

## 环境要求

| 项目 | 版本要求 |
|------|---------|
| 操作系统 | Ubuntu 20.04 |
| ROS | ROS1 Noetic（鱼香一键安装） |
| OpenCV | **4.10**（需从源码编译，替换系统自带 4.2） |
| librealsense2 | v2.50.0（paw_camera 使用） |
| realsense-ros | ROS1 分支（源码编译，放入 catkin_ws） |
| astra_camera | OrbbecSDK_ROS1 v1.5.8+（源码编译，behind_camera 使用） |
| scikit-image | `pip3 install scikit-image`（热成像节点使用） |
| matplotlib | 系统自带即可（热成像节点使用） |

---

## 安装步骤

### 第一步：安装 ROS Noetic（鱼香一键脚本）

如果尚未安装 ROS：

```bash
wget http://fishros.com/install -O fishros && . fishros
```

按提示选择安装 ROS1 Noetic。

### 第二步：安装基础依赖

```bash
sudo apt update
sudo apt install -y \
    build-essential cmake git pkg-config \
    libegl1-mesa-dev libgl1-mesa-dev \
    libusb-1.0-0-dev libssl-dev libgtk-3-dev \
    libglfw3-dev libglu1-mesa-dev \
    python3-catkin-tools \
    ros-noetic-image-transport \
    ros-noetic-cv-bridge \
    ros-noetic-usb-cam \
    ros-noetic-joy \
    ros-noetic-ddynamic-reconfigure \
    ros-noetic-rgbd-launch \
    ros-noetic-backward-ros \
    libopenni2-dev \
    libuvc-dev

pip3 install scikit-image
```

### 第三步：从源码编译安装 OpenCV 4.10

卸载系统自带 OpenCV 4.2 并编译安装 4.10：

```bash
# 安装编译依赖
sudo apt install -y \
    libavcodec-dev libavformat-dev libswscale-dev \
    libv4l-dev libxvidcore-dev libx264-dev \
    libjpeg-dev libpng-dev libtiff-dev \
    libatlas-base-dev gfortran \
    python3-numpy

# 下载 OpenCV 4.10 源码
cd ~
git clone -b 4.10.0 --depth 1 https://github.com/opencv/opencv.git
git clone -b 4.10.0 --depth 1 https://github.com/opencv/opencv_contrib.git

# 编译安装
cd ~/opencv
mkdir build && cd build
cmake -D CMAKE_BUILD_TYPE=RELEASE \
      -D CMAKE_INSTALL_PREFIX=/usr/local \
      -D OPENCV_EXTRA_MODULES_PATH=~/opencv_contrib/modules \
      -D WITH_CUDA=OFF \
      -D BUILD_EXAMPLES=OFF \
      -D BUILD_TESTS=OFF \
      -D BUILD_PERF_TESTS=OFF \
      -D BUILD_opencv_python2=OFF \
      -D BUILD_opencv_python3=ON \
      ..

make -j$(nproc)
sudo make install
sudo ldconfig
echo 'export OpenCV_DIR=/usr/local/lib/cmake/opencv4' >> ~/.bashrc
source ~/.bashrc
```

> **注意**：编译安装 OpenCV 4.10 后，还需要重新编译 `cv_bridge` 以匹配新版本 OpenCV，否则 catkin_make 时可能出现链接错误。方法如下：

```bash
cd ~/catkin_ws/src
git clone -b noetic https://github.com/ros-perception/vision_opencv.git
```

这样 `cv_bridge` 会在工作空间内从源码编译，自动链接到新版 OpenCV。

### 第四步：安装 Intel RealSense SDK（librealsense2 v2.50.0）
从 Intel apt 源安装的 `librealsense2-dev` 的 cmake 配置中引用了 `fastcdr` 和 `fastrtps`（ROS2 DDS 依赖），
 会导致 `catkin_make` 时报错：
 The following imported targets are referenced, but are missing: fastcdr fastrtps
>解决办法：卸载 apt 版本，改用源码编译安装 librealsense2：
```
 # 卸载已有 apt 版本
 sudo apt remove librealsense2-dev librealsense2 librealsense2-utils
 sudo apt autoremove

 # 安装编译依赖
 sudo apt install -y git libssl-dev libusb-1.0-0-dev pkg-config libgtk-3-dev \
   libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev

 # 源码编译（v2.50.0）
 cd ~
 git clone https://github.com/IntelRealSense/librealsense.git
 cd librealsense
 git checkout v2.50.0
 mkdir build && cd build
 cmake .. -DCMAKE_BUILD_TYPE=Release -DBUILD_EXAMPLES=false
 make -j$(nproc)
 sudo make install
```

### 第五步：获取 realsense-ros 源码包

将自己的 realsense-ros 包（ROS1 分支）放到工作空间中：

```bash
cd ~/catkin_ws/src

# 克隆 realsense-ros（ROS1 分支）
git clone https://github.com/IntelRealSense/realsense-ros.git
cd realsense-ros
git checkout `git tag | sort -V | grep -P "^2.\d+\.\d+" | tail -1`
cd ..
```

> 如果你已经有自己的 realsense-ros 包，直接将其复制到 `~/catkin_ws/src/` 下即可。

### 第五步 b：获取 Astra+ 相机驱动（behind_camera 使用）

**重要**：必须使用 v1.5.8 版本，v2.x 不支持 Astra+。

```bash
cd ~/catkin_ws/src
git clone --depth 1 --branch v1.5.8 https://github.com/orbbec/OrbbecSDK_ROS1.git
cd OrbbecSDK_ROS1
git checkout v1.5.8
cd ~/catkin_ws/src
```

安装 udev 规则（否则普通用户无权限访问设备）：

```bash
cd ~/catkin_ws/src/OrbbecSDK_ROS1/scripts
sudo cp 99-obsensor-ros1-libusb.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger
```

### 第六步：放置第三方头文件库

在 vision_pkg 目录下创建 third_party 并放入 GLM 和 Eigen：

```bash
cd ~/catkin_ws/src/vision_pkg
mkdir -p third_party/include

# GLM
cd /tmp
wget https://github.com/g-truc/glm/releases/download/0.9.9.8/glm-0.9.9.8.zip
unzip glm-0.9.9.8.zip -d glm-0.9.9.8
cp -r glm-0.9.9.8 ~/catkin_ws/src/vision_pkg/third_party/include/glm-0.9.9.8

# Eigen（也可使用 apt 安装后创建软链接）
sudo apt install -y libeigen3-dev
ln -s /usr/include/eigen3/Eigen ~/catkin_ws/src/vision_pkg/third_party/include/Eigen
```

### 第七步：编译工作空间

```bash
cd ~/catkin_ws

# 确保 source 了 ROS 环境
source ~/catkin_ws/devel/setup.bash

catkin_make -DCATKIN_ENABLE_TESTING=False -DCMAKE_BUILD_TYPE=Release
```

编译成功后，配置环境变量：

```bash
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

---

## 运行

### 一键启动所有节点

```bash
source ~/catkin_ws/devel/setup.bash
roslaunch vision_pkg vision.launch
```

这将同时启动：
- RealSense 深度相机 paw_camera（含点云）
- Behind Camera（Astra+ 或 D435i，自动检测）
- 热成像 Xtherm T2S+
- YOLOv8 综合视觉显示节点（paw_vision + behind_vision）
- 物体 3D 位姿估计节点
- 全景拼接与渲染节点
- 4 路 USB 摄像头（默认关闭，需设置 `enable_usb_cams:=true`）

---

## 注意事项

1. **OpenCV 版本**：本项目要求 OpenCV 4.10，系统自带的 4.2 版本不满足需求。编译安装后务必确认 `cv_bridge` 也使用新版本。
2. **RealSense 序列号**：launch 文件中 paw_camera 的 `serial_no` 需要替换为实际设备序列号，可通过 `rs-enumerate-devices | grep Serial` 获取。
3. **Astra+ 序列号**：launch 文件中 behind_camera 的 `serial_number` 需要替换为实际设备序列号。
4. **Behind Camera** 选择方式：vision.launch 通过 behind_camera_type 选择 astra 或 realsense。可手动传参
    （behind_camera_type:=astra/realsense）或设置环境变量 BEHIND_CAMERA_TYPE。若需自动检测，请在外部脚本检测后再传给
    roslaunch。
5. **USB 摄像头设备路径**：launch 文件中的设备路径需要根据实际硬件连接情况调整，可通过 `v4l2-ctl --list-devices` 查看可用设备。
6. **EGL 离屏渲染**：全景模块使用 EGL PBuffer 进行无显示器离屏渲染，需确保安装了 Mesa EGL 驱动（`libegl1-mesa-dev`）。
7. **手柄控制**：全景视角控制需要连接手柄并安装 `ros-noetic-joy` 包。
8. **Astra+ USB 规则**：首次使用 Astra+ 需要配置 udev 规则，否则可能无权限访问设备，详见安装步骤。
9. **热成像依赖**：热成像节点需要 `scikit-image` 和 `matplotlib`，通过 `pip3 install scikit-image` 安装。

