# Robot24 Catkin Workspace

基于 ROS Noetic 的机器人工作空间，包含底盘控制、机械臂轨迹拆分、视觉感知、MoveIt 配置与启动流程等多个包。适用于越障车与机械臂一体系统的集成开发与测试。

## 包功能概览

| 包 | 说明 |
| --- | --- |
| `robot_bringup` | 系统启动入口与常用 launch 集合 |
| `robot_moveit_config` | MoveIt 配置与规划 |
| `arm_traj_splitter` | 机械臂轨迹拆分与执行接口 |
| `can_driver` | 基于 SocketCAN 的电机驱动（ros_control） |
| `yiyou_canopen` | CANopen 相关实现与工具 |
| `can_driver_ui` | 驱动相关可视化与调试 UI |
| `car_total` | 车体 URDF 与整机描述 |
| `vision_pkg` | 视觉感知与拼接/检测节点 |
| `vision_opencv` | OpenCV 相关 ROS 包（上游） |
| `realsense-ros` | RealSense ROS 包（上游） |
| `opencv_tests` | OpenCV/视觉相关测试 |

## 目录结构

```
.
├── arm_traj_splitter/
├── can_driver/
├── can_driver_ui/
├── car_total/
├── docs/
├── realsense-ros/
├── robot_bringup/
├── robot_moveit_config/
├── vision_opencv/
├── vision_pkg/
└── yiyou_canopen/
```

## 依赖

基础环境：
- Ubuntu 20.04
- ROS Noetic
- Catkin

核心依赖（常见）：
- `ros-noetic-roscpp`
- `ros-noetic-ros-control` / `ros-noetic-ros-controllers`
- `ros-noetic-moveit`
- `ros-noetic-image-transport`
- `ros-noetic-cv-bridge`
- `ros-noetic-realsense2-camera`（使用 RealSense 时）
- `ros-noetic-socketcan-interface`（使用 `can_driver` 时）

推荐使用 `rosdep` 安装依赖：

```bash
sudo apt-get update
sudo apt-get install -y python3-rosdep
sudo rosdep init || true
rosdep update
rosdep install --from-paths src --ignore-src -r -y --rosdistro noetic
```

如果 `rosdep` 无法解析 `socketcan_interface`，可手动安装：

```bash
sudo apt-get install -y ros-noetic-socketcan-interface
```

## 编译

```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

注意事项：
- USB 摄像头设备号（如 `/dev/video0`）与 RealSense 序列号需要在对应 `launch` 中配置。
- `vision_pkg` 里如需目标检测，请提供 `model/best.onnx`。
- `can_driver` 依赖 SocketCAN 设备（`can0`/`vcan0`），请提前配置系统网络接口。

## 测试

```bash
cd ~/catkin_ws
catkin_make run_tests -j2
catkin_test_results build
```

## 文档

更多设计与测试文档见 `docs/` 目录。

当前系统不完整，无法运行完整的测试流程。如果要开发测试，请在单个包里进行。