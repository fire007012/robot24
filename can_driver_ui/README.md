# can_driver_ui

`can_driver_ui` 是 `can_driver` 的调试 UI 包，提供桥接节点、终端键盘控制和 Tk GUI。它主要面向联调与排障，不是正式运行链的一部分。

## 使用前说明

本包根目录当前带有 `CATKIN_IGNORE`，默认不会参与工作区发现和编译。要启用它，先移除或改名这个文件：

```bash
mv can_driver_ui/CATKIN_IGNORE can_driver_ui/CATKIN_IGNORE.disabled
```

然后再编译：

```bash
cd ~/robot24_ws
catkin_make --pkg can_driver_ui
source devel/setup.bash
```

## 包结构

```text
can_driver_ui/
|-- config/
|   `-- motors.yaml
|-- docs/
|   |-- README.md
|   `-- USAGE.md
|-- launch/
|   |-- ui_bridge.launch
|   |-- keyboard_ui.launch
|   `-- gui_ui.launch
`-- scripts/
    |-- bridge_node.py
    |-- keyboard_ui.py
    `-- gui_ui.py
```

## 快速开始

先启动桥接：

```bash
roslaunch can_driver_ui ui_bridge.launch backend_ns:=/can_driver_node
```

终端键盘控制：

```bash
roslaunch can_driver_ui keyboard_ui.launch bridge_ns:=/can_driver_ui_bridge
```

图形界面：

```bash
roslaunch can_driver_ui gui_ui.launch bridge_ns:=/can_driver_ui_bridge
```

## 接口速查

桥接节点：

- 输入后端：
  - `backend_ns/motor_command`
  - `backend_ns/motor_states`
- 对外暴露：
  - `/can_driver_ui_bridge/motor_command`
  - `/can_driver_ui_bridge/motor_states`
  - `/can_driver_ui_bridge/motor/<name>/cmd_velocity`
  - `/can_driver_ui_bridge/motor/<name>/cmd_position`

键盘 UI：

- 参数：
  - `bridge_ns`
  - `velocity_value`
- 默认键位：
  - `w` / `s`：正反向
  - `space`：停止
  - `1..9`：切换电机
  - `n` / `p`：下一台 / 上一台
  - `e` / `d` / `x`：enable / disable / stop
  - `q`：退出

GUI：

- 支持手动指定 `bridge_ns`、service topic、command topic
- 支持按钮和键盘混合控制
- 状态来自 `/can_driver_ui_bridge/motor_states`

## 文档入口

- [`docs/README.md`](docs/README.md)
  - 文档导航与速查
- [`docs/USAGE.md`](docs/USAGE.md)
  - 更详细的 UI 使用说明

## 使用边界

- 本包不替代 `can_driver` 的正式接口，只做调试入口。
- 默认依赖 `can_driver` 已经可用，并且后端命名空间能正确解析到 `motor_command` / `motor_states`。
- 若只做标准系统联调，优先使用 `can_driver` 或 `Eyou_ROS1_Master` 的正式 launch；只有需要人工单轴试车时再启用本包。
