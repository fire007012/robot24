# can_driver_ui 文档导航

主入口是 [`../README.md`](../README.md)。本目录只保留当前有效的使用说明。

## 当前文档

- [`../README.md`](../README.md)
  - 包职责、启用方式、快速开始、接口速查
- [`USAGE.md`](USAGE.md)
  - 更详细的使用步骤和界面说明
- [`../config/motors.yaml`](../config/motors.yaml)
  - UI 默认电机列表

## 运行顺序

1. 先确保 `can_driver` 或其他兼容后端已启动。
2. 启动 `ui_bridge.launch`。
3. 再启动 `keyboard_ui.launch` 或 `gui_ui.launch`。

## 备注

- 根目录存在 `CATKIN_IGNORE`，默认不会被 catkin 发现。
- 这是调试包，不建议把 UI 节点纳入正式 bringup。
