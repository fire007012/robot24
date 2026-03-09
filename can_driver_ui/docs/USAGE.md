# can_driver_ui 使用指南

本文档面向日常联调，目标是让你快速把 UI 与 `can_driver` 对接起来，并可手动验证每个电机。

## 1. 包作用

`can_driver_ui` 提供两类节点：

1. `bridge_node.py`
- 统一 UI 入口，转发 service/topic 到 `can_driver`
- 做每轴命令限幅（来自 `config/motors.yaml`）

2. `keyboard_ui.py`
- 终端键盘小 UI（`w/s` 正反转）
- 可选择控制电机、调用使能/失能/急停

---

## 2. 文件结构

- `can_driver_ui/scripts/bridge_node.py`
- `can_driver_ui/scripts/keyboard_ui.py`
- `can_driver_ui/launch/ui_bridge.launch`
- `can_driver_ui/launch/keyboard_ui.launch`
- `can_driver_ui/config/motors.yaml`
- `can_driver_ui/docs/README.md`

---

## 3. 先决条件

1. `can_driver` 已能启动。
2. `can_driver` 对外至少有：
- `motor_command` service
- `motor_states` topic
- `/motor/<joint>/cmd_velocity` 或 `/motor/<joint>/cmd_position`

3. 已完成编译并 `source`：

```bash
cd /home/dianhua/catkin_ws
catkin_make --pkg can_driver can_driver_ui
source devel/setup.bash
```

---

## 4. 配置电机清单

编辑：`can_driver_ui/config/motors.yaml`

示例：

```yaml
motors:
  - name: left_wheel
    motor_id: 321
    mode: velocity
    min: -200
    max: 200
    step: 5
  - name: rotary_table
    motor_id: 6
    mode: position
    min: -2000
    max: 2000
    step: 20
```

字段说明：
- `name`: joint 名称（必须与 `can_driver` 侧一致）
- `motor_id`: service 控制用 ID
- `mode`: `velocity` 或 `position`
- `min/max`: UI 与 bridge 限幅
- `step`: UI 层建议步进（当前键盘节点未直接使用）

---

## 5. 启动顺序（推荐）

1. 启动 `can_driver`：

```bash
roslaunch can_driver can_driver.launch
```

2. 启动 bridge：

```bash
roslaunch can_driver_ui ui_bridge.launch
```

3. 启动键盘 UI（可选）：

```bash
roslaunch can_driver_ui keyboard_ui.launch
```

---

## 6. UI 侧对接接口（统一走 bridge）

假设 bridge 节点名是 `can_driver_ui_bridge`：

1. service
- `/can_driver_ui_bridge/motor_command`

2. 状态 topic
- `/can_driver_ui_bridge/motor_states`

3. 直接命令 topic
- `/can_driver_ui_bridge/motor/<name>/cmd_velocity`
- `/can_driver_ui_bridge/motor/<name>/cmd_position`

建议：前端只使用以上接口，不直接连后端原始 topic/service。

---

## 7. 键盘 UI 使用

运行：

```bash
roslaunch can_driver_ui keyboard_ui.launch
```

按键：
- `w`: 正向命令
- `s`: 反向命令
- `space`: 发送 0
- `1..9`: 选择第 N 个电机
- `n/p`: 下一个/上一个电机
- `e`: 使能选中电机
- `d`: 失能选中电机
- `x`: 急停选中电机（service CMD_STOP）
- `q`: 退出

说明：
- `w/s` 数值默认是 `30.0`，可通过 launch 参数改：

```bash
roslaunch can_driver_ui keyboard_ui.launch velocity_value:=50.0
```

---

## 8. 常用验证命令

1. 看 bridge 是否启动：

```bash
rosnode list | grep can_driver_ui_bridge
```

2. 看桥接接口是否存在：

```bash
rosservice list | grep can_driver_ui_bridge
rostopic list | grep can_driver_ui_bridge
```

3. 手动测试 service：

```bash
rosservice call /can_driver_ui_bridge/motor_command "{motor_id: 321, command: 0, value: 0.0}"
```

4. 手动发命令 topic：

```bash
rostopic pub -1 /can_driver_ui_bridge/motor/left_wheel/cmd_velocity std_msgs/Float64 "data: 20.0"
```

5. 看状态：

```bash
rostopic echo /can_driver_ui_bridge/motor_states
```

---

## 9. 故障排查

1. 报 `service timeout`
- 后端 `can_driver` 可能没启动，或 `motor_command` 名字不一致。
- 先用 `rosservice list | grep motor_command` 查实际名称。

2. 能发命令但电机不动
- 未使能（先 `e` 或 CMD_ENABLE）
- `motors.yaml` 的 `name/motor_id/mode` 与后端不一致
- 后端 controller/direct 命令冲突

3. 没有状态更新
- 后端 `motor_states` 本身没数据
- bridge `backend_ns` 配错（如后端接口在命名空间下）

4. 键盘按键无效
- 键盘节点需在前台终端运行
- 终端焦点不在该窗口时不会读键

---

## 10. 与 MoveIt / Navi 的关系

`can_driver_ui` 只负责 UI 与单轴调试，不决定 controller 选择。

controller 由这些文件决定：
- `can_driver/config/can_driver.yaml`（关节接口类型）
- `can_driver/config/ros_controllers.yaml`（控制器定义）
- `can_driver/launch/can_driver.launch`（spawner 实际加载）

当前你的默认配置下，通常只自动启动 `joint_state_controller`。

---

## 11. 安全建议

1. 首次联调只用小命令值。
2. 每次测试后发 0 或 STOP。
3. 不要同时用 MoveIt/Navi 与 direct topic 控同一电机。
4. UI 页面保留常驻“总急停”入口。
