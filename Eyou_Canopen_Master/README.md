# Eyou_Canopen_Master

> 现状说明：当前实机机械臂前两轴 `shoulder_yaw_joint`、`shoulder_pitch_joint` 已切到 `can_driver` 的 MT 后端，
> 标准整机启动请使用 `Eyou_ROS1_Master` / `robot_bringup`。
> 本包目前仅保留给历史 CANopen 调试链路和遗留场景，不再是机械臂前两轴的默认控制入口。

目录名当前是 `Eyou_Canopen_master/`，但包名与 `roslaunch` / `catkin_make --pkg` 使用的名字仍是 `Eyou_Canopen_Master`。这个包负责 CANopen 主站、`ros_control` 适配、生命周期服务和单后端 bringup。

## 包结构

```text
Eyou_Canopen_master/
|-- config/
|   |-- master.yaml / master.dcf
|   |-- joints.yaml
|   `-- master_flipper_4axis.* / joints_flipper_4axis.yaml
|-- docs/
|   |-- README.md
|   `-- archive/
|-- launch/
|   |-- canopen_hw.launch
|   |-- bringup.launch
|   |-- canopen_hw_flipper_4axis.launch
|   `-- bringup_flipper_4axis.launch
|-- scripts/
|   |-- joint_action_ui.py
|   `-- plot_ruckig_profile.py
|-- src/
|   |-- canopen_hw_ros_node.cpp
|   |-- canopen_master.cpp
|   |-- canopen_robot_hw_ros.cpp
|   |-- service_gateway.cpp
|   `-- controllers/ip_follow_joint_trajectory_executor_*.cpp
|-- srv/
|   |-- SetMode.srv
|   |-- SetZero.srv
|   `-- ApplyLimits.srv
`-- test/
    `-- test_*.cpp
```

## 快速开始

编译：

```bash
cd ~/robot24_ws
catkin_make --pkg Eyou_Canopen_Master
source devel/setup.bash
```

标准单后端 bringup（遗留 CANopen 链路）：

```bash
roslaunch Eyou_Canopen_Master bringup.launch
```

注意：该命令仅用于独立历史 CANopen 调试，不得与 `roslaunch robot_bringup full_system.launch` 或已启动的 `Eyou_ROS1_Master/hybrid_motor_hw.launch` 同时运行。

四摆臂 4 轴 bringup：

```bash
roslaunch Eyou_Canopen_Master bringup_flipper_4axis.launch
```

注意：该命令同样属于遗留单后端入口；若整车已经走 hybrid/full_system 基线，不要再额外启动这条 4 轴 CANopen 链路。

启用 IP executor（仅遗留 CANopen 场景）：

```bash
roslaunch Eyou_Canopen_Master bringup.launch \
  use_ip_executor:=true \
  ip_executor_action_ns:=arm_position_controller/follow_joint_trajectory
```

## 常用命令

生命周期：

```bash
rosservice call /canopen_hw_node/init "{}"
rosservice call /canopen_hw_node/enable "{}"
rosservice call /canopen_hw_node/halt "{}"
rosservice call /canopen_hw_node/resume "{}"
rosservice call /canopen_hw_node/recover "{}"
rosservice call /canopen_hw_node/shutdown "{}"
```

维护服务：

```bash
rosservice call /canopen_hw_node/set_mode "{axis_index: 0, mode: 8}"
rosservice call /canopen_hw_node/set_zero "{axis_index: 0, zero_offset_rad: 0.0, use_current_position_as_zero: true}"
rosservice call /canopen_hw_node/apply_limits "{axis_index: 0, min_position: -1.0, max_position: 1.0, use_urdf_limits: false, require_current_inside_limits: true}"
```

调试：

```bash
rosservice list | grep canopen_hw_node
rostopic echo /diagnostics
rosrun Eyou_Canopen_Master joint_action_ui.py
```

## 接口速查

节点：

- `canopen_hw_node`
  - launch 中实际运行的 ROS 节点是 `canopen_hw_ros_node`

服务：

- `~init`
- `~enable`
- `~disable`
- `~halt`
- `~resume`
- `~recover`
- `~shutdown`
- `~set_mode`
- `~set_zero`
- `~apply_limits`

关键配置：

- `config/master.yaml` / `config/master.dcf`
  - 主站 DCF 与总线配置
- `config/joints.yaml`
  - 关节与限位配置
- `launch/bringup.launch`
  - 正式单后端 bringup
- `launch/bringup_flipper_4axis.launch`
  - 四摆臂专用 bringup

关键能力：

- `ros_control` 适配与 controller_manager
- IP `FollowJointTrajectory` executor
- PDO 映射校验
- 生命周期状态机与辅助维护服务

## 使用边界

- 如果系统已经切到统一外观层，优先使用 `Eyou_ROS1_Master`，不要在上层直接绕过 facade 调底层。
- 若已使用 `robot_bringup/full_system.launch` 或 `Eyou_ROS1_Master/hybrid_motor_hw.launch`，不要再并发启动本包任一 bringup/canopen_hw launch，也不要再执行 `/canopen_hw_node/*` 生命周期命令。
- 机械臂前两轴当前由 `can_driver` 直接托管；不要再用本包驱动 `shoulder_yaw_joint`、`shoulder_pitch_joint` 实机联调。
- `joint_action_ui.py` 适合动作 / 轨迹链路验证，不覆盖 `flipper_control` 的 `csv_velocity` 调试路径。
- `docs/archive/` 里保留大量历史草案与 bug 记录，当前执行基线以顶层 README 与现行文档为准。

## 文档入口

- [`docs/README.md`](docs/README.md)
  - 当前文档导航、命令速查和归档说明
