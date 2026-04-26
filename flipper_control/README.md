# flipper_control

`flipper_control` 是四摆臂上层控制包，负责把北向轨迹/点动命令转换成底层控制器可执行的 `CSP` 或 `CSV` 指令，并管理档位切换、联动展开、参考生成与运行态发布。

## 包结构

```text
flipper_control/
|-- config/
|   `-- flipper_control.yaml          # 关节名、控制器名、时序与后端参数
|-- docs/
|   |-- README.md                     # 补充文档导航
|   `-- flipper_motor_debug_ui.md     # 调试 UI 使用说明
|-- include/flipper_control/
|   `-- flipper_reference_generator.hpp
|-- launch/
|   `-- flipper_control.launch        # 标准启动入口
|-- msg/
|   `-- FlipperControlState.msg       # 运行态汇总消息
|-- scripts/
|   `-- flipper_motor_debug_ui.py     # 调试 UI
|-- src/
|   |-- flipper_manager_node.cpp      # 模式管理、切换、接口编排
|   `-- flipper_reference_generator.cpp
|-- srv/
|   |-- SetControlProfile.srv         # 切换 csp_position/csp_jog/csv_velocity
|   `-- SetLinkageMode.srv            # 切换摆臂联动模式
`-- test/
    `-- test_flipper_reference_generator.cpp
```

## 控制链路

```text
trajectory / jog cmd
        |
        v
flipper_manager_node
  - profile 管理
  - 联动模式展开
  - q_ref / vel_ref 生成
  - CSP <-> CSV 冷切换
        |
        +--> /flipper_csp_controller/command
        `--> /flipper_csv_controller/command
                |
          `--> hybrid backend: /hybrid_motor_hw_node
```

## 快速开始

编译：

```bash
catkin build flipper_control
```

默认启动：

```bash
roslaunch flipper_control flipper_control.launch
```

全车联调，显式指定 hybrid 命名空间：

```bash
roslaunch flipper_control flipper_control.launch \
  hybrid_ns:=/hybrid_motor_hw_node
```

打开调试 UI：

```bash
rosrun flipper_control flipper_motor_debug_ui.py \
  --flipper-ns /flipper_control \
  --hybrid-ns /hybrid_motor_hw_node
```

调试 UI 当前只面向 hybrid 链路：

- 读取 `/hybrid_motor_hw_node/joint_runtime_states`，显示真实生命周期、online/enabled/fault。
- 显示 `joint_state_controller` / `flipper_csp_controller` / `flipper_csv_controller` 的当前状态，便于确认命令是否经过控制器。

## 接口速查

节点：

- `flipper_control`
  - 可执行体：`flipper_manager_node`

订阅：

- `~command` `trajectory_msgs/JointTrajectory`
  - 仅 `csp_position` 档位接受。
- `~jog_cmd` `control_msgs/JointJog`
  - `csp_jog` 和 `csv_velocity` 档位接受。
- `/joint_states` `sensor_msgs/JointState`
  - 用于测量值与参考初始化。
- `~runtime_state_topic`
  - 读取 `Eyou_ROS1_Master/JointRuntimeStateArray`。

发布：

- `~state` `flipper_control/FlipperControlState`
- `~active_profile` `std_msgs/String`
- `/<csp_controller>/command` `trajectory_msgs/JointTrajectory`
- `/<csv_controller>/command` `trajectory_msgs/JointTrajectory`

服务：

- `~set_control_profile`
  - 输入：`csp_position | csp_jog | csv_velocity`
- `~set_linkage_mode`
  - 输入：`independent | left_right_mirror | front_rear_sync | side_pair | diagonal_pair`

消息与服务定义：

- `msg/FlipperControlState.msg`
- `srv/SetControlProfile.srv`
- `srv/SetLinkageMode.srv`

## 常用命令

切到 `csv_velocity`：

```bash
rosservice call /flipper_control/set_control_profile "{profile: 'csv_velocity'}"
```

切回 `csp_position`：

```bash
rosservice call /flipper_control/set_control_profile "{profile: 'csp_position'}"
```

切换联动模式：

```bash
rosservice call /flipper_control/set_linkage_mode "{mode: 'left_right_mirror'}"
```

发送 `JointJog`：

```bash
rostopic pub -1 /flipper_control/jog_cmd control_msgs/JointJog \
"{
  joint_names: ['left_front_arm_joint','right_front_arm_joint'],
  velocities: [0.2, 0.2],
  duration: 0.2
}"
```

查看运行态：

```bash
rostopic echo /flipper_control/state
```

## 参数重点

- `joint_names`
  - 摆臂关节顺序，默认 4 个关节。
- `direction_corrections`
  - 按关节名配置方向修正乘子；默认全为 `1.0`，需要反向的关节设为 `-1.0`。
  - `flipper_control` 会在 `/joint_states`、`~command`、`~jog_cmd` 与下游控制器命令之间自动做双向换算。
- `controllers/csp` / `controllers/csv`
  - 输出控制器名，最终发布到 `/<controller>/command`。
- `initial_profile`
  - 初始档位，默认 `csp_position`。
- `initial_linkage_mode`
  - 初始联动模式，默认 `independent`。
- `controller_manager_ns`
  - 控制器切换服务命名空间。
- `hybrid_ns`
  - 南向后端命名空间。
- `control_loop_hz` / `state_publish_hz`
  - 控制循环与状态发布频率。
- `command_timeout`
  - `jog` 超时后自动清零速度。
- `dt_clamp` / `jog_velocity_alpha` / `reference_drift_threshold`
  - 参考生成器稳定性参数。
- `require_backend_feedback`
  - 默认 `true`，要求收到后端 runtime 反馈后才会 `ready=true`。
  - 仿真可设为 `false`，此时只要 `/joint_states` 正常就允许摆臂控制。
- `fallback_min_position` / `fallback_max_position` / `fallback_max_velocity`
  - 缺少 `robot_description` 时的关节限位后备值。

完整参数见 [config/flipper_control.yaml](/home/rera/robot24_ws/src/flipper_control/config/flipper_control.yaml)。

## 使用边界

- 本包不直接实现电机执行层，只负责编排与路由。
- `csp_position` 接受轨迹命令；`csp_jog` 和 `csv_velocity` 接受点动命令。
- `CSP` 内部档位切换不做冷切换；`CSP <-> CSV` 会调用底层服务执行冷切换。
- `ready=false`、`switching=true` 或底层生命周期不满足时，运动命令会被拒绝。
- `cst_torque` 当前未实现。

## 延伸文档

- 补充导航见 [docs/README.md](/home/rera/robot24_ws/src/flipper_control/docs/README.md)
- 调试 UI 单独说明见 [docs/flipper_motor_debug_ui.md](/home/rera/robot24_ws/src/flipper_control/docs/flipper_motor_debug_ui.md)
- 相关实现与设计记录仍保留在项目级 `docs/` 中，便于追溯控制包拆分和模式设计
