# Ruckig 控制器 v2.1 完整设计方案（2026-04-14）

## 文档元信息

- 基于文档：
  - [25_Ruckig控制器架构与职责说明_2026-04-14.md](25_Ruckig控制器架构与职责说明_2026-04-14.md)
  - [26_Ruckig控制器设计评审Checklist_2026-04-14.md](26_Ruckig控制器设计评审Checklist_2026-04-14.md)
- 目标版本：`v2.1`
- 文档目的：给出一份可直接指导实现的设计稿，并收敛上一轮 review 提出的关键语义问题

---

## 0. 实施前提条件检查

在开始重构前，先确认当前代码基线，避免按错误假设切 commit。

### 0.1 需要确认的当前状态

- `HybridTrajectoryTimeSampler` 是否已经输出加速度
- `HybridJointTargetExecutor::Target` 是否已经通过 `Target.state` 支持位置、速度、加速度
- 当前 `feedback.error` 的符号约定是什么
- 当前 `trajectoryFinished()` 是否还被 action 完成判定依赖

### 0.2 推荐检查命令

```bash
rg -n "accelerations|HermiteAcceleration" Eyou_ROS1_Master/src/hybrid_trajectory_time_sampler.cpp
rg -n "struct Target|State state|continuous_reference" Eyou_ROS1_Master/include/Eyou_ROS1_Master/hybrid_joint_target_executor.hpp
rg -n "feedback\\.error\\." Eyou_ROS1_Master/src/hybrid_follow_joint_trajectory_executor.cpp
rg -n "trajectoryFinished" Eyou_ROS1_Master/src Eyou_ROS1_Master/tests
```

### 0.3 当前已确认的基线

基于 2026-04-14 工作区代码：

- `HybridTrajectoryTimeSampler` 已经计算位置、速度、加速度
- `HybridJointTargetExecutor::Target` 目前通过 `Target.state` 携带完整状态
- 当前 `feedback.error` 符号为 `desired - actual`
- 当前 action 仍然依赖 `trajectoryFinished()` 推进 waypoint

因此：

- 不再把“加速度支持”当作全新功能
- 不再把“Target 支持速度/加速度”当作全新功能
- 重构重点转为语义修正和状态机澄清

---

## 1. 设计目标

### 1.1 核心目标

将当前的 Ruckig 控制器从“waypoint 完成驱动的执行器”改造为“符合 `FollowJointTrajectory` 协议语义的时间跟踪控制器”，同时保留统一执行内核和多源输入能力。

### 1.2 P0 目标

- Action 执行按 `time_from_start` 驱动，而不是按 waypoint 完成推进
- `goal_tolerance` / `path_tolerance` / `goal_time_tolerance` 语义与 ROS Noetic 常见 `joint_trajectory_controller` 行为对齐
- `HybridJointTargetExecutor` 继续作为唯一机械臂命令写入者
- Ruckig 继续作为下游平滑与约束保护层，而不是 action 语义真相源

### 1.3 P1 目标

- 提供可观测的执行诊断，区分 nominal、command、actual 三种状态
- Servo 正常更新平滑，超时后平滑减速
- 连续跟踪模式具备明确的状态同步策略和纠偏机制

### 1.4 非目标

- 本轮不把该实现重构成标准 `ros_control` controller plugin
- 本轮不重新设计 `HybridRobotHW` / `HybridServiceGateway`
- 本轮不把 Servo 和 Action 拆成两套不同执行内核

---

## 2. 兼容性声明

### 2.1 协议层兼容性

本设计追求：

- 与 `control_msgs/FollowJointTrajectory` Action 协议兼容
- 与 MoveIt 作为 Action 客户端的预期语义兼容

兼容内容包括：

- 时间驱动执行
- 标准 feedback/result 语义
- `path_tolerance`
- `goal_tolerance`
- `goal_time_tolerance`

### 2.2 架构层兼容性

本设计暂不追求：

- 标准 `controller_manager` 插件形态
- 与 `rqt_controller_manager` 等工具链的完整架构兼容

当前架构仍然是：

- 节点内手工调度
- `controller_manager.update()` 之后的后置执行层

因此应区分：

- 协议兼容：是本轮目标
- 架构兼容：不是本轮目标

---

## 3. 保留的结构与不变量

### 3.1 保留的模块结构

本轮保留以下组件划分：

- `HybridFollowJointTrajectoryExecutor`
- `HybridServoBridge`
- `HybridJointTargetExecutor`
- `HybridTrajectoryTimeSampler`

### 3.2 保留的主循环顺序

```text
hybrid_hw.read(now, period)
controller_manager.update(now, period)
ip_executor.update(now, period)
servo_bridge.update(now)
joint_target_executor.update(period)
hybrid_hw.write(now, period)
```

### 3.3 必须保持的不变量

- `HybridJointTargetExecutor` 是唯一最终位置命令写入者
- Action 可以抢占 Servo
- Servo 不能反抢 Action
- 标准 action feedback 的 `desired` 表示 nominal trajectory reference
- 标准 action feedback 的 `actual` 表示 hardware state
- 标准 action feedback 的 `error` 保持当前实现的符号约定：`desired - actual`

---

## 4. 关键设计决策

### 4.1 Ruckig 的角色

Ruckig 在 v2.1 中的角色是：

- 时间跟踪器
- 平滑器
- 约束保护层

Ruckig 不是：

- Action 完成判定的唯一真相源
- 标准 `FollowJointTrajectory` 语义本身

### 4.2 Action 的驱动方式

Action 改为：

- 按 `elapsed_time` 对原始轨迹做连续采样
- 保留采样得到的 nominal `position / velocity / acceleration` 作为协议层 reference
- 向 Ruckig 下发的 action target 默认采用“纯位置跟踪”：
  - 连续更新 `target_position`
  - 不做每周期 `target_velocity / target_acceleration` 前馈
- 最终按标准 tolerance 窗口判定成功或失败

### 4.3 Servo 的驱动方式

Servo 改为：

- 正常流式更新：默认使用 `CONTINUOUS` 纯位置跟踪
- 若上游稳定提供速度信息，可选择性透传 velocity 作为 hint
- 输入超时：切换为基于实测状态的 `WAYPOINT` 减速停车目标

### 4.4 关于“Ruckig 约束 >= MoveIt 约束”

这是必要条件和配置诊断工具，不是充分保证。

它能带来：

- 避免明显约束冲突
- 提高正常情况下跟踪成功率
- 启动时发现配置错误

它不能保证：

- 一定能跟上
- 一定满足真实硬件动态性能
- 一定没有时间延迟

实际表现仍取决于：

- 硬件动力学
- 底层控制环
- 通信延迟
- jerk 假设差异
- 原始轨迹本身的动力学可行性

### 4.5 为什么 v2.1 默认采用纯位置跟踪

在当前架构下，action 每周期都能从原始轨迹采样到 nominal 的位置、速度、加速度。

但这并不意味着这些量都应该直接作为 Ruckig 的连续目标前馈输入。

v2.1 默认采用纯位置跟踪的原因是：

- waypoint 边界处的 nominal velocity / acceleration 可能出现不连续
- 某些轨迹在段末会天然回到零速度/零加速度
- 将这些离散段语义直接前馈给连续 OTG，容易把不连续传到执行层
- 若 Ruckig 约束不小于 MoveIt 约束，则让 Ruckig 只围绕位置目标自主规划，通常更平滑、更稳健

因此：

- Action 仍然采样完整 nominal state
- feedback 和 diagnostics 仍然保留完整 nominal state
- 但 action 默认只把 nominal position 下发给 Ruckig

---

## 5. 三层误差模型

v2.1 明确拆分三类误差，不再混用一个 tolerance 概念。

### 5.1 标准路径误差

定义：

- `path_error = actual - nominal_reference`

用途：

- 对应 `FollowJointTrajectory` 协议层的 `path_tolerance`
- 语义上属于“测量状态相对参考轨迹的偏差”

处理：

- 超限则 action 以 `PATH_TOLERANCE_VIOLATED` 中止
- 在纯位置跟踪方案下，这一误差衡量的是“硬件相对 nominal 的协议偏差”，不是“硬件相对 Ruckig command 的执行偏差”
- 因此默认 `path_tolerance` 建议保持禁用或宽松，由内部 tracking fault 承担执行层安全检查

### 5.2 执行器跟踪误差

定义：

- `tracking_error = actual - ruckig_command`

用途：

- 描述硬件是否跟得上执行器命令
- 属于控制器内部安全与健康检查

处理：

- 不复用 `path_tolerance` 这个名字
- 使用独立阈值 `tracking_fault_threshold`
- 超限时视为执行层 tracking fault，可中止 action，但错误描述必须明确这是内部执行故障语义

### 5.3 规划偏离

定义：

- `planning_deviation = ruckig_command - nominal_reference`

用途：

- 描述 Ruckig 为了平滑和约束保护而对 nominal 参考做了多大偏离

处理：

- 使用独立阈值 `planning_deviation_warn_threshold`
- 在纯位置跟踪方案下，`planning_deviation` 可能为正也可能为负：
  - 正值：Ruckig 相对 nominal 落后
  - 负值：Ruckig 相对 nominal 超前
- 默认只报警和发布诊断，不直接中止 action

---

## 6. Tolerance 解析规则

### 6.1 设计原则

v2.1 不再“猜测” goal 的 tolerance 语义，而是显式对齐 ROS Noetic 下 `joint_trajectory_controller` 的解析规则。

参考：

- `/opt/ros/noetic/share/control_msgs/msg/JointTolerance.msg`
- `/opt/ros/noetic/include/joint_trajectory_controller/tolerances.h`

### 6.2 解析规则

对每个 joint 的 position / velocity / acceleration tolerance：

- `> 0`：覆盖默认值
- `= 0`：保持默认值不变
- `< 0`：清除该项限制

对 `goal_time_tolerance`：

- `> 0`：覆盖默认值
- `= 0`：保持默认值
- `< 0`：按 0 处理

### 6.3 默认值来源

控制器提供自己的默认 tolerance 配置：

- `default_path_tolerance_position`
- `default_goal_tolerance_position`
- `default_stopped_velocity_tolerance`
- `default_goal_time_tolerance`

Action goal 只做覆盖，不重定义默认策略。

### 6.4 是否支持“宽松模式”

v2.1 默认不引入自动“late-success position-only”模式。

如确有遥操作需求，可后续增加一个显式开关：

- `nonstandard_allow_position_only_late_success`

但默认值必须为 `false`，避免破坏标准语义。

---

## 7. Action 执行语义

### 7.1 Action update 主流程

每个控制周期，`HybridFollowJointTrajectoryExecutor::update()` 执行：

1. 若无 active goal，直接返回
2. 推进 `active_goal_elapsed_sec_`
3. 用 `HybridTrajectoryTimeSampler` 对当前时间采样，得到 nominal reference
4. 构造“纯位置跟踪”的 `CONTINUOUS` 模式 target，喂给 `HybridJointTargetExecutor`
5. 从 JointState 读取 hardware actual state
6. 发布标准 action feedback
7. 检查标准 `path_tolerance`
8. 查询执行器状态，若进入 tracking fault / error，则中止 action
9. 在轨迹末尾及 `goal_time_tolerance` 窗口内检查 goal tolerance

### 7.2 Continuous 模式下的 target 构造

Action 构造的 target 使用：

- `tracking_mode = CONTINUOUS`
- `state.positions = nominal positions`
- `state.velocities` 默认留空
- `state.accelerations` 默认留空

并且：

- `minimum_duration_sec` 在 `CONTINUOUS` 模式下不设置

原因：

- 连续模式强调“更新跟踪参考”
- 不应该把每周期 `dt` 错当作新的最小时长约束
- 不将 waypoint 处可能不连续的 nominal velocity / acceleration 直接前馈到 Ruckig
- 让 Ruckig 在自身约束下围绕位置目标自主规划更平滑的实际命令

同时保留：

- nominal velocity / acceleration 继续用于 feedback
- nominal velocity / acceleration 继续用于 tolerance 与 diagnostics 解释

### 7.3 Path tolerance 检查

Action 前端只负责标准的 `actual - nominal_reference` 检查。

它不负责：

- `actual - ruckig_command` 的内部 tracking fault 判断
- `ruckig_command - nominal_reference` 的 planning deviation 故障判定

因为在当前主循环顺序下：

- `actual - ruckig_command` 更适合在执行器内部判断，避免使用旧一拍的 command 状态
- `ruckig_command - nominal_reference` 在纯位置跟踪方案下本来就允许存在，只应作为诊断/告警

### 7.4 Goal 完成判定

完成判定对齐 JTC 语义：

1. 在轨迹末尾之前，不做最终成功判定
2. 到达轨迹末尾后，开始检查最终 goal tolerance
3. 在 `end_time + goal_time_tolerance` 之前，只要任一周期满足最终 tolerance，就成功
4. 超过该窗口仍不满足，则 `GOAL_TOLERANCE_VIOLATED`

### 7.5 Feedback 语义

标准 feedback 定义为：

- `desired = nominal_reference`
- `actual = hardware_state`
- `error = desired - actual`

这里刻意保持当前实现已有的 error 符号，避免破坏下游已有认知和测试。

需要特别说明：

- `desired` 仍然发布完整的 nominal `position / velocity / acceleration`
- 即使 action 下发给执行器的 target 默认只包含 position，也不改变 feedback 的协议语义

---

## 8. Servo 执行语义

### 8.1 正常更新

Servo 正常收到新消息时：

- 只保留最新消息
- 只使用最后一个 point
- 默认转成“纯位置跟踪”的 `CONTINUOUS` 模式 target
- 若上游 message 自带稳定 velocity，可选择性填入 `target_velocity`
- `target_acceleration` 默认不提供

原因：

- Servo 是流式实时输入
- 高频 waypoint 重建会增加抖动和状态重置风险

### 8.2 超时行为

Servo 超时后，不直接 `clearTarget()`，而是执行平滑减速停车。

具体做法：

1. 从执行器读取当前实测状态
2. 构造一个 `WAYPOINT` 模式 stop target：
   - `target_position = measured_position`
   - `target_velocity = 0`
   - `target_acceleration = 0`
3. 交给执行器重新规划一条减速到零的停止轨迹

### 8.3 关键要求

stop target 必须基于实测状态，而不是最后一次 Servo 参考值。

否则当硬件落后于参考时，会出现“超时后仍继续追旧目标”的反直觉行为。

---

## 9. HybridJointTargetExecutor v2.1 设计

### 9.1 目标

执行器需要同时支持：

- 离散 waypoint 轨迹
- 连续参考跟踪
- 内部 tracking fault 检查
- 连续模式下的自适应纠偏

### 9.2 Target 语义

保留 `Target.state` 结构，不新增平铺数组字段。

扩展 `Target` 为：

```text
Target
- state.positions
- state.velocities
- state.accelerations
- minimum_duration_sec
- continuous_reference
- tracking_mode = WAYPOINT | CONTINUOUS
```

其中：

- `WAYPOINT`：从当前实测状态重建一条新轨迹
- `CONTINUOUS`：保持一条连续跟踪轨迹，只更新目标参考

其中 `state.velocities / state.accelerations` 的语义改为“可选输入”：

- Action 默认纯位置跟踪，连续模式下这两个字段留空
- Servo 可按上游能力选择是否填写 velocity
- measured-state stop target 应显式写 `velocity = 0`、`acceleration = 0`

### 9.3 执行状态

新增执行器运行状态：

- `kHold`
- `kTracking`
- `kResyncing`
- `kFinished`
- `kTrackingFault`
- `kError`

### 9.4 连续模式状态源状态机

连续模式不再只写“混合策略”，而是显式定义状态机：

- `kInitFromHardware`
- `kFollowInternalState`
- `kResyncFromHardware`

#### `kInitFromHardware`

进入条件：

- 首次进入 `CONTINUOUS`
- source 切换到 action / servo 的连续模式
- 从 `WAYPOINT` 切回 `CONTINUOUS`
- 连续模式目标首次生效

行为：

- 读取硬件实测位置/速度
- 以硬件状态初始化 `input_.current_*`
- 以当前 target 初始化 `input_.target_*`
- `otg_.reset()`
- 完成后切入 `kFollowInternalState`

#### `kFollowInternalState`

行为：

- 每周期只更新 `target_*`
- 用 `output_.pass_to_input(input_)` 维持内部连续状态

切出条件：

- 若 `|actual - command|` 连续 `N` 个周期大于 `resync_threshold`，切到 `kResyncFromHardware`

#### `kResyncFromHardware`

行为：

- 使用实测硬件状态重新初始化当前轨迹
- 当前 target 不变
- `otg_.reset()`

恢复条件：

- 若 `|actual - command|` 连续 `M` 个周期小于 `resync_recovery_threshold`，切回 `kFollowInternalState`

### 9.5 必要约束

为避免“还没 resync 就先 fault”：

- `resync_threshold < tracking_fault_threshold`

并建议：

- `N >= 2`
- `M >= 2`

避免单拍噪声导致来回切换。

### 9.6 Tracking fault 处理

`actual - ruckig_command` 超过 `tracking_fault_threshold` 时：

- 执行器状态置为 `kTrackingFault`
- 保留诊断信息：超限 joint、误差值、时间戳
- Action 前端在下一个周期读取该状态后中止 goal

这样可以避免让 action 前端直接依赖“本周期刚生成的 command”。

---

## 10. 可观测性与诊断

### 10.1 诊断消息

新增包内消息：

- `Eyou_ROS1_Master/TrajectoryExecutionState.msg`

### 10.2 消息字段原则

必须显式包含：

- `joint_names`
- `active_source`
- `executor_status`
- `continuous_mode_state_source`
- `nominal_reference`
- `ruckig_command`
- `actual_state`
- `tracking_error`
- `planning_deviation`
- `elapsed_time`
- `trajectory_duration`

补充约束：

- `nominal_reference` 仍保存完整的 sampled `position / velocity / acceleration`
- `planning_deviation` 必须按有符号量发布，允许正负两侧偏离

### 10.3 joint 顺序约定

所有数组都以 `joint_names` 为唯一顺序基准。

若 `actual_state.name` 被填写，则必须与 `joint_names` 完全一致；否则消费者应按 `joint_names` 解释 `actual_state.position/velocity/effort` 的顺序。

### 10.4 发布时机

诊断状态建议在 `joint_target_executor.update()` 之后发布。

这样：

- `ruckig_command` 是本周期刚生成的命令
- 不会和 action update 中拿到的旧一拍 command 混淆

---

## 11. 配置模型

### 11.1 协议层 tolerance 配置

```yaml
ruckig_executor:
  default_path_tolerance_position: 0.0
  default_goal_tolerance_position: 0.0
  default_stopped_velocity_tolerance: 0.05
  default_goal_time_tolerance: 2.0
```

说明：

- 这些是 controller 默认值
- Action goal 通过 `JointTolerance` 和 `goal_time_tolerance` 对其做覆盖
- 上述数值是纯位置跟踪方案下的推荐部署值，不影响解析语义本身

### 11.2 执行器内部阈值

```yaml
ruckig_executor:
  tracking_fault_threshold: 0.08
  planning_deviation_warn_threshold: 0.20
```

说明：

- `planning_deviation_warn_threshold` 只用于告警和诊断
- 在纯位置跟踪方案下，应接受 `planning_deviation` 比速度前馈方案更大

### 11.3 连续模式配置

```yaml
ruckig_executor:
  continuous_mode:
    resync_threshold: 0.04
    resync_recovery_threshold: 0.02
    resync_enter_cycles: 2
    resync_recovery_cycles: 2
```

要求：

- `resync_threshold < tracking_fault_threshold`

### 11.4 MoveIt 对比校验

```yaml
ruckig_executor:
  constraint_validation:
    enabled: true
    velocity_margin: 1.05
    acceleration_margin: 1.10
```

此校验只做：

- 配置诊断
- 早期失败提示

不对真实跟踪性能作保证。

### 11.5 Servo 配置

```yaml
ruckig_executor:
  servo:
    timeout: 0.2
```

---

## 12. 代码改动范围

### 12.1 重点修改文件

- `include/Eyou_ROS1_Master/hybrid_follow_joint_trajectory_executor.hpp`
- `src/hybrid_follow_joint_trajectory_executor.cpp`
- `include/Eyou_ROS1_Master/hybrid_joint_target_executor.hpp`
- `src/hybrid_joint_target_executor.cpp`
- `include/Eyou_ROS1_Master/hybrid_servo_bridge.hpp`
- `src/hybrid_servo_bridge.cpp`
- `src/hybrid_ip_executor_config.cpp`
- `src/hybrid_motor_hw_node.cpp`

### 12.2 新增文件

- `msg/TrajectoryExecutionState.msg`
- `docs/28_Ruckig控制器v2.1按Commit实施路线_2026-04-14.md`

### 12.3 不再建议新增的内容

本轮不再把下面内容作为单独功能点立项：

- “给 sampler 增加 acceleration”
- “给 Target 增加 velocities/accelerations”

因为当前代码基线已具备这些能力。

---

## 13. 测试策略

### 13.1 单元测试必须覆盖

- tolerance 解析语义：
  - `> 0`
  - `= 0`
  - `< 0`
- Action 按时间推进，而不是按 waypoint 完成推进
- Action 连续 target 默认只下发 position，不前馈 nominal velocity / acceleration
- `goal_time_tolerance` 窗口内成功与超时失败
- `feedback.error` 符号保持 `desired - actual`
- `CONTINUOUS` 模式的初始化、resync、恢复
- Servo 正常连续更新
- Servo 可选 velocity 透传不会破坏默认纯位置跟踪路径
- Servo 超时基于实测状态减速停车
- `planning_deviation` 正负方向都能被正确观测和限幅告警
- tracking fault 与 path tolerance 不混淆

### 13.2 集成测试必须覆盖

- 多 waypoint MoveIt 轨迹执行
- path tolerance 违规
- tracking fault 违规
- nominal / command 偏离告警但不直接 abort
- Action 抢占 Servo
- Servo 无法反抢 Action
- 主循环顺序不变时，诊断 topic 中 command/actual 对齐

### 13.3 真实硬件验证重点

- 长轨迹时间跟踪误差
- 末端到位精度
- Servo 高频流式更新手感
- 通信抖动下的 resync 行为

---

## 14. 风险与缓解

### 14.1 连续模式状态机过于敏感

风险：

- 频繁在 `kFollowInternalState` 与 `kResyncFromHardware` 之间切换

缓解：

- 使用 enter/recovery 连续周期计数
- 设置滞回阈值

### 14.2 tracking fault 过早触发

风险：

- 一次短暂滞后就导致 action 中止

缓解：

- 将 tracking fault 与标准 path tolerance 分离
- 允许执行器先走 resync 再判定 fault

### 14.3 诊断数据误导

风险：

- command 和 actual 的时间戳错拍

缓解：

- 在 executor update 之后统一发布诊断状态
- 明确 `joint_names` 顺序和数据语义

### 14.4 纯位置跟踪下 nominal 偏离增大

风险：

- `ruckig_command` 相对 `nominal_reference` 的偏离明显增大
- 若沿用过紧的 path / deviation 阈值，容易把正常行为误判为异常

缓解：

- 将 `planning_deviation` 保持为 warn-only 语义
- 为真实机器人配置足够的 `goal_time_tolerance`
- 仅在确有协议需要时启用严格 `path_tolerance`

---

## 15. 成功标准

### 15.1 语义层

- Action 不再依赖 `trajectoryFinished()` 作为推进依据
- Action 默认采用时间驱动的纯位置连续跟踪
- tolerance 解析与 JTC 行为一致
- Servo 超时停车基于实测状态
- 三类误差语义分离清楚

### 15.2 工程层

- 所有关键状态可诊断
- 配置项含义不再重叠
- 主循环顺序依赖被文档化并纳入测试

### 15.3 测试层

- 单元测试覆盖关键语义
- 集成测试覆盖 Action / Servo / resync / tolerance
- 真实硬件验证通过

---

## 16. 总结

v2.1 的核心不是“再给现有实现补几个 if”，而是把 6 个关键问题彻底定型：

1. 直接对齐 JTC tolerance 语义，不再靠自适应猜测
2. 将 `actual-nominal`、`actual-command`、`command-nominal` 三类误差分开
3. Action 默认只向 Ruckig 下发 position，避免把 nominal velocity / acceleration 的段间不连续直接前馈到执行层
4. 把连续模式写成明确状态机，而不是口头上的混合策略
5. Servo 正常用连续跟踪，超时用基于实测状态的停车轨迹
6. 将 command 相关故障判定下沉到执行器，避免与当前主循环顺序冲突

在这个基础上，后续无论继续优化性能，还是再演进为标准 controller plugin，都会清楚很多。
