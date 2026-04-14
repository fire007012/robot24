# Ruckig 控制器 v2.1 实现说明与配置指南（2026-04-14）

## 1. 文档目的

本文档面向已经接手 `Eyou_ROS1_Master` 代码的人，说明本次 v2.1 实现最终落地成了什么、运行时如何协同，以及应当如何配置和调参。

配套文档：

- [27_Ruckig控制器v2.1完整设计方案_2026-04-14.md](27_Ruckig控制器v2.1完整设计方案_2026-04-14.md)
- [28_Ruckig控制器v2.1按Commit实施路线_2026-04-14.md](28_Ruckig控制器v2.1按Commit实施路线_2026-04-14.md)

---

## 2. 最终实现概览

### 2.1 模块职责

- `HybridFollowJointTrajectoryExecutor`
  - 负责 `FollowJointTrajectory` 协议语义
  - 按 `time_from_start` 采样 nominal reference
  - 处理 `path_tolerance` / `goal_tolerance` / `goal_time_tolerance`
  - 产出 action feedback 与内部诊断参考

- `HybridServoBridge`
  - 接收 Servo 流式目标
  - 正常情况下按连续位置目标更新执行器
  - 超时后用实测状态构造平滑停车目标

- `HybridJointTargetExecutor`
  - 是唯一命令下发者
  - 负责 Ruckig 在线轨迹生成
  - 维护 continuous mode 内部状态机、resync 和 tracking fault

- `HybridExecutionDiagnostics`
  - 把 nominal / command / actual 三种状态整理成诊断消息

### 2.2 关键语义

- Action 改为时间驱动，不再按 waypoint 完成推进
- Action 下发给 Ruckig 的连续目标默认只包含位置
- nominal velocity / acceleration 继续保留在 feedback 与 diagnostics 中
- Action 可以抢占 Servo
- Servo 不能反抢正在执行的 Action
- Action 结束或失败后会主动释放执行器所有权，Servo 可重新接管

---

## 3. 纯位置跟踪方案

### 3.1 为什么只下发位置

v2.1 的默认策略是：

- Action 每周期仍完整采样 nominal `position / velocity / acceleration`
- 但写给 `HybridJointTargetExecutor` 的连续目标只填 `position`
- `velocity / acceleration` 留空，由 Ruckig 在约束内自行规划

这样做的主要原因是避免把 waypoint 边界上的速度或加速度不连续直接传到执行层，尤其是段末回零这类典型不连续点。

### 3.2 反馈和诊断怎么理解

- `feedback.desired`
  - 仍然表示 MoveIt / 原始轨迹的 nominal reference
- `feedback.actual`
  - 表示硬件实测状态
- `feedback.error`
  - 保持当前实现约定：`desired - actual`
- `TrajectoryExecutionState.nominal_reference`
  - 表示 nominal reference
- `TrajectoryExecutionState.ruckig_command`
  - 表示 Ruckig 实际生成的命令状态
- `TrajectoryExecutionState.actual_state`
  - 表示硬件实测状态
- `tracking_error`
  - 表示 `actual - ruckig_command`
- `planning_deviation`
  - 表示 `ruckig_command - nominal_reference`

需要注意：

- `planning_deviation` 正负都可能出现
- 在纯位置跟踪下，适度偏离 nominal 是正常现象
- 真正应该严格盯的是硬件对 `ruckig_command` 的跟踪误差

---

## 4. 当前运行链路

当前主循环仍然是：

```text
hybrid_hw.read(now, period)
controller_manager.update(now, period)
ip_executor.update(now, period)
servo_bridge.update(now)
joint_target_executor.update(period)
hybrid_hw.write(now, period)
```

约束与语义分层如下：

- MoveIt / FollowJointTrajectory
  - 负责 nominal 轨迹与协议层 tolerance
- HybridFollowJointTrajectoryExecutor
  - 负责时间采样与 action 语义
- HybridJointTargetExecutor / Ruckig
  - 负责平滑、限速、限加速度、限 jerk 与最终命令
- 硬件与底层控制环
  - 负责实际跟踪命令

---

## 5. 配置入口

### 5.1 加载方式

新增配置文件：

- `config/ruckig_executor.yaml`

新增 launch 参数：

- `ruckig_executor_config`

launch 中会把该文件加载到：

```text
hybrid_motor_hw_node/ruckig_executor
```

也就是说，`BuildHybridIpExecutorConfigFromParams(...)` 读取的是 `hybrid_motor_hw_node` 节点下 `ruckig_executor` 这一层参数。

### 5.2 当前示例配置

当前样例文件包含以下字段：

```yaml
default_path_tolerance_position: 0.0
default_stopped_velocity_tolerance: 0.05
default_goal_time_tolerance: 2.0

planning_deviation_warn_threshold: 0.20
tracking_fault_threshold: 0.08

continuous_mode:
  resync_threshold: 0.04
  resync_recovery_threshold: 0.02
  resync_enter_cycles: 2
  resync_recovery_cycles: 2

constraint_validation:
  enabled: false
  moveit_joint_limits_path: "$(find car_moveit_config)/config/joint_limits.yaml"
  velocity_margin: 1.05
  acceleration_margin: 1.10
```

另外，代码还支持一个可选字段：

- `default_goal_tolerance_position`

如果不显式配置，默认 goal position tolerance 会沿用各 joint 的底层 `ip_goal_tolerance`。

---

## 6. 各参数建议

### 6.1 tolerance 相关

- `default_path_tolerance_position`
  - 默认路径容差
  - 设为 `0.0` 表示默认不启用位置路径容差

- `default_goal_tolerance_position`
  - 默认 goal 位置容差
  - 未设置时继承 joint 级 `ip_goal_tolerance`

- `default_stopped_velocity_tolerance`
  - goal 检查时默认的停止速度容差
  - 对纯位置跟踪应保持适度宽松

- `default_goal_time_tolerance`
  - nominal 结束后允许继续追赶的时间窗口
  - 纯位置跟踪下建议不要过紧

### 6.2 诊断与故障阈值

- `planning_deviation_warn_threshold`
  - 只用于告警和诊断
  - 不直接触发 action abort
  - 纯位置跟踪下应明显大于普通 path tolerance

- `tracking_fault_threshold`
  - 表示硬件偏离 `ruckig_command` 的故障阈值
  - 这是执行安全层面的更强约束

### 6.3 continuous mode

- `continuous_mode.resync_threshold`
  - 超过该阈值并持续若干周期后进入 resync

- `continuous_mode.resync_recovery_threshold`
  - 低于该阈值并持续若干周期后退出 resync

- `continuous_mode.resync_enter_cycles`
  - 进入 resync 所需连续周期数

- `continuous_mode.resync_recovery_cycles`
  - 恢复到内部跟踪所需连续周期数

应满足：

- `resync_recovery_threshold <= resync_threshold`
- `resync_threshold < tracking_fault_threshold`

### 6.4 MoveIt 约束校验

- `constraint_validation.enabled`
  - 是否在启动时校验 MoveIt 约束与 Ruckig 约束关系

- `constraint_validation.moveit_joint_limits_path`
  - MoveIt `joint_limits.yaml` 路径

- `constraint_validation.velocity_margin`
- `constraint_validation.acceleration_margin`
  - 用于要求 `Ruckig >= MoveIt * margin`

当前仓库默认把 `enabled` 设为 `false`，原因是现有机器人配置还没有全面满足“Ruckig 约束不小于 MoveIt 约束”的推荐前提。

如果后续统一了 `ip_max_velocity / ip_max_acceleration` 与 MoveIt 限制，再建议打开该检查。

---

## 7. 调参顺序建议

建议按下面顺序调：

1. 先保证底层硬件闭环稳定，能可靠跟踪缓慢位置命令
2. 再核对 MoveIt 与 Ruckig 的约束是否一致
3. 再调 `tracking_fault_threshold`
4. 再调 `continuous_mode.*`
5. 最后再看 `planning_deviation_warn_threshold`

不建议一开始就盯着 `planning_deviation` 调，因为它更多反映的是“Ruckig 与 nominal 的相对时序差”，不是硬件本身的跟踪质量。

---

## 8. 已知限制与非目标

### 8.1 已知限制

- 当前仍不是标准 `ros_control` controller plugin
- 当前 diagnostics 主要用于状态理解，不替代更完整的系统级监控
- `constraint_validation.enabled` 在仓库默认配置下仍是保守关闭
- 纯位置跟踪会允许 command 相对 nominal 出现超前或滞后，这属于方案预期

### 8.2 非目标

- 本轮不重写 `HybridRobotHW`
- 本轮不拆成两套 Action/Servo 执行内核
- 本轮不在 Servo 链路中强制要求速度前馈

---

## 9. 落地检查清单

部署或回归时建议最少确认以下几点：

1. Action 多 waypoint 轨迹按时间连续推进，不再在 waypoint 处抽动
2. Servo 连续刷新时没有重复起步感
3. Servo 超时后机械臂平滑减速而不是突然停死
4. Action 结束后 Servo 能重新接管
5. `TrajectoryExecutionState` 中的 `joint_names` 顺序和执行器一致
6. `tracking_error` 与 `planning_deviation` 的符号符合预期
