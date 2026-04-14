# Ruckig 控制器 v2.1 迁移说明（2026-04-14）

## 1. 适用范围

本文档描述从旧版混合执行语义迁移到 Ruckig 控制器 v2.1 时，需要关注的行为变化、配置变化和联调步骤。

适用对象：

- 使用 `FollowJointTrajectory` 的上层调用方
- 使用 MoveIt Servo 的联调人员
- 维护 `hybrid_motor_hw.launch` 的集成人员

---

## 2. 主要行为变化

### 2.1 Action 从 waypoint 驱动改为时间驱动

迁移前常见现象：

- 逻辑更接近“一个 waypoint 到位了再推进下一个”
- 落后时容易出现对后续 waypoint 的离散追赶

v2.1 改为：

- 严格按 `time_from_start` 采样 nominal reference
- action 完成判定由 tolerance 和 `goal_time_tolerance` 决定
- 不再把“Ruckig 自己 finished”当成 action 完成的唯一依据

### 2.2 Action 连续目标默认只下发位置

迁移前如果你依赖了“每周期把 nominal velocity / acceleration 也前馈给 Ruckig”的思路，需要注意：

- v2.1 对 Action 默认不再这么做
- 这样是为了避免把轨迹分段不连续直接注入执行层

保留下来的内容：

- feedback 仍然保留 nominal 的位置、速度、加速度
- diagnostics 仍然能看到 nominal / command / actual 三种状态

### 2.3 Servo 超时语义更明确

v2.1 中 Servo 超时不再只是“停止更新”。

当前实现会：

- 读取实测状态
- 构造一个非连续的停车目标
- 以零速度、零加速度为目标让执行器平滑收敛

### 2.4 所有权切换更明确

v2.1 明确保证：

- Action 可以抢占 Servo
- Servo 不能反抢正在执行的 Action
- Action 成功或失败后会释放执行器所有权

这意味着：

- Action 结束后，Servo 可以恢复更新
- 不再需要靠手工清理内部 target 才能重新接管

---

## 3. 配置迁移

### 3.1 新增的配置命名空间

现在推荐把执行器相关参数统一放到：

```text
hybrid_motor_hw_node/ruckig_executor
```

典型加载方式是：

```xml
<arg name="ruckig_executor_config"
     default="$(find Eyou_ROS1_Master)/config/ruckig_executor.yaml"/>

<rosparam file="$(arg ruckig_executor_config)"
          command="load"
          ns="hybrid_motor_hw_node/ruckig_executor"
          subst_value="true"/>
```

### 3.2 需要关注的新字段

新增或强化的字段包括：

- `default_path_tolerance_position`
- `default_goal_tolerance_position`
- `default_stopped_velocity_tolerance`
- `default_goal_time_tolerance`
- `planning_deviation_warn_threshold`
- `tracking_fault_threshold`
- `continuous_mode.*`
- `constraint_validation.*`

### 3.3 配置默认值的含义变化

与旧实现相比，下面两个默认值需要重点理解：

- `default_goal_time_tolerance`
  - v2.1 纯位置跟踪下建议保留缓冲，不要默认为 0

- `planning_deviation_warn_threshold`
  - 现在是显式独立阈值
  - 不再隐式复用 path tolerance / goal tolerance 推导

---

## 4. 上层系统迁移注意事项

### 4.1 对 MoveIt / Action Client

需要接受一个事实：

- `desired` 仍是 nominal reference
- `actual` 仍是硬件状态
- `error` 仍是 `desired - actual`
- 但 `actual` 不一定严格贴着 nominal 时间轴跑

只要：

- 最终在 goal tolerance 内
- 并且没有超过 `goal_time_tolerance`

那么晚到成功是允许的。

### 4.2 对 Servo 链路

如果上游偶尔能提供速度信息：

- 可以透传 velocity
- 但不是硬要求

如果上游只提供位置：

- v2.1 仍然是有效方案
- Ruckig 会围绕位置目标自行生成平滑轨迹

### 4.3 对诊断与告警

迁移后建议不要再把下面两类偏差混为一谈：

- `tracking_error = actual - ruckig_command`
- `planning_deviation = ruckig_command - nominal_reference`

前者反映硬件跟踪质量，后者反映 Ruckig 与 nominal 的相对偏差。

---

## 5. 启用 MoveIt 约束校验前的准备

代码已经支持启动时校验：

- MoveIt `joint_limits.yaml`
- 与执行器 `ip_max_velocity / ip_max_acceleration` 的关系

但是在当前仓库配置下，示例参数仍默认：

```yaml
constraint_validation:
  enabled: false
```

迁移时建议按下面顺序处理：

1. 先统一 canopen / can_driver / MoveIt 三处关节约束
2. 确认 `Ruckig >= MoveIt * margin`
3. 再打开 `constraint_validation.enabled`

如果直接打开，很可能会在当前仓库配置下触发启动失败，这不是代码 bug，而是配置前提尚未统一。

---

## 6. 建议的迁移步骤

### 6.1 代码与启动层

1. 更新到包含 v2.1 的提交序列
2. 在 launch 中引入 `ruckig_executor_config`
3. 确认参数实际加载到 `hybrid_motor_hw_node/ruckig_executor`

### 6.2 约束与 tolerance

1. 核对各 joint 的 `ip_max_velocity / ip_max_acceleration / ip_max_jerk`
2. 确认默认 `goal_time_tolerance` 不要过紧
3. 确认 `planning_deviation_warn_threshold` 明显大于普通 path tolerance

### 6.3 联调顺序

1. 先只跑 Action，验证时间驱动和 tolerance 语义
2. 再跑 Servo，验证连续更新和平滑超时停车
3. 最后验证 Action / Servo 抢占与恢复

---

## 7. 回归检查项

迁移完成后，至少回归以下场景：

1. 多 waypoint action 在段边界无明显抽动
2. path tolerance 超限时 action 正常 abort
3. nominal 结束后允许在 `goal_time_tolerance` 内晚到成功
4. Action 启动后能够抢占 Servo
5. Servo 无法在 Action 执行中反抢
6. Action 成功或失败后 Servo 能再次接管
7. 诊断消息中的 `joint_names`、`tracking_error`、`planning_deviation` 符号正确

---

## 8. 常见误区

### 8.1 “planning_deviation 大就是坏事”

不一定。

纯位置跟踪下，Ruckig 相对 nominal 提前或滞后都可能发生。只要硬件稳定跟踪 `ruckig_command`，并且 action 最终在 tolerance 内完成，这通常是可接受的。

### 8.2 “把 velocity / acceleration 全部前馈给 Ruckig 才更高级”

在当前架构里并不总是这样。

如果前馈本身在轨迹段边界不连续，它反而更容易把抖动引入执行层。

### 8.3 “打开 MoveIt 约束校验一定更安全”

前提是三处约束已经统一。

如果仓库里的 MoveIt 约束、底层执行约束还没对齐，直接打开只会让系统在启动阶段提前失败。
