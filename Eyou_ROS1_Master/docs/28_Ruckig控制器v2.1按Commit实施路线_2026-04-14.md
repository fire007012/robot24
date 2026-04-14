# Ruckig 控制器 v2.1 按 Commit 实施路线（2026-04-14）

## 1. 目的

本文档给出与 [27_Ruckig控制器v2.1完整设计方案_2026-04-14.md](27_Ruckig控制器v2.1完整设计方案_2026-04-14.md) 对齐的实施路线。

设计重点已经从“新增字段和采样能力”转为：

- 语义对齐
- 状态机澄清
- 可观测性增强
- 测试补齐

因此 commit 路线按“先协议语义，再执行器状态机，再诊断与测试”的顺序重排。

---

## 2. 实施前检查

正式开发前先完成一次基线核对，不必单独成大 commit，但建议留下简短记录。

### 2.1 检查项

- `HybridTrajectoryTimeSampler` 已支持加速度
- `Target.state` 已支持位置/速度/加速度
- 当前 `feedback.error` 符号为 `desired - actual`
- 当前主循环顺序未变

### 2.2 目的

避免把已经存在的能力重复当作“新功能”，导致 commit 粒度失真。

---

## 3. 总体策略

### 3.1 切分原则

- 先把 tolerance 和协议语义落稳
- 再改执行器内部状态机
- 再把 action 驱动方式从 waypoint 改成时间驱动的纯位置跟踪
- 最后补 Servo、诊断、配置校验和测试

### 3.2 建议节奏

- Week 1：协议语义与执行器接口准备
- Week 2：连续模式状态机与 action 时间驱动
- Week 3：Servo、诊断、配置校验
- Week 4：集成测试、基准、文档

---

## 4. Commit 路线

## Commit 1: `feat(tolerance): add JTC-compatible tolerance resolver`

### 目标

新增一层明确的 tolerance 解析逻辑，对齐 Noetic `joint_trajectory_controller` 语义。

### 主要内容

- 新增 `ResolvedTolerances` 数据结构
- 实现默认配置 + goal 覆盖逻辑
- 支持：
  - `> 0` 覆盖
  - `= 0` 保持默认
  - `< 0` 清除限制
- 为 `goal_time_tolerance` 提供一致解析

### 主要文件

- `src/hybrid_follow_joint_trajectory_executor.cpp`
- `include/Eyou_ROS1_Master/hybrid_follow_joint_trajectory_executor.hpp`
- 可选新文件：
  - `include/Eyou_ROS1_Master/hybrid_tolerance_resolver.hpp`
  - `src/hybrid_tolerance_resolver.cpp`

### 必测项

- `JointTolerance` 的三种值语义
- 每 joint 单独覆盖
- 默认 stopped velocity tolerance

### 预计工作量

- 0.5 - 1 天

---

## Commit 2: `feat(executor): expose measured state, command state and execution status`

### 目标

给后续 action、servo、diagnostics 提供统一可查询的执行状态接口。

### 主要内容

- 为 `HybridJointTargetExecutor` 增加只读接口：
  - `getMeasuredState()`
  - `getCurrentCommand()`
  - `getExecutionStatus()`
  - `getJointNames()`
- 定义执行器状态枚举：
  - `kHold`
  - `kTracking`
  - `kResyncing`
  - `kFinished`
  - `kTrackingFault`
  - `kError`

### 主要文件

- `include/Eyou_ROS1_Master/hybrid_joint_target_executor.hpp`
- `src/hybrid_joint_target_executor.cpp`

### 必测项

- command state 与 measured state 可稳定读取
- 无 target 时状态为 `kHold`

### 预计工作量

- 0.5 天

---

## Commit 3: `feat(executor): implement continuous-mode state machine with hysteresis resync`

### 目标

把连续跟踪从“口头混合策略”落实为明确状态机。

### 主要内容

- 新增连续模式子状态：
  - `kInitFromHardware`
  - `kFollowInternalState`
  - `kResyncFromHardware`
- 进入连续模式时从硬件初始化
- 正常跟踪时使用 `output.pass_to_input(input_)`
- tracking error 连续超阈值后触发 resync
- 引入恢复阈值和连续周期计数

### 主要文件

- `include/Eyou_ROS1_Master/hybrid_joint_target_executor.hpp`
- `src/hybrid_joint_target_executor.cpp`

### 必测项

- 首次进入 continuous 的初始化行为
- resync 触发与恢复
- hysteresis 防抖

### 预计工作量

- 1 - 1.5 天

---

## Commit 4: `feat(executor): add internal tracking fault detection`

### 目标

把 `actual - ruckig_command` 故障检测下沉到执行器内部，不让 action 前端承担旧一拍 command 的时序问题。

### 主要内容

- 新增 `tracking_fault_threshold`
- 区分：
  - resync 条件
  - tracking fault 条件
- 保留 fault 细节：
  - joint name
  - error value
  - timestamp

### 主要文件

- `src/hybrid_joint_target_executor.cpp`
- `include/Eyou_ROS1_Master/hybrid_joint_target_executor.hpp`

### 必测项

- `resync_threshold < tracking_fault_threshold`
- 允许先 resync，不会立即误触发 fault

### 预计工作量

- 0.5 天

---

## Commit 5: `refactor(action): switch from waypoint-driven execution to time-driven pure-position tracking`

### 目标

把 action 主语义从 waypoint 推进改成时间驱动的 nominal 跟踪，并默认采用纯位置跟踪方案。

### 主要内容

- 移除 waypoint index 推进逻辑
- 每周期按 `elapsed_time` 采样 nominal reference
- 以 `CONTINUOUS` 模式把 nominal position 喂给执行器
- action continuous target 不前馈 nominal velocity / acceleration
- 保留 nominal velocity / acceleration 用于 feedback、tolerance 和 diagnostics
- 不再在 continuous 模式下设置 `minimum_duration_sec`

### 主要文件

- `src/hybrid_follow_joint_trajectory_executor.cpp`
- `include/Eyou_ROS1_Master/hybrid_follow_joint_trajectory_executor.hpp`

### 必测项

- action 进度不再依赖 `trajectoryFinished()`
- 多 waypoint 轨迹按时间推进
- action 下发的 continuous target 默认不携带 velocity / acceleration

### 预计工作量

- 1 - 1.5 天

---

## Commit 6: `feat(action): implement standard path tolerance and goal completion semantics`

### 目标

完成 action 协议层语义收口。

### 主要内容

- 标准 `path_tolerance` 检查使用：
  - `actual - nominal_reference`
- `planning_deviation = command - nominal_reference` 只做 warn，不作为 abort 条件
- 标准 `goal_tolerance` 检查使用解析后的最终 tolerances
- 支持 `goal_time_tolerance` 窗口
- 到期后返回：
  - `SUCCESSFUL`
  - `PATH_TOLERANCE_VIOLATED`
  - `GOAL_TOLERANCE_VIOLATED`

### 主要文件

- `src/hybrid_follow_joint_trajectory_executor.cpp`
- `tests/test_hybrid_follow_joint_trajectory_executor.cpp`

### 必测项

- path tolerance 违规
- planning deviation 超阈值仅告警
- late success within goal_time_tolerance
- timeout failure after deadline

### 预计工作量

- 1 天

---

## Commit 7: `fix(action): preserve feedback semantics and annotate sign convention`

### 目标

明确 feedback 语义，不在本轮翻转 error 符号。

### 主要内容

- 保持：
  - `desired = nominal`
  - `actual = hardware`
  - `error = desired - actual`
- `desired` 仍发布完整 nominal `position / velocity / acceleration`
- 在代码中补注释说明符号
- 补测试固定约定

### 主要文件

- `src/hybrid_follow_joint_trajectory_executor.cpp`
- `tests/test_hybrid_follow_joint_trajectory_executor.cpp`

### 必测项

- feedback 数值方向

### 预计工作量

- 0.5 天

---

## Commit 8: `fix(servo): use continuous updates and measured-state smooth stop on timeout`

### 目标

修正 Servo 正常更新和超时行为。

### 主要内容

- 正常 Servo 更新改为默认 `CONTINUOUS` 纯位置跟踪
- 若上游提供稳定 velocity，则允许选择性透传 velocity
- Servo 超时时读取实测状态
- 基于实测状态构造 `WAYPOINT` stop target：
  - `position = measured_position`
  - `velocity = 0`
  - `acceleration = 0`

### 主要文件

- `src/hybrid_servo_bridge.cpp`
- `include/Eyou_ROS1_Master/hybrid_servo_bridge.hpp`

### 必测项

- 高频 Servo 不抖动
- Servo 无 velocity 输入时仍能稳定连续跟踪
- 硬件落后于参考时，超时后能就地减速停车

### 预计工作量

- 0.5 - 1 天

---

## Commit 9: `feat(diagnostics): publish TrajectoryExecutionState with explicit joint order`

### 目标

补齐可观测性。

### 主要内容

- 新增 `TrajectoryExecutionState.msg`
- 明确 `joint_names` 顺序
- 发布：
  - nominal reference
  - ruckig command
  - actual state
  - tracking error
  - planning deviation
  - active source
  - executor status
  - continuous mode state source

### 主要文件

- `msg/TrajectoryExecutionState.msg`
- `src/hybrid_motor_hw_node.cpp`
- `CMakeLists.txt`
- `package.xml`

### 必测项

- topic 可发布
- 三组状态数组顺序一致
- `planning_deviation` 正负方向都可解释

### 预计工作量

- 0.5 - 1 天

---

## Commit 10: `feat(config): add moveit-vs-ruckig constraint validation and config cleanup`

### 目标

把配置职责和诊断边界写清楚。

### 主要内容

- 增加 MoveIt 与 Ruckig 约束对比校验
- 加入裕度参数
- 把配置拆成：
  - default tolerances
  - tracking fault threshold
  - planning deviation threshold
  - continuous mode thresholds
  - validation margins
- 为纯位置跟踪调整默认 `goal_time_tolerance` / `stopped_velocity_tolerance` / `planning_deviation` 推荐值

### 主要文件

- `src/hybrid_ip_executor_config.cpp`
- 配置示例文件
- 启动参数读取逻辑

### 必测项

- 配置缺失
- 约束过紧
- 阈值关系非法
- 纯位置跟踪下的 planning deviation 推荐值不过紧

### 预计工作量

- 1 天

---

## Commit 11: `test: add semantic unit tests for tolerance resolver, continuous state machine and servo timeout`

### 目标

优先把最容易回归的语义固定下来。

### 主要内容

- tolerance resolver tests
- continuous mode state machine tests
- tracking fault tests
- Servo timeout stop tests
- feedback sign tests

### 预计工作量

- 1 - 1.5 天

---

## Commit 12: `test: add integration tests for action timing, tolerance handling and source switching`

### 目标

覆盖跨模块时序。

### 主要内容

- action 时间驱动多 waypoint
- path tolerance abort
- goal timeout / late success
- action 抢占 servo
- servo 无法反抢 action
- diagnostics state coherence

### 预计工作量

- 1.5 - 2 天

---

## Commit 13: `docs: add v2.1 implementation notes, configuration guide and migration notes`

### 目标

补齐文档交付。

### 主要内容

- 更新 README
- 添加配置指南
- 添加从 v1 到 v2.1 的迁移说明
- 记录非目标和已知限制

### 预计工作量

- 1 天

---

## 5. Week 视图

### Week 1

- Commit 1
- Commit 2
- Commit 3
- Commit 4

里程碑：

- tolerance 语义明确
- executor 状态接口可用
- continuous resync 状态机跑通

### Week 2

- Commit 5
- Commit 6
- Commit 7

里程碑：

- action 已经变成时间驱动纯位置跟踪
- feedback 与标准 tolerance 语义稳定

### Week 3

- Commit 8
- Commit 9
- Commit 10

里程碑：

- Servo 行为修正完成
- diagnostics 可用
- 配置校验落地

### Week 4

- Commit 11
- Commit 12
- Commit 13

里程碑：

- 测试完成
- 文档完备
- 可进入真实机器人验证

---

## 6. 第一周建议落地顺序

如果这周就开始做，建议按下面顺序起步：

1. 先做 Commit 1  
   原因：这是 action 语义的基础，后面的完成判定都依赖它。

2. 再做 Commit 2  
   原因：不给 executor 暴露状态接口，Servo 停车和 diagnostics 都会绕。

3. 然后做 Commit 3  
   原因：continuous 模式状态机是这轮最核心的执行层改动。

4. 最后补 Commit 4  
   原因：tracking fault 阈值和状态机要一起调。

---

## 7. 风险提醒

### 7.1 不建议提前做的事

- 不建议一开始就做 diagnostics 消息
- 不建议一开始就做大量 launch/config 包装
- 不建议先做 benchmark 再修语义

### 7.2 最可能拖慢进度的点

- continuous mode resync 状态机细节
- tolerance 解析与现有默认值的兼容
- Servo timeout stop 的真实硬件行为

---

## 8. 完成标准

以下条件满足后，才算 v2.1 设计落地完成：

- action 时间驱动纯位置跟踪已替代 waypoint 驱动
- standard tolerance 语义可测试、可复现
- tracking fault 与 path tolerance 已语义分离
- Servo 正常更新与超时停车行为正确
- diagnostics 可以解释 nominal / command / actual 差异以及 planning deviation 的正负方向
- README 与实施文档已同步

---

## 9. 总结

这份 commit 路线的核心变化是：

- 不再从“字段扩展”起步
- 先解决语义对齐问题
- 再解决执行器连续跟踪状态机
- 再把 action 收口到时间驱动的纯位置跟踪
- 最后补齐诊断、配置和测试

这样可以把最容易引起行为偏差和讨论反复的部分尽早收敛。
