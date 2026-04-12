# MoveIt Servo 接入与统一 Ruckig 执行设计（2026-04-12）

## 1. 目的

本文档用于定义 `Eyou_ROS1_Master` 接入 `MoveIt Servo` 时的正式运动执行方案。

目标不是“先接通再说”，而是从一开始就避免形成两套并行的运动执行链：

- 一套给 `FollowJointTrajectory`
- 一套给 `MoveIt Servo`

本设计要求：

1. `MoveIt Servo` 可作为实时目标源接入 `Eyou_ROS1_Master`
2. 电机侧运动执行统一经过多轴 `Ruckig`
3. 多轴运动以同一执行内核做同步规划，避免不同后端/不同前端各自生成命令
4. 不引入明显技术债务，不复制第二套“临时运动执行器”

---

## 2. 现状

### 2.1 当前 `MoveIt Servo` 路径

当前仓库中：

- `car_control/launch/moveit_servo.launch` 只启动 `servo_server`
- `car_control/config/moveit_servo.yaml` 配置：
  - `command_out_type: trajectory_msgs/JointTrajectory`
  - `command_out_topic: /arm_controller/command`

也就是说，当前 `MoveIt Servo` 默认输出到一个标准 `JointTrajectoryController` 话题。

### 2.2 当前 `Eyou_ROS1_Master` 路径

当前 `Eyou_ROS1_Master` 默认：

- 启动统一节点 `hybrid_motor_hw_node`
- 默认只加载 `joint_state_controller`
- 可选启用内部 `IpFollowJointTrajectoryExecutor`
- 默认 action namespace 为 `/arm_position_controller/follow_joint_trajectory`

现有内部 IP executor 已经使用一个共享的多轴 `Ruckig` 实例，按整组 joint 做在线更新。

### 2.3 当前缺口

因此当前系统里实际上存在两条未接通的链：

1. `MoveIt Servo -> /arm_controller/command`
2. `Eyou_ROS1_Master -> 内部 IP executor -> PositionJointInterface`

它们目前不是同一套执行内核，也没有统一的命令源仲裁。

---

## 3. 设计目标

### 3.1 功能目标

- 接受来自 `MoveIt Servo` 的连续目标点
- 电机实际执行走统一的多轴 `Ruckig`
- 多轴目标按同一同步策略生成，使各轴同时间到达
- `FollowJointTrajectory` action 继续可用

### 3.2 架构目标

- 只有一个机械臂运动执行内核
- 只有一个机械臂命令写入者
- `Servo` 与 `Action` 只是两种前端输入源
- `can_driver` 与 `Eyou_Canopen_Master` 仍由 `HybridRobotHW` 统一承载

### 3.3 非目标

- 不在本阶段引入第二套新的外部运动控制包
- 不让 `MoveIt Servo` 直接绕过 `Eyou_ROS1_Master` 写电机
- 不保留“`Servo -> arm_controller`”与“`Action -> internal executor`”长期并存的双轨架构

---

## 4. 核心判断

### 4.1 可以接受 `MoveIt Servo` 目标点，再由电机侧做时间最优同步规划

这是可行的。

但语义上必须明确：

- 对 `FollowJointTrajectory` 这类离散完整轨迹，可以谈“整段轨迹时间最优参数化”
- 对 `MoveIt Servo` 这类实时连续更新目标，只能谈：
  - “针对当前最新目标点的在线、多轴同步、约束下最优更新”

不能把后者表述成“整段全局时间最优轨迹”，否则会造成文档和行为预期偏差。

### 4.2 不建议直接使用现有 `Servo -> arm_controller`

原因：

1. 当前 `arm_controller` 路径里没有统一的 `Ruckig` 执行层
2. 会形成与内部 IP executor 并行的第二条运动链
3. 后续很容易出现：
   - 不同前端不同手感
   - 不同前端不同安全行为
   - 同一组 joint 被两套执行器竞争写入

---

## 5. 推荐架构

推荐采用“单执行内核、双前端”的结构。

```text
MoveIt Servo --------------------\
                                  \
                                   > HybridJointTargetExecutor -> PositionJointInterface -> HybridRobotHW -> canopen/can_driver
                                  /
FollowJointTrajectory action ----/
```

### 5.1 统一执行内核

新增一个统一的机械臂目标执行内核，建议命名：

- `HybridJointTargetExecutor`

建议文件：

- `Eyou_ROS1_Master/include/Eyou_ROS1_Master/hybrid_joint_target_executor.hpp`
- `Eyou_ROS1_Master/src/hybrid_joint_target_executor.cpp`

职责：

1. 持有统一的机械臂 joint 列表
2. 从 `JointStateInterface` 读取当前状态
3. 持有一份共享的多轴 `Ruckig`
4. 每周期根据当前 source 生成统一位置命令
5. 把结果写入 `PositionJointInterface`

这部分应成为系统中唯一的“机械臂位置命令写入者”。

### 5.2 Action 前端

现有 `IpFollowJointTrajectoryExecutor` 不再长期保留完整的独立执行状态机。

建议拆成两层：

1. `FollowJointTrajectory` 前端
   - 校验 goal
   - 管理 action 生命周期
   - 提供离散 waypoint 目标
2. 统一执行内核
   - 实际执行离散 waypoint 的时序推进
   - 用共享多轴 `Ruckig` 生成每周期位置命令

也就是说，action 前端保留，但运动生成逻辑统一下沉。

### 5.3 Servo 前端

新增一个轻量前端，建议命名：

- `HybridServoBridge`

建议文件：

- `Eyou_ROS1_Master/include/Eyou_ROS1_Master/hybrid_servo_bridge.hpp`
- `Eyou_ROS1_Master/src/hybrid_servo_bridge.cpp`

职责：

1. 订阅 `MoveIt Servo` 输出的 `trajectory_msgs/JointTrajectory`
2. 只保留“最新一条”消息，不做排队
3. 把消息转换为统一执行内核可消费的“最新目标状态”
4. 超时后通知执行内核进入 hold

该前端不直接写 `PositionJointInterface`。

---

## 6. Servo 输入约定

### 6.1 话题建议

将 `MoveIt Servo` 输出话题从：

- `/arm_controller/command`

改为：

- `/hybrid_motor_hw_node/servo_joint_targets`

消息类型仍使用：

- `trajectory_msgs/JointTrajectory`

这样改动最小：

- 不需要改 `MoveIt Servo` 输出类型
- 只需要改 `command_out_topic`
- `Eyou_ROS1_Master` 负责消费该消息并走统一执行内核

### 6.2 消息消费规则

对每条 `JointTrajectory`：

1. 必须能映射到统一的机械臂 joint 名集合
2. 只取最后一个 point 作为当前目标点
3. 若提供 velocity，则作为目标速度
4. 若未提供 velocity，则使用 0
5. 若 point 为空，则忽略

选择“只取最新 point”的原因：

- `MoveIt Servo` 是流式控制源
- 它的输出本质上是实时更新目标，不应积压旧目标

---

## 7. 统一执行内核行为

### 7.1 执行模式

统一执行内核维护三种 source 状态：

- `Idle`
- `Servo`
- `TrajectoryAction`

同一时刻只允许一个 source 对机械臂 joint 生效。

### 7.2 Servo 模式

每周期执行：

1. 读取当前 joint 状态
2. 读取最新 Servo 目标
3. 用多轴 `Ruckig` 计算：
   - `current state -> latest target state`
4. 输出新的位置命令

这是在线流式追踪，不是全局一次性整段轨迹规划。

### 7.3 Action 模式

每周期执行：

1. 读取当前 active goal 的当前目标段
2. 用同一个多轴 `Ruckig` 做在线推进
3. 到达当前 segment 终点后切到下一段
4. 全段完成后回到 `Idle`

### 7.4 Hold 模式

当以下任一条件成立时，执行内核进入 hold：

- 无 active source
- Servo 输入超时
- 生命周期不允许运动
- global fault / all_axes_halted_by_fault

hold 的输出应为：

- 位置锁当前
- 速度为 0

---

## 8. Ruckig 使用规范

### 8.1 同步策略

执行内核中不应依赖 Ruckig 默认行为，必须显式设置：

- `input_.synchronization = ruckig::Synchronization::Time`
- `input_.duration_discretization = ruckig::DurationDiscretization::Discrete`

原因：

1. 语义必须写进代码，而不是隐含在第三方默认值里
2. 控制循环是离散周期，离散时长更符合控制器执行现实

### 8.2 约束来源

每轴约束统一来自已有配置：

- `ip_max_velocity`
- `ip_max_acceleration`
- `ip_max_jerk`
- `ip_goal_tolerance`

这些约束已经在 `hybrid` 侧对两类 backend 有统一聚合基础，不应再新建第三份参数源。

### 8.3 时间最优语义

应在文档、日志、测试中统一使用以下表述：

- `FollowJointTrajectory`：离散轨迹的多轴同步在线执行
- `MoveIt Servo`：连续最新目标点的多轴同步在线最优逼近

不使用“Servo 全局时间最优轨迹”这一说法。

---

## 9. 关于追踪滞后

### 9.1 定义

追踪滞后是指：

- 上游 `Servo` 目标持续变化
- 下游执行器因速度/加速度/jerk 约束无法瞬时跟上
- 所以实际命令总是略落后于最新目标

### 9.2 这是不是错误

不是。

这是所有受约束在线运动生成都会出现的自然结果。

### 9.3 会带来的影响

- 手感更平滑，但更“软”
- 快速反向会有拖尾
- 高频小步变化会被再次平滑
- 停止响应可能慢于直接裸跟随

### 9.4 为什么仍然接受

因为本项目目标不是“完全裸跟随最新输入”，而是：

- 同步
- 平滑
- 可实现
- 可控
- 可维护

只要把滞后控制在合理范围内，它是可接受的工程折中。

---

## 10. 无技术债原则

为避免留下长期技术债，必须遵守以下原则：

### 10.1 不保留两套运动执行器

不允许长期存在：

- `Servo -> arm_controller`
- `Action -> internal executor`

两套机械臂运动链同时作为正式路径。

### 10.2 不复制第二份 Ruckig 逻辑

`Servo` 不应再单独写一套新的多轴 Ruckig 状态机。

统一执行内核是唯一合法位置。

### 10.3 不允许双写 PositionJointInterface

同一时刻只能有一个模块写机械臂 joint 的位置命令。

### 10.4 不新增第二份机械臂参数源

所有速度/加速度/jerk/tolerance 约束统一复用现有配置项。

---

## 11. 最小实施路径

建议按以下顺序实施：

### 步骤 1

抽出统一执行内核：

- `HybridJointTargetExecutor`

并先让现有 `FollowJointTrajectory` action 前端改为走这套内核。

目的：

- 先把“单执行内核”架构落稳
- 避免后续接 Servo 时再做一次重构

### 步骤 2

新增 `HybridServoBridge`，订阅：

- `/hybrid_motor_hw_node/servo_joint_targets`

并把目标喂给统一执行内核。

### 步骤 3

把 `car_control/config/moveit_servo.yaml` 的：

- `command_out_topic`

从 `/arm_controller/command` 改为：

- `/hybrid_motor_hw_node/servo_joint_targets`

### 步骤 4

补 source arbitration、timeout 和监控：

- 当前 source
- Servo 输入是否超时
- 统一执行器当前是否 hold
- 最近目标与实际误差

---

## 12. Commit 切分方案

为保证实现过程不留下结构性债务，建议按以下 commit 顺序推进。

### Commit 1

提交信息建议：

- `refactor(Eyou_ROS1_Master): extract shared multi-axis joint target executor`

目标：

- 抽出统一的 `HybridJointTargetExecutor`
- 让“多轴 Ruckig + PositionJointInterface 写入”成为独立内核

修改范围建议：

- `Eyou_ROS1_Master/include/Eyou_ROS1_Master/hybrid_joint_target_executor.hpp`
- `Eyou_ROS1_Master/src/hybrid_joint_target_executor.cpp`
- `Eyou_ROS1_Master/CMakeLists.txt`
- 必要的单元测试骨架

本 commit 不做：

- Servo 接入
- action 行为改造
- launch / YAML 改动

验收：

- 新内核可独立编译
- 能从 `RobotHW` 读取 joint state 并向 `PositionJointInterface` 写命令
- 显式设置 `Ruckig` 的 `Synchronization::Time`

### Commit 2

提交信息建议：

- `refactor(Eyou_ROS1_Master): route FollowJointTrajectory through shared executor`

目标：

- 让现有 `FollowJointTrajectory` 前端改为调用统一执行内核
- 去掉 action 路径中的独立运动生成逻辑

修改范围建议：

- `Eyou_ROS1_Master/src/hybrid_motor_hw_node.cpp`
- 现有 action executor 相关 glue code
- `Eyou_ROS1_Master/tests/test_hybrid_launch_ip_executor.cpp`
- 新增/修改执行器相关测试

本 commit 不做：

- Servo 输入桥接
- MoveIt Servo 配置修改

验收：

- 现有 `/arm_position_controller/follow_joint_trajectory` 仍可用
- action 与原先相比对外接口不变
- action 实际执行走统一执行内核

### Commit 3

提交信息建议：

- `feat(Eyou_ROS1_Master): add servo joint target bridge`

目标：

- 新增 `HybridServoBridge`
- 订阅 Servo 输出的 `trajectory_msgs/JointTrajectory`
- 只保留 latest target，不排队

修改范围建议：

- `Eyou_ROS1_Master/include/Eyou_ROS1_Master/hybrid_servo_bridge.hpp`
- `Eyou_ROS1_Master/src/hybrid_servo_bridge.cpp`
- `Eyou_ROS1_Master/src/hybrid_motor_hw_node.cpp`
- 相关单元测试

本 commit 不做：

- 修改 `car_control` 里的 Servo 输出 topic

验收：

- 新 bridge 可解析 joint names
- 能正确提取 latest point
- 输入超时后进入 hold

### Commit 4

提交信息建议：

- `feat(Eyou_ROS1_Master): add motion source arbitration for action and servo`

目标：

- 明确 `Idle / Servo / TrajectoryAction` 三态
- 保证同一时刻只有一个 source 生效
- 解决 action 与 Servo 抢写同一组 joint 的问题

修改范围建议：

- `HybridJointTargetExecutor`
- `HybridServoBridge`
- action glue 层
- 相关状态/监控输出

验收：

- Servo 激活时 action 不会并发写入
- action 激活时 Servo 输入不会破坏当前 goal
- source 切换有确定规则

### Commit 5

提交信息建议：

- `feat(car_control): send MoveIt Servo output into hybrid servo target topic`

目标：

- 修改 `MoveIt Servo` 输出目的地
- 从 `/arm_controller/command` 切到 `/hybrid_motor_hw_node/servo_joint_targets`

修改范围建议：

- `car_control/config/moveit_servo.yaml`
- `car_control/README.md`
- 如有需要，补充硬件 bringup 文档

本 commit 不做：

- 再保留“Servo 直接驱动 arm_controller”的正式路径

验收：

- Servo 输出 topic 指向 hybrid
- 不再依赖 `arm_controller` 作为 Servo 主路径

### Commit 6

提交信息建议：

- `test(Eyou_ROS1_Master): cover synchronized servo execution and timeout hold`

目标：

- 补足回归测试与监控验证

测试重点：

- Servo 连续输入时 canopen / can_driver joint 都有响应
- 多轴同步同到达
- 超时 hold
- source arbitration
- 生命周期异常时停止输出

验收：

- 关键链路有自动化测试兜底
- 文档与测试对“Servo 在线最优逼近”表述一致

### 不建议的 commit 方式

以下切法容易留下技术债，建议避免：

1. 先把 Servo 直接接到 `arm_controller`，后面再补统一执行内核
2. 先给 Servo 单独写一套 Ruckig，再后续考虑和 action 合并
3. 在同一个 commit 里同时做：
   - 执行内核抽象
   - action 重构
   - Servo 接入
   - YAML/launch 调整

这三种做法都会使回滚边界和行为归因变差。

---

## 13. 验收标准

### 13.1 架构验收

- 机械臂位置命令只有一个写入者
- action 与 Servo 都通过统一执行内核
- `MoveIt Servo` 不再直接驱动 `arm_controller`

### 13.2 行为验收

- Servo 连续输入时，canopen 与 can_driver 关节都能响应
- 多轴运动观察到同步同到达行为
- 停止输入后，系统按 timeout 进入 hold
- action 执行与 Servo 执行不会互相竞争

### 13.3 回归验收

- 现有 `hybrid` 生命周期链路不回归
- `/joint_states` 继续完整反映两类 backend
- `can_driver` 与 `Eyou_Canopen_Master` 的监控状态继续可用

---

## 14. 推荐结论

如果要在“不新增太多东西”的前提下接入 `MoveIt Servo`，推荐正式方案为：

1. 保留 `MoveIt Servo` 作为上游目标源
2. 新增一个轻量 Servo 前端桥接
3. 复用并统一现有多轴 `Ruckig` 执行能力
4. 让 `Servo` 与 `FollowJointTrajectory` 都走同一执行内核

这是当前最平衡、也最不容易留下技术债的方案。
