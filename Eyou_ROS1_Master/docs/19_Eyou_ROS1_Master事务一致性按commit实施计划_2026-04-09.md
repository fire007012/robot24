# Eyou_ROS1_Master 事务一致性按 Commit 实施计划（2026-04-09）

## 1. 目标

本计划用于修复 `Eyou_ROS1_Master` 当前已确认的 3 个一致性问题：

1. `Enable / Disable / Release / Halt` rollback 失败后不进入 fail-safe shutdown
2. CANopen 自动启动旁路 hybrid facade
3. `RequestInit()` 丢失 `"already initialized"` 语义

目标是按最小、可验证、可回滚的 commit 粒度推进。

## 2. Commit 0：冻结问题描述与验收口径

### 目的

在动代码前，先固定问题定义和验收标准，避免“修了但没有共同目标”。

### 修改内容

- 新增问题修复报告：
  - `Eyou_ROS1_Master/docs/18_Eyou_ROS1_Master事务一致性问题修复报告_2026-04-09.md`
- 新增本计划：
  - `Eyou_ROS1_Master/docs/19_Eyou_ROS1_Master事务一致性按commit实施计划_2026-04-09.md`
- 更新包内索引：
  - `Eyou_ROS1_Master/docs/README.md`

### 建议提交信息

- `docs: add hybrid lifecycle consistency report and commit plan`

### 验收

- 问题边界明确
- 团队对修复优先级达成一致

## 3. Commit 1：补齐 4 条 rollback-failure 的 fail-safe shutdown

### 目的

让 `HybridOperationalCoordinator` 的以下 4 条路径在 rollback 失败时，
统一进入 fail-safe shutdown，而不是直接返回错误：

- `RequestEnable`
- `RequestDisable`
- `RequestRelease`
- `RequestHalt`

### 修改文件

- `Eyou_ROS1_Master/src/hybrid_operational_coordinator.cpp`

### 修改策略

对于每条路径：

1. 保留现有的“先记录前态，再调用两边，再尝试回滚”逻辑
2. 当回滚失败时：
   - 对两边都尝试执行 `RequestShutdown`
   - 把最终消息拼成：
     - 主错误
     - rollback 失败信息
     - fail-safe shutdown 信息
3. 保证最终尽量收敛到 `Configured`

### 验收

- 后半段失败且 rollback 失败时：
  - 返回错误
  - 两边最终都不应停留在 `Armed/Running`
  - 最好统一收敛到 `Configured`

### 建议提交信息

- `fix(Eyou_ROS1_Master): fail safe on rollback failure for hybrid lifecycle`

## 4. Commit 2：为 rollback-failure 场景补单元测试

### 目的

把 Commit 1 的行为锁死，避免后续回归。

### 修改文件

- `Eyou_ROS1_Master/tests/test_hybrid_operational_coordinator.cpp`
- `Eyou_ROS1_Master/CMakeLists.txt`

### 新增测试

建议至少补 4 条：

1. `EnableFailureRollbackFailureFallsBackToConfigured`
2. `DisableFailureRollbackFailureFallsBackToConfigured`
3. `ReleaseFailureRollbackFailureFallsBackToConfigured`
4. `HaltFailureRollbackFailureFallsBackToConfigured`

### 测试策略

- 通过 fake can_driver / fake canopen 协调器状态，构造：
  - 前半成功
  - 后半失败
  - rollback 再失败
- 验证：
  - 返回 `ok=false`
  - 最终状态收敛到 `Configured`
  - 错误消息包含主错误来源

### 建议提交信息

- `test(Eyou_ROS1_Master): cover rollback failure fail-safe paths`

## 5. Commit 3：收口 CANopen 自动启动旁路

### 目的

让自动启动路径不再绕开 hybrid facade。

### 修改文件

- `Eyou_ROS1_Master/src/hybrid_motor_hw_node.cpp`

### 修改策略

建议二选一：

#### 方案 A：彻底禁用 hybrid 下的 CANopen 原生自动启动

- 在 hybrid 节点里忽略 `auto_init / auto_enable / auto_release`
- 只保留人工经由 hybrid service 的启动路径

适用场景：

- 你希望 facade 成为唯一权威入口

#### 方案 B：把 auto 启动改为经由 hybrid coordinator 触发

- 不再直接调用 `CanopenStartupSequence::Run(canopen_coord, ...)`
- 改成顺序调用 hybrid 侧：
  - `RequestInit`
  - `RequestEnable`
  - `RequestRelease`
- 必要时再在成功后执行 hook

适用场景：

- 你仍然需要自动启动能力
- 但必须确保两边一起进入同一状态

我建议优先选 `方案 A`，因为更简单、更不容易引入新分叉。

### 验收

- 打开 `auto_init/auto_enable/auto_release` 时，不会只推进 CANopen 一侧
- facade 始终是唯一生命周期入口

### 建议提交信息

- `fix(Eyou_ROS1_Master): route auto startup through hybrid lifecycle authority`

## 6. Commit 4：恢复 “already initialized” 语义透传

### 目的

让重复调用 `~/init` 时，`HybridServiceGateway` 能正确跳过 `post_init_hook`。

### 修改文件

- `Eyou_ROS1_Master/src/hybrid_operational_coordinator.cpp`
- `Eyou_ROS1_Master/src/hybrid_service_gateway.cpp`

### 修改策略

1. 在 `RequestInit()` 中读取两边初始 mode
2. 若两边本来都已经处于 `Armed`，直接返回：
   - `ok=true`
   - `message="already initialized"`
3. 若出现一边已 Armed、一边未 Armed 的不一致状态：
   - 优先按“异常状态”处理
   - 不要伪装成 already

### 验收

- 首次 init：
  - hook 会执行
  - 返回 `initialized (armed)` 或等价成功信息
- 重复 init：
  - hook 不会执行
  - 返回 `already initialized`

### 建议提交信息

- `fix(Eyou_ROS1_Master): preserve already initialized semantics for hybrid init`

## 7. Commit 5：补 `HybridServiceGateway` hook 语义测试

### 目的

验证 `post_init_hook` 与 rollback 的组合语义。

### 修改文件

- 新增：
  - `Eyou_ROS1_Master/tests/test_hybrid_service_gateway.cpp`
- 修改：
  - `Eyou_ROS1_Master/CMakeLists.txt`

### 建议测试点

1. 首次 init 成功后执行 hook
2. 重复 init 不执行 hook
3. hook 失败时执行 shutdown 回滚
4. rollback 失败时错误消息包含 hook failure + rollback failure

### 建议提交信息

- `test(Eyou_ROS1_Master): verify hybrid init hook and rollback semantics`

## 8. Commit 6：对齐修复报告与实现

### 目的

当代码与测试都通过后，再更新报告，确保文档与实现一致。

### 修改文件

- `Eyou_ROS1_Master/docs/18_Eyou_ROS1_Master事务一致性问题修复报告_2026-04-09.md`
- 视情况更新：
  - `docs/17_can_driver解耦与Eyou_ROS1_Master事务回滚修复报告_2026-04-09.md`

### 修改内容

- 将“待修复问题”改为“已修复项”
- 补充 commit 记录和验证结果
- 明确剩余风险

### 建议提交信息

- `docs(Eyou_ROS1_Master): update hybrid lifecycle consistency report after remediation`

## 9. 推荐执行顺序

1. Commit 0
2. Commit 1
3. Commit 2
4. Commit 3
5. Commit 4
6. Commit 5
7. Commit 6

## 10. 每个 Commit 的统一验收模板

```bash
# 重新配置并编译相关包
catkin_make \
  -DCATKIN_WHITELIST_PACKAGES='can_driver;Eyou_Canopen_Master;Eyou_ROS1_Master' \
  --force-cmake \
  --pkg can_driver Eyou_Canopen_Master Eyou_ROS1_Master

# can_driver 既有回归
/home/dianhua/Robot24_catkin_ws/devel/lib/can_driver/test_can_driver_hw_smoke

# Eyou_ROS1_Master 单测
/home/dianhua/Robot24_catkin_ws/devel/lib/Eyou_ROS1_Master/test_hybrid_operational_coordinator
```

若新增 `HybridServiceGateway` 单测，则一并加入。

## 11. 结论

本计划的核心思想是：

- 先补 fail-safe shutdown 的一致性缺口
- 再收口自动启动旁路
- 最后修语义细节与测试覆盖

这样既能优先解决最危险的混合态问题，也能保持每个 commit 都可独立验证、可独立回滚。
