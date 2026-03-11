# Splitter 测试执行指南

## 测试环境

所有测试脚本和 launch 文件已就绪，位于 `arm_traj_splitter` 包中。

---

## 测试清单

### ✅ 测试 1: 正常路径（已通过）

**目的**：验证 6 轴轨迹正确拆分为 4+2

```bash
# 终端 1
roslaunch arm_traj_splitter test_splitter.launch

# 终端 2
rosrun arm_traj_splitter send_test_goal.py
```

**预期**：SUCCEEDED (3)

---

### ✅ 测试 2: 关节顺序打乱（已通过）

**目的**：验证按名字匹配而非索引

```bash
# 终端 1（使用相同的 launch）
roslaunch arm_traj_splitter test_splitter.launch

# 终端 2
rosrun arm_traj_splitter send_test_goal_shuffled.py
```

**预期**：SUCCEEDED (3)，且位置值正确映射

---

### 测试 3: CANopen 侧 Abort

**目的**：验证错误传播，一侧失败导致整体失败

```bash
# 终端 1
roslaunch arm_traj_splitter test_abort_canopen.launch

# 终端 2
rosrun arm_traj_splitter send_test_goal.py
```

**预期**：
- CANopen 侧打印 `[FakeCANopen] 模拟 ABORT`
- PP 侧可能被 cancel（如果还在执行）
- 客户端收到 `ABORTED (4)`
- Splitter 打印错误信息包含 "canopen_arm_controller"

---

### 测试 3b: PP 侧 Abort

**目的**：验证另一侧失败的情况

```bash
# 终端 1
roslaunch arm_traj_splitter test_abort_pp.launch

# 终端 2
rosrun arm_traj_splitter send_test_goal.py
```

**预期**：
- PP 侧打印 `[FakePP] 模拟 ABORT`
- CANopen 侧可能被 cancel
- 客户端收到 `ABORTED (4)`
- Splitter 打印错误信息包含 "pp_arm_controller"

---

### 测试 4: 超时处理

**目的**：验证一侧超时不会导致死锁

```bash
# 终端 1
roslaunch arm_traj_splitter test_timeout.launch

# 终端 2
rosrun arm_traj_splitter send_test_goal.py
```

**预期**：
- CANopen 侧打印 `[FakeCANopen] 模拟超时...`
- 客户端在 10 秒后超时，打印 `执行超时!`
- 不会死锁，最终返回失败

**注意**：当前 splitter 没有内部超时机制，依赖客户端超时。

---

### 测试 5: 上游 Cancel

**目的**：验证 preempt 正确转发到两个后端

```bash
# 终端 1
roslaunch arm_traj_splitter test_cancel.launch

# 终端 2
rosrun arm_traj_splitter send_and_cancel.py
```

**预期**：
- 两个假 server 都打印 `收到 CANCEL`
- 客户端收到 `结果状态: 2 (2=PREEMPTED)`

---

### 测试 6: 畸形输入

**目的**：验证鲁棒性，不会 crash

#### 6a. 空轨迹

修改 `send_test_goal.py` 临时测试：
```python
names = []
waypoints = []
times = []
```

**预期**：splitter 返回 ABORTED，错误信息 "Trajectory has empty joint_names"

#### 6b. 只有部分关节

```python
names = ['arm_joint_1', 'arm_joint_2']
waypoints = [[0.0, 0.0]]
times = [0.0]
```

**预期**：splitter 返回 ABORTED，错误信息 "Trajectory missing joints for backend: canopen_arm_controller"

#### 6c. 未知关节

```python
names = ['arm_joint_1', 'unknown_joint', 'arm_joint_5']
waypoints = [[0.0, 0.0, 0.0]]
times = [0.0]
```

**预期**：splitter 返回 ABORTED，错误信息 "Joint not assigned in splitter config: unknown_joint"

---

### 测试 7: 连续发 goal

**目的**：验证 preempt 逻辑

**手动方式**：
```bash
# 终端 1
roslaunch arm_traj_splitter test_cancel.launch  # 使用长执行时间

# 终端 2 - 快速连续运行两次
rosrun arm_traj_splitter send_test_goal.py
rosrun arm_traj_splitter send_test_goal.py  # 立即再运行
```

**预期**：第一个 goal 被 preempt，第二个正常执行完成

---

## 测试结果记录

| 测试 | 状态 | 备注 |
|------|------|------|
| 1. 正常路径 | ✅ 通过 | 拆分正确，��间戳保留 |
| 2. 关节顺序打乱 | ✅ 通过 | 名字匹配正确 |
| 3. CANopen Abort | ⏳ 待测 | |
| 3b. PP Abort | ⏳ 待测 | |
| 4. 超时 | ⏳ 待测 | |
| 5. Cancel | ⏳ 待测 | |
| 6. 畸形输入 | ⏳ 待测 | |
| 7. 连续 goal | ⏳ 待测 | |

---

## 监控工具

测试时可以开启额外终端监控：

```bash
# 监控 action 状态
rostopic echo /arm_controller/follow_joint_trajectory/status

# 监控 PP 侧 goal
rostopic echo /pp_arm_controller/follow_joint_trajectory/goal

# 监控 CANopen 侧 goal
rostopic echo /canopen/canopen_arm_controller/follow_joint_trajectory/goal

# 可视化节点关系
rqt_graph
```

---

## 文件清单

### Launch 文件
- `test_splitter.launch` - 正常测试（测试 1, 2）
- `test_abort_canopen.launch` - CANopen 侧 abort（测试 3）
- `test_abort_pp.launch` - PP 侧 abort（测试 3b）
- `test_timeout.launch` - 超时测试（测试 4）
- `test_cancel.launch` - Cancel 测试（测试 5）

### 测试脚本
- `send_test_goal.py` - 标准 6 轴轨迹
- `send_test_goal_shuffled.py` - 打乱顺序的轨迹
- `send_and_cancel.py` - 发送后立即 cancel

### 假 Server
- `fake_pp_arm_server.py` - PP 侧模拟
- `fake_canopen_arm_server.py` - CANopen 侧模拟
