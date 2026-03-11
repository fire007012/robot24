# arm_traj_splitter 测试计划（按 commit）

本计划按修复/改动提交精确列出测试目标与最小可重复步骤，便于回归与追踪。

## 1. 前置条件

- ROS 环境已加载：`source /opt/ros/noetic/setup.bash`
- 工作区已编译：`catkin_make --pkg arm_traj_splitter`
- splitter 参数文件已准备：`arm_traj_splitter/config/splitter.yaml`
- 后端 action 名称与配置一致：
  - `/pp_arm_controller/follow_joint_trajectory`
  - `/canopen/canopen_arm_controller/follow_joint_trajectory`

## 2. 测试工具

推荐使用“假 action server + 发送脚本”的方式进行离线验证。
若测试脚本放在 `arm_traj_splitter/scripts`，建议至少包含：

- `fake_pp_arm_server.py`
- `fake_canopen_arm_server.py`
- `send_test_goal.py`
- `send_and_cancel.py`

## 3. 按 commit 测试清单

### Commit `4934434`
**目标**：unnamed 容差不再被丢弃，按索引回退拆分。

**步骤**：
1. 启动两个 fake server。
2. 发送包含 `goal_tolerance`/`path_tolerance` 且 `name` 为空的 goal（长度与 `joint_names` 对齐）。

**期望**：
- splitter 日志出现 “index fallback for unnamed … entries”
- 两个后端都能收到拆分后的容差（`name` 被补全为 joint 名）。

---

### Commit `6a12673`
**目标**：索引构建从 O(n²) 降为 O(n) 且行为不变。

**步骤**：
1. 使用“关节顺序打乱”的输入发送 goal。
2. 比对拆分后的 joint 顺序和值是否保持正确对应。

**期望**：
- PP 收到 `arm_joint_1~4`，CANopen 收到 `arm_joint_5~6`。
- 值与原始 joint 名对应关系正确。

---

### Commit `2268a2f`
**目标**：smoke 启动可选图检查。

**步骤**：
1. `roslaunch robot_bringup acceptance_smoke.launch enable_graph_check:=true check_timeout_sec:=30`
2. 观察 `wait_for_graph.sh` 结果。

**期望**：
- PASS 时正常退出，FAIL 时提示缺失条目并返回非 0。

---

### Commit `cdf3ea7`
**目标**：canopen 进程韧性与 spawner 超时参数生效。

**步骤**：
1. `roslaunch yiyou_canopen yiyou_canopen.launch`
2. 手动 `kill` `canopen_motor_node` 进程。

**期望**：
- 约 2 秒内进程自动拉起。
- controller spawner 在超时内完成加载。

---

### Commit `ff23190`
**目标**：转换因子来源可追溯。

**步骤**：
1. 阅读 `yiyou_canopen/docs/12_canopen参数说明.md`
2. 对照 `motor.yaml` 中的常数。

**期望**：
- 文档给出公式、单位与重算方法。

---

### Commit `0173276`
**目标**：display.launch 不再依赖缺失 rviz 配置。

**步骤**：
1. `roslaunch car_total display.launch`

**期望**：
- 启动不因缺少 `urdf.rviz` 报错（RViz 默认配置可打开）。

---

### Commit `25d143b`
**目标**：文档状态口径统一。

**步骤**：
1. 检查以下文档的“当前状态”段落是否一致：
   - `docs/08_细化开发步骤与检查清单.md`
   - `docs/09_代码评估报告.md`
   - `docs/08_阶段变更记录.md`

**期望**：
- 均体现已完成项、排除项与入口文档一致。

## 4. 最小回归集

如需快速回归，仅执行：

1. Happy Path + 乱序 joint（覆盖 `4934434` + `6a12673`）
2. 一侧 abort + cancel（覆盖执行链路可靠性）
3. `acceptance_smoke.launch` 启动校验（覆盖 `2268a2f`）
