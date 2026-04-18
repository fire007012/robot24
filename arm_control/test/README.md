# `arm_control` Test Notes

## 合同测试

`test_execute_arm_goal_contract.py` 是纯 Python `unittest` 合同测试，目标是验证 `ExecuteArmGoal` action 生成后的基础导入与构造，不依赖 ROS master。

它覆盖：

- `named` goal 的基本构造
- `joint` goal 的基本构造
- `pose` goal 的基本构造
- `ExecuteArmGoalResult` 错误码常量存在

如果本地还没有生成 `arm_control.msg` 或 `geometry_msgs.msg`，测试会以 `skip` 结束，并提示先完成 catkin 构建和环境 `source`。

## 运行方式

先保证第 3 步里的 action 已经加入 `arm_control` 并成功生成消息，然后在工作区根目录执行：

```bash
catkin_make --pkg arm_control
source devel/setup.bash
python3 src/arm_control/test/test_execute_arm_goal_contract.py
```

如果你使用隔离构建，也可以在隔离后的环境里执行同一个 `unittest` 命令；关键前提仍然是 `ExecuteArmGoal.action` 已生成对应 Python 消息。

## 真实 ROS Smoke

下面这些不属于纯合同测试，仍然需要真实 ROS 运行环境：

- `named goal` / `joint goal` / `pose goal` 真正送进 `/arm_control/execute_goal`
- `move_group` 规划成功与失败路径
- TF 转换失败路径
- controller readiness 检查
- preempt 行为验证

这些 smoke 需要至少具备：

- `move_group`
- `controller_manager`
- `joint_state_controller`
- `arm_position_controller`
- `/arm_position_controller/follow_joint_trajectory`

如果只是检查 action 合同是否生成、字段和常量是否可用，运行本目录下的 `unittest` 就够了。
