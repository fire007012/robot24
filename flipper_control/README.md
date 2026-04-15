# flipper_control

`flipper_control` 是新的四摆臂上层控制包。

当前阶段先创建包骨架。后续计划在本包实现：

- `flipper_manager_node`
  - 接收四摆臂目标或预置位
  - 负责限位、联动、速率限制和模式管理
  - 统一输出到 `/flipper_controller/command`

四摆臂执行仍依赖底层主站与 `hybrid_motor_hw_node` 所在的统一执行体系，不会在本包重复实现电机执行层。
