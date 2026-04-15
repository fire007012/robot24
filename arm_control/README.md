# arm_control

`arm_control` 是新的机械臂上层控制包。

当前阶段先创建包骨架，不迁移现有实现。后续计划逐步从 `car_control` 迁入或在本包新增以下节点：

- `arm_goal_executor_node`
  - 接收目标位姿或目标关节
  - 调用 MoveIt 做规划与执行
  - 北向接口为高层目标，而不是直接写电机
- `gripper_cmd_node`
  - 夹爪开合/位置控制
- `servo_twist_frame_bridge_node`
  - optical frame 到控制 frame 的 Servo 输入桥接

当前正式的电机执行与生命周期入口仍是 `Eyou_ROS1_Master/hybrid_motor_hw_node`。
