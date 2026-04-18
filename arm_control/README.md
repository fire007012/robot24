# arm_control

`arm_control` 是新的机械臂上层控制包。

当前阶段先创建包骨架，不迁移现有实现。后续计划逐步从 `car_control` 迁入或在本包新增以下节点：

- `arm_goal_executor_node`
  - 已新增：统一 Action 入口，支持 joints / named / pose 三类离散目标
- `gripper_cmd_node`
  - 已迁入：夹爪开合/位置控制
- `servo_twist_frame_bridge_node`
  - 已迁入：optical frame 到 control frame 的 Servo 输入桥接

当前正式的电机执行与生命周期入口仍是 `Eyou_ROS1_Master/hybrid_motor_hw_node`。

面向视觉抓取扩展的包内设计指南见：

- `docs/01_arm_control包设计指南_视觉抓取扩展.md`
- `docs/02_arm_goal_executor按commit实施计划.md`

当前可直接启动的节点：

- `roslaunch arm_control arm_goal_executor.launch`
- `roslaunch arm_control gripper_cmd.launch`
- `roslaunch arm_control servo_twist_frame_bridge.launch`

当前 smoke client：

- `rosrun arm_control send_execute_arm_goal.py named --name ready`
- `rosrun arm_control send_execute_arm_goal.py joints --joint shoulder_yaw_joint=0.0 --joint shoulder_pitch_joint=-0.2 --joint elbow_pitch_joint=1.0 --joint wrist_pitch_joint=0.0 --joint wrist_roll_joint=0.0 --joint wrist_yaw_joint=0.0`
- `rosrun arm_control send_execute_arm_goal.py pose --frame base_link --x 0.30 --y 0.00 --z 0.25 --qx 0.0 --qy 0.0 --qz 0.0 --qw 1.0`
