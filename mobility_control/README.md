# mobility_control

`mobility_control` 是新的底盘控制包。

当前阶段先创建包骨架，不迁移现有实现。后续计划逐步从 `car_control` 迁入以下节点：

- `base_cmd_node`
  - `/cmd_vel` 或中间速度话题的限幅与超时刹停
- `tracked_cmd_bridge_node`
  - Gazebo 履带插件桥接
- `gazebo_model_odom_node`
  - 仿真里程计重建

这里的职责仅限于底盘控制，不承担机械臂、摆臂或总控编排。
