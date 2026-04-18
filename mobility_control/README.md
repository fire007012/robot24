# mobility_control

`mobility_control` 是新的底盘控制包，负责承接底盘北向控制链。

当前正式底盘执行链已经收敛为：

```text
/cmd_vel -> mobility_control/base_cmd_node -> wheel_controller -> can_driver
```

当前阶段已经正式迁入：

- `base_cmd_node`
  - 来自 `car_control/scripts/base_cmd_node.py` 的直接搬迁版本
  - 默认把 `/cmd_vel` 整形成 `/wheel_controller/cmd_vel`
  - 提供限幅与超时刹停
  - 不直接写 CAN，也不再承载旧达妙节点里的差速执行逻辑

后续仍计划逐步从 `car_control` 迁入以下节点：

- `tracked_cmd_bridge_node`
  - Gazebo 履带插件桥接
- `gazebo_model_odom_node`
  - 仿真里程计重建

这里的职责仅限于底盘控制，不承担机械臂、摆臂或总控编排。

旧 `control/src/damiao_diff_chassis_node.cpp` 已退役为归档参考实现，不再属于正式 bringup 路径。

常用启动方式：

```bash
roslaunch mobility_control base_cmd.launch
```
