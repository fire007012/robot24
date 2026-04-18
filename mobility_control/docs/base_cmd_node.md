# `base_cmd_node`

`base_cmd_node` 是底盘控制北向入口，负责在进入 `wheel_controller` 之前完成两件事：

1. 对 `geometry_msgs/Twist` 的线速度与角速度做统一限幅。
2. 在上游命令超时后自动发送零速度，防止底盘持续运动。

## 来源

- 实现直接搬自 `car_control/scripts/base_cmd_node.py`
- 第一阶段只迁移到底盘正式链路，不重写算法逻辑

## 默认链路

默认参数文件见 `config/base_cmd.yaml`：

- 输入：`/cmd_vel`
- 输出：`/wheel_controller/cmd_vel`
- 最大线速度：`0.8 m/s`
- 最大角速度：`1.5 rad/s`
- 超时刹停：`0.3 s`

## 启动

```bash
roslaunch mobility_control base_cmd.launch
```

如果需要临时改 topic 或限幅，可直接覆盖 launch 参数，例如：

```bash
roslaunch mobility_control base_cmd.launch \
  input_topic:=/car_control/cmd_vel \
  output_topic:=/wheel_controller/cmd_vel
```

## 说明

- 该节点不直接写 CAN，不承载底盘逆运动学。
- 真正的左右履带速度拆解由 `diff_drive_controller` 完成。
- 旧 `control/src/damiao_diff_chassis_node.cpp` 中 `/cmd_vel -> 差速逆运动学 -> 达妙 CAN 直写` 的一体化路径已退役，只保留源码归档。
