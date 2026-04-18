# control

`control` 已退役为归档参考包，不再承担正式执行链职责。

保留内容：

- 达妙协议 PDF 与调试资料
- 旧版 `damiao_diff_chassis_node.cpp`
- 迁移时使用过的常量头文件与协议辅助代码

旧链路已经退役：

```text
/cmd_vel
  -> damiao_diff_chassis_node
    -> 差速逆运动学
      -> 达妙 CAN 直写
```

当前正式链路为：

```text
/cmd_vel
  -> mobility_control/base_cmd_node
    -> wheel_controller (diff_drive_controller)
      -> controller_manager / HybridRobotHW
        -> can_driver::DamiaoCan
```

职责拆分如下：

- `mobility_control`
  - `/cmd_vel` 限幅与超时刹停
- `Eyou_ROS1_Master`
  - 统一 `controller_manager` 与生命周期编排
- `can_driver`
  - 达妙协议后端、生命周期与 wheel joint 硬件接口

本包当前不会编译任何可执行节点，只作为源码归档与问题追溯参考保留。
