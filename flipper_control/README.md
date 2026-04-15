# flipper_control

`flipper_control` 是四摆臂上层控制包，当前第一阶段已经落地。

## 当前实现

- `flipper_manager_node`
  - 接收 `/flipper_control/command` 与 `/flipper_control/jog_cmd`
  - 支持 `csp_position`、`csp_jog`、`csv_velocity`
  - 负责 `CSP <-> CSV` 冷切换
  - 负责 `csp_position <-> csp_jog` 无冷切换切换
  - 负责联动展开、关节限位、命令超时与 `q_ref` 参考生成
- 消息与服务
  - `flipper_control/FlipperControlState`
  - `flipper_control/SetControlProfile`
  - `flipper_control/SetLinkageMode`

## 北向接口

- `/flipper_control/command`
- `/flipper_control/jog_cmd`
- `/flipper_control/set_control_profile`
- `/flipper_control/set_linkage_mode`
- `/flipper_control/state`
- `/flipper_control/active_profile`

## 联动模式

- `independent`
- `left_right_mirror`
  - 前组：`left_front_arm_joint + right_front_arm_joint`
  - 后组：`left_rear_arm_joint + right_rear_arm_joint`
- `front_rear_sync`
  - 四摆臂整组同步
- `side_pair`
  - 左组：`left_front_arm_joint + left_rear_arm_joint`
  - 右组：`right_front_arm_joint + right_rear_arm_joint`
- `diagonal_pair`
  - 对角配对：`left_front_arm_joint + right_rear_arm_joint`
  - 对角配对：`right_front_arm_joint + left_rear_arm_joint`

当前实现采用“同号展开”，不在管理层内隐式做镜像符号翻转。若实机校准后需要反号联动，应增加显式可配置符号矩阵。

## 运行方式

```bash
roslaunch flipper_control flipper_control.launch
```

## 当前边界

- 摆臂执行仍依赖底层主站与 `hybrid_motor_hw_node`，本包不重复实现电机执行层
- `cst_torque` 尚未实现
- 当前已通过编译与参考生成器单元测试，尚未完成实机联调
- CANopen 摆臂节点号当前按 `1~4` 假设配置，现场接线后需再次确认
