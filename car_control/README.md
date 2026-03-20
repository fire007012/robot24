# car_control

用于控制底盘移动、夹爪移动，以及 DS4/DS5 手柄控制 MoveIt Servo。

## 节点

1. `base_cmd_node.py`
- 输入：`/car_control/cmd_vel` (`geometry_msgs/Twist`)
- 输出：`/cmd_vel` (`geometry_msgs/Twist`)
- 功能：速度限幅 + 超时自动刹停

2. `gripper_cmd_node.py`
- 输入1：`/car_control/gripper_position` (`std_msgs/Float64`)
- 输入2：`/car_control/gripper_open` (`std_msgs/Bool`)
- 输出：`/gripper_controller/command` (`trajectory_msgs/JointTrajectory`)
- 功能：按位置控制夹爪，或 Bool 一键开/合

3. `ds5_teleop_node.py`（兼容 DS4/DS5）
- 输入：`/joy` (`sensor_msgs/Joy`)
- 输出1：`/car_control/cmd_vel`（底盘）
- 输出2：`/servo_server/delta_twist_cmds` (`geometry_msgs/TwistStamped`，MoveIt Servo）
- 输出3：`/car_control/gripper_open`（夹爪开闭）
- 功能：
  - `Options` 切换模式：`CHASSIS` / `ARM_SERVO`
  - `Square` 打开夹爪，`Circle` 闭合夹爪

## 启动

仅底盘+夹爪：
```bash
roslaunch car_control control_nodes.launch
```

底盘+夹爪+DS4/DS5+MoveIt Servo：
```bash
roslaunch car_control teleop_system.launch joy_dev:=/dev/input/js0
```

## 手柄映射（默认）

- 模式切换：`Options`
- 底盘模式：
  - 左摇杆 `LY`：前后
  - 左摇杆 `LX`：原地转向
- 机械臂 Servo 模式：
  - 左摇杆：`linear y/z`
  - `R2/L2`：`linear x` 正/负
  - 右摇杆：`angular z/y`（yaw/pitch）
  - `L1/R1`：`angular x`（roll）

## 测试命令

底盘前进：
```bash
rostopic pub -1 /car_control/cmd_vel geometry_msgs/Twist \
'{linear: {x: 0.3, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
```

夹爪打开：
```bash
rostopic pub -1 /car_control/gripper_open std_msgs/Bool 'data: true'
```

夹爪闭合：
```bash
rostopic pub -1 /car_control/gripper_open std_msgs/Bool 'data: false'
```

MoveIt Servo 手动测试：
```bash
rostopic pub -1 /servo_server/delta_twist_cmds geometry_msgs/TwistStamped \
'{header: {frame_id: "base_link_root"}, twist: {linear: {x: 0.05, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}}'
```
