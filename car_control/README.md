# car_control

用于控制底盘移动、夹爪移动，以及 DS4/DS5 手柄控制 MoveIt Servo。

## 坐标系约定

- 相机传感器话题默认发布在 `catch_camera_optical_frame`
- DS5 遥操作和 MoveIt Servo 控制命令默认使用 `catch_camera`
- 不要直接把 optical frame 的 Twist 当作 Servo 控制输入，否则会出现“相机前方像是 z 轴正方向”的轴置换现象

当前这套模型里：

- `catch_camera_optical_frame.z` 对应 `catch_camera.x`
- `catch_camera_optical_frame.x` 对应 `-catch_camera.y`
- `catch_camera_optical_frame.y` 对应 `-catch_camera.z`

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
- 输出2：`/servo_server/delta_twist_cmds` (`geometry_msgs/TwistStamped`，MoveIt Servo，frame=`catch_camera`）
- 输出3：`/car_control/gripper_open`（夹爪开闭）
- 功能：
  - `Options` 切换模式：`CHASSIS` / `ARM_SERVO`
  - `Square` 打开夹爪，`Circle` 闭合夹爪

4. `servo_twist_frame_bridge_node.py`
- 输入：`/car_control/delta_twist_cmds_optical` (`geometry_msgs/TwistStamped`)
- 输出：`/servo_server/delta_twist_cmds` (`geometry_msgs/TwistStamped`)
- 功能：将视觉/相机链路里使用 `catch_camera_optical_frame` 的 Twist 转换到 `catch_camera`

## 启动

仅底盘+夹爪：
```bash
roslaunch car_control control_nodes.launch
```

底盘+夹爪+DS4/DS5+MoveIt Servo：
```bash
roslaunch car_control teleop_system.launch joy_dev:=/dev/input/js0
```

`teleop_system.launch`、`sim_test.launch`、`demo_gazebo.launch` 现在默认会一起启动
`servo_twist_frame_bridge_node.py`。若你只单独启动了 MoveIt Servo，又需要接
optical frame 的视觉 Twist，可以单独补启动桥接：
```bash
roslaunch car_control servo_twist_frame_bridge.launch
```

## 手柄映射（默认）

- 模式切换：`Options`
- 底盘模式：
  - 左摇杆 `LY`：前后
  - 左摇杆 `LX`：原地转向
- 机械臂 Servo 模式：
  - 左摇杆：`linear y/z`
  - `L2/R2`：`linear x` 正/负
  - `L1/R1`：`angular x`（roll）正/负
  - 右摇杆左右：`angular z`（yaw）
  - 右摇杆上下：`angular y`（pitch，方向已反转）

说明：
- 手柄输出的 `TwistStamped.header.frame_id` 默认是 `catch_camera`

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
'{header: {frame_id: "catch_camera"}, twist: {linear: {x: 0.05, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}}'
```

Optical frame 输入经桥接后再送给 MoveIt Servo：
```bash
roslaunch car_control servo_twist_frame_bridge.launch

rostopic pub -1 /car_control/delta_twist_cmds_optical geometry_msgs/TwistStamped \
'{header: {frame_id: "catch_camera_optical_frame"}, twist: {linear: {x: 0.0, y: 0.0, z: 0.05}, angular: {x: 0.0, y: 0.0, z: 0.0}}}'
```
