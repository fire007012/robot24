# robot_bringup

顶层系统编排包，负责把底盘、摆臂、夹爪、MoveIt 和验收脚本组合成统一启动入口。

## 包职责

- 提供仿真和实机的顶层 launch 入口。
- 提供 MoveIt 键盘微调工具和拓扑就绪检查脚本。
- 约束上层统一入口，减少各子系统单独启动顺序问题。

## 包结构

```text
robot_bringup/
|-- launch/
|   |-- acceptance_smoke.launch
|   |-- full_system.launch
|   |-- full_system_simulate.launch
|   `-- keyboard_moveit_server.launch
`-- scripts/
    |-- keyboard_moveit_server.py
    `-- wait_for_graph.sh
```

## 快速开始

先跑仿真总入口：

```bash
cd ~/robot24_ws
catkin_make --pkg robot_bringup car_moveit_config car_urdf
source devel/setup.bash
roslaunch robot_bringup full_system_simulate.launch
```

实机入口：

```bash
roslaunch robot_bringup full_system.launch
```

`full_system.launch` 通过 `Eyou_ROS1_Master` 统一拉起实机硬件门面；当前默认控制路径为 `can_driver`，其中旋转台和大臂两轴也已按 MT 电机接入，不再要求顶层显式传入 CANopen 配置。

## 常用命令

```bash
# 无界面冒烟验收
roslaunch robot_bringup acceptance_smoke.launch

# 终端 1：最小化启动实机硬件节点，保持运行
roslaunch Eyou_ROS1_Master hybrid_motor_hw.launch spawn_controllers:=false auto_init:=false auto_enable:=false auto_release:=false profile_auto_lifecycle:=false enable_ecb_control:=false

# 终端 2：只启动 joint_state_controller，并触发一次硬件初始化/位置同步
rosrun controller_manager spawner joint_state_controller
rosservice call /hybrid_motor_hw_node/init
rostopic echo -n 1 /joint_states

# 确认 main_arm 六轴不是全 0 后，从当前 /joint_states 采样并写回 MoveIt/Gazebo 初始姿态
rosrun robot_bringup capture_arm_initial_pose.py

# 启动 MoveIt 键盘微调
roslaunch robot_bringup keyboard_moveit_server.launch

# 仿真入口，关闭 RViz
roslaunch robot_bringup full_system_simulate.launch enable_rviz:=false

# 实机入口，关闭 RViz
roslaunch robot_bringup full_system.launch enable_rviz:=false
```

## 节点与接口摘要

- `full_system_simulate.launch`
  - 启动 Gazebo、底盘控制、摆臂控制、夹爪命令、MoveIt、RViz。
- `full_system.launch`
  - 以 `Eyou_ROS1_Master` 为硬件统一门面，再接入同一套上层控制节点。
- `keyboard_moveit_server.py`
  - 笛卡尔平移：`w/a/s/d/r/f`
  - 姿态调节：`i/j/k/l/u/o`
  - 停止：`Space`
  - 退出：`q`
- `capture_arm_initial_pose.py`
  - 从 `/joint_states` 读取 `main_arm` 六个关节当前位置。
  - 更新 `car_moveit_config/config/car_urdf.srdf` 与 `car_urdf_gazebo.srdf` 的 `up` 姿态。
  - 更新仿真入口中的 `initial_joint_positions`，让后续整车仿真从当前实机姿态起步。
- `wait_for_graph.sh`
  - 检查 `/gazebo/model_states`
  - 检查 `/wheel_controller/cmd_vel`
  - 检查 `/flipper_control/state`
  - 检查 `/move_group/status`
  - 检查两条 `follow_joint_trajectory` goal 接口
  - 检查 `/joint_states`

## 详细文档索引

- `README.md`：当前维护入口。
- `docs/README.md`：包内文档索引。
