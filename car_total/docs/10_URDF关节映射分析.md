# URDF 关节映射分析

## 发现的 URDF 文件

**文件路径**：`~/Robot24_catkin_ws/src/car_total/urdf/car_total.urdf`

**验证结果**：
- ✅ URDF 语法正确
- ✅ 944 行完整定义
- ✅ 包含 mesh 文件引用

---

## URDF 中的关节列表

根据 `check_urdf` 输出和文件分析，URDF 包含以下关节：

### 底盘轮子（4个 continuous joints）
1. `left_ahead_wheel_joint` (continuous)
2. `left_back_wheel_joint` (continuous)
3. `right_ahead_wheel_joint` (continuous)
4. `right_back_wheel_joint` (continuous)

### 摆臂（4个 revolute joints）
5. `left_ahead_arm_joint` (revolute)
6. `left_back_arm_joint` (revolute)
7. `right_ahead_arm_joint` (revolute)
8. `right_back_arm_joint` (revolute)

### 机械臂（6个 revolute joints）
9. `big_course_joint` (revolute) - 大转台
10. `big_pitch_joint` (revolute) - 大臂俯仰
11. `small_pitch_joint` (revolute) - 小臂俯仰
12. `hands_pitch_joint` (revolute) - 手腕俯仰
13. `hands_roll_joint` (revolute) - 手腕翻滚
14. `hands_course_joint` (revolute) - 手腕旋转

### 传感器（固定连接）
- `camera` (link)
- `lidar` (link)

---

## 关键问题：关节命名不匹配！

### 当前配置中的命名

**can_driver.yaml**（私有协议，can1）：
- arm_joint_1
- arm_joint_2
- arm_joint_3
- arm_joint_4
- left_track
- right_track

**motor.yaml**（CANopen，can0）：
- arm_joint_5
- arm_joint_6
- flipper_1
- flipper_2
- flipper_3
- flipper_4

### URDF 中的命名

**机械臂**：
- big_course_joint
- big_pitch_joint
- small_pitch_joint
- hands_pitch_joint
- hands_roll_joint
- hands_course_joint

**底盘**：
- left_ahead_wheel_joint
- left_back_wheel_joint
- right_ahead_wheel_joint
- right_back_wheel_joint

**摆臂**：
- left_ahead_arm_joint
- left_back_arm_joint
- right_ahead_arm_joint
- right_back_arm_joint

---

## 问题分析

### 问题 1：机械臂关节数量不匹配

- **配置文件**：6 个关节（arm_joint_1~6）
- **URDF**：6 个关节（big_course_joint, big_pitch_joint, small_pitch_joint, hands_pitch_joint, hands_roll_joint, hands_course_joint）
- **问题**：名称完全不同

### 问题 2：底盘关节不匹配

- **配置文件**：2 个关节（left_track, right_track）
- **URDF**：4 个轮子（left_ahead_wheel_joint, left_back_wheel_joint, right_ahead_wheel_joint, right_back_wheel_joint）
- **问题**：
  - 配置假设是差速驱动（2个履带）
  - URDF 显示是 4 轮独立驱动
  - 需要确认实际硬件配置

### 问题 3：摆臂关节匹配

- **配置文件**：4 个关节（flipper_1~4）
- **URDF**：4 个摆臂（left_ahead_arm_joint, left_back_arm_joint, right_ahead_arm_joint, right_back_arm_joint）
- **问题**：名称不同，但数量匹配

---

## 解决方案

### 方案 A：修改配置文件以匹配 URDF（推荐）

**优点**：
- URDF 是从 SolidWorks 导出的，反映真实机械结构
- 保持 URDF 不变，避免破坏几何和惯性参数

**缺点**：
- 需要修改多个配置文件
- 需要重新编译（如果有硬编码）

**工作量**：约 2 小时

### 方案 B：修改 URDF 以匹配配置文件

**优点**：
- 配置文件已经完成
- 代码不需要修改

**缺点**：
- 修改 URDF 可能破坏几何关系
- 需要手动重命名所有关节
- 容易出错

**工作量**：约 3 小时

### 方案 C：创建映射层

**优点**：
- 两边都不需要大改
- 灵活性高

**缺点**：
- 增加系统复杂度
- 性能开销

**工作量**：约 4 小时

---

## 推荐方案：方案 A（修改配置文件）

### 步骤 1：确认硬件配置

**关键问题**：底盘是差速驱动还是 4 轮独立驱动？

**如果是差速驱动（2个履带）**：
- 左履带驱动：left_ahead_wheel + left_back_wheel（机械连接）
- 右履带驱动：right_ahead_wheel + right_back_wheel（机械连接）
- 在 URDF 中，前后轮通过 mimic 或传动链连接

**如果是 4 轮独立驱动**：
- 需要 4 个独立的速度控制器
- diff_drive_controller 不适用
- 需要使用 mecanum_drive_controller 或自定义控制器

### 步骤 2：更新关节命名映射表

#### 机械臂关节映射

| 配置文件名称 | URDF 名称 | 物理含义 | 驱动 |
|------------|----------|---------|------|
| arm_joint_1 | big_course_joint | 大转台 | can_driver (PP) |
| arm_joint_2 | big_pitch_joint | 大臂俯仰 | can_driver (PP) |
| arm_joint_3 | small_pitch_joint | 小臂俯仰 | can_driver (PP) |
| arm_joint_4 | hands_pitch_joint | 手腕俯仰 | can_driver (PP) |
| arm_joint_5 | hands_roll_joint | 手腕翻滚 | canopen |
| arm_joint_6 | hands_course_joint | 手腕旋转 | canopen |

#### 底盘关节映射（假设差速驱动）

| 配置文件名称 | URDF 名称 | 物理含义 | 驱动 |
|------------|----------|---------|------|
| left_track | left_ahead_wheel_joint | 左前轮（主驱动） | can_driver (MT) |
| right_track | right_ahead_wheel_joint | 右前轮（主驱动） | can_driver (MT) |
| - | left_back_wheel_joint | 左后轮（从动，mimic） | - |
| - | right_back_wheel_joint | 右后轮（从动，mimic） | - |

#### 摆臂关节映射

| 配置文件名称 | URDF 名称 | 物理含义 | 驱动 |
|------------|----------|---------|------|
| flipper_1 | left_ahead_arm_joint | 左前摆臂 | canopen |
| flipper_2 | left_back_arm_joint | 左后摆臂 | canopen |
| flipper_3 | right_ahead_arm_joint | 右前摆臂 | canopen |
| flipper_4 | right_back_arm_joint | 右后摆臂 | canopen |

### 步骤 3：修改配置文件

#### 3.1 修改 can_driver.yaml

```yaml
can_driver_node:
  control_frequency: 100.0
  debug_bypass_ros_control: false
  motor_query_hz: 0.0

  joints:
    # 机械臂前 4 轴（PP 协议）
    - name: big_course_joint
      motor_id: 0x06
      protocol: PP
      can_device: can1
      control_mode: position

    - name: big_pitch_joint
      motor_id: 0x05
      protocol: PP
      can_device: can1
      control_mode: position

    - name: small_pitch_joint
      motor_id: 0x13
      protocol: PP
      can_device: can1
      control_mode: position

    - name: hands_pitch_joint
      motor_id: 0x14
      protocol: PP
      can_device: can1
      control_mode: position

    # 底盘差速驱动（MT 协议）
    - name: left_ahead_wheel_joint
      motor_id: 0x141
      protocol: MT
      can_device: can1
      control_mode: velocity

    - name: right_ahead_wheel_joint
      motor_id: 0x142
      protocol: MT
      can_device: can1
      control_mode: velocity
```

#### 3.2 修改 ros_controllers.yaml

```yaml
controller_manager:
  update_rate: 100

joint_state_controller:
  type: joint_state_controller/JointStateController
  publish_rate: 100

pp_arm_controller:
  type: position_controllers/JointTrajectoryController
  joints:
    - big_course_joint
    - big_pitch_joint
    - small_pitch_joint
    - hands_pitch_joint
  constraints:
    goal_time: 5.0
    big_course_joint: {goal: 0.02}
    big_pitch_joint: {goal: 0.02}
    small_pitch_joint: {goal: 0.02}
    hands_pitch_joint: {goal: 0.02}

diff_drive_controller:
  type: diff_drive_controller/DiffDriveController
  left_wheel: left_ahead_wheel_joint
  right_wheel: right_ahead_wheel_joint
  publish_rate: 50
  wheel_separation: 0.438  # 实测：219mm * 2 = 438mm
  wheel_radius: 0.05       # 需要实测
```

#### 3.3 修改 motor.yaml

```yaml
hands_roll_joint: &motor_defaults
  switching_state: "Operation_Enable"
  pos_to_device: "rint(pos * 6619136.0 / (2.0 * pi))"
  pos_from_device: "obj * (2.0 * pi) / 6619136.0"
  vel_to_device: "rint(vel * 6619136.0 / (2.0 * pi))"
  vel_from_device: "obj * (2.0 * pi) / 6619136.0"

hands_course_joint:
  <<: *motor_defaults

left_ahead_arm_joint:
  <<: *motor_defaults

left_back_arm_joint:
  <<: *motor_defaults

right_ahead_arm_joint:
  <<: *motor_defaults

right_back_arm_joint:
  <<: *motor_defaults
```

#### 3.4 修改 controllers.yaml (canopen)

```yaml
joint_state_controller:
  type: joint_state_controller/JointStateController
  publish_rate: 100

canopen_arm_controller:
  type: position_controllers/JointTrajectoryController
  joints:
    - hands_roll_joint
    - hands_course_joint
  constraints:
    goal_time: 5.0
    hands_roll_joint: {goal: 0.02}
    hands_course_joint: {goal: 0.02}

flipper_controller:
  type: position_controllers/JointGroupPositionController
  joints:
    - left_ahead_arm_joint
    - left_back_arm_joint
    - right_ahead_arm_joint
    - right_back_arm_joint
```

#### 3.5 修改 splitter.yaml

```yaml
arm_controller_name: "arm_controller"
wait_server_timeout_sec: 5.0
poll_rate_hz: 50.0

backend_controllers:
  - name: "pp_arm_controller"
    action_ns: "/pp_arm_controller/follow_joint_trajectory"
    joints:
      - big_course_joint
      - big_pitch_joint
      - small_pitch_joint
      - hands_pitch_joint

  - name: "canopen_arm_controller"
    action_ns: "/canopen/canopen_arm_controller/follow_joint_trajectory"
    joints:
      - hands_roll_joint
      - hands_course_joint
```

### 步骤 4：更新 full_system.launch

```xml
<launch>
  <arg name="enable_moveit" default="false"/>

  <!-- Load URDF -->
  <param name="robot_description"
         textfile="$(find car_total)/urdf/car_total.urdf"/>

  <!-- Driver side: private CAN motors on can_B -->
  <include file="$(find can_driver)/launch/can_driver.launch"/>

  <!-- Driver side: CANopen motors on can_A -->
  <include file="$(find yiyou_canopen)/launch/yiyou_canopen.launch"/>

  <!-- Merge partial joint states to global /joint_states -->
  <node name="joint_state_publisher" pkg="joint_state_publisher" type="joint_state_publisher">
    <rosparam param="source_list">
      - can_driver/joint_states
      - canopen/joint_states
    </rosparam>
  </node>

  <node name="robot_state_publisher" pkg="robot_state_publisher" type="robot_state_publisher"/>

  <!-- 6-axis FollowJointTrajectory splitter -->
  <include file="$(find arm_traj_splitter)/launch/splitter.launch"/>

  <!-- Optional MoveIt entrypoint -->
  <include if="$(arg enable_moveit)" file="$(find robot_moveit_config)/launch/move_group.launch"/>
</launch>
```

---

## 底盘驱动的特殊处理

### 如果后轮是从动轮（推荐）

在 URDF 中添加 mimic 约束，使后轮跟随前轮：

```xml
<joint name="left_back_wheel_joint" type="continuous">
  <origin xyz="-0.156 0.219 0" rpy="0 0 0"/>
  <parent link="base_link"/>
  <child link="left_back_wheel"/>
  <axis xyz="0 -1 0"/>
  <mimic joint="left_ahead_wheel_joint" multiplier="1.0" offset="0"/>
</joint>

<joint name="right_back_wheel_joint" type="continuous">
  <origin xyz="-0.156 -0.219 0" rpy="0 0 0"/>
  <parent link="base_link"/>
  <child link="right_back_wheel"/>
  <axis xyz="0 -1 0"/>
  <mimic joint="right_ahead_wheel_joint" multiplier="1.0" offset="0"/>
</joint>
```

### 如果是 4 轮独立驱动

需要修改控制器配置，使用 4 个独立的速度控制器或自定义控制器。

---

## 下一步行动

1. **确认硬件配置**（30 分钟）
   - 底盘是差速还是 4 轮独立？
   - 后轮是主动还是从动？

2. **修改配置文件**（1 ���时）
   - 按照上述映射表更新所有配置

3. **测试编译**（10 分钟）
   ```bash
   cd ~/Robot24_catkin_ws
   catkin_make
   ```

4. **软件测试**（30 分钟）
   - 启动系统，检查关节名称
   - 验证 joint_states 包含正确的关节

5. **硬件测试**（按需）
   - 连接硬件后逐个测试

---

## 总结

- ✅ URDF 文件已找到且语法正确
- ⚠️ 关节命名与配置文件不匹配
- 📋 需要统一命名（推荐修改配置文件）
- ⏱️ 预计工作量：2-3 小时
