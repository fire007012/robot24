# arm_traj_splitter 节点实现详解

## 1. 设计目标

`arm_traj_splitter` 用于解决以下约束冲突：

1. MoveIt 只向一个 `FollowJointTrajectory` Action Server 发 6 轴轨迹。
2. 6 轴机械臂关节分布在两个独立后端（`can_driver` 与 `canopen`），各自有独立 `controller_manager`。
3. 不改现有驱动核心代码。

节点职责：

1. 对外暴露统一 Action：`/<arm_controller_name>/follow_joint_trajectory`
2. 按关节名拆分轨迹
3. 并发转发给多个后端 Action
4. 汇总执行结果并回传给 MoveIt

---

## 2. 包结构

```text
arm_traj_splitter/
├── include/arm_traj_splitter/
│   └── SplitterNode.h
├── src/
│   ├── SplitterNode.cpp
│   └── main.cpp
├── config/
│   └── splitter.yaml
├── launch/
│   └── splitter.launch
├── CMakeLists.txt
└── package.xml
```

---

## 3. 对外接口

### 3.1 Action Server（上游）

- 名称：`/<arm_controller_name>/follow_joint_trajectory`
- 类型：`control_msgs/FollowJointTrajectoryAction`
- 默认：`/arm_controller/follow_joint_trajectory`

### 3.2 Action Client（下游）

由参数 `backend_controllers` 决定，可扩展为 N 个后端。当前默认 2 个：

1. `/pp_arm_controller/follow_joint_trajectory`（`arm_joint_1~4`）
2. `/canopen/canopen_arm_controller/follow_joint_trajectory`（`arm_joint_5~6`）

---

## 4. 参数配置（splitter.yaml）

```yaml
arm_controller_name: "arm_controller"
wait_server_timeout_sec: 5.0
poll_rate_hz: 50.0

backend_controllers:
  - name: "pp_arm_controller"
    action_ns: "/pp_arm_controller/follow_joint_trajectory"
    joints: [arm_joint_1, arm_joint_2, arm_joint_3, arm_joint_4]

  - name: "canopen_arm_controller"
    action_ns: "/canopen/canopen_arm_controller/follow_joint_trajectory"
    joints: [arm_joint_5, arm_joint_6]
```

字段说明：

1. `arm_controller_name`：对 MoveIt 暴露的控制器前缀。
2. `wait_server_timeout_sec`：启动阶段等待后端 Action Server 的超时。
3. `poll_rate_hz`：执行期间轮询后端状态频率。
4. `backend_controllers[]`：后端列表。
5. `backend_controllers[].joints`：后端负责的关节集合，是拆分依据。

---

## 5. 核心流程

### 5.1 启动阶段

1. 从参数读取 `backend_controllers`。
2. 做配置合法性检查：
   - 列表非空
   - 每个 backend 有 `name/action_ns/joints`
   - backend 内关节不重复
   - 全局关节不重复归属
3. 为每个 backend 创建 Action Client。
4. 等待后端 Action Server 可连接（超时仅告警，节点仍可启动）。
5. 启动自身 Action Server。

### 5.2 接收 Goal

收到完整 6 轴轨迹后：

1. 调用 `splitGoal()`：
   - `splitTrajectory()` 拆分 `trajectory`（含 points）
   - `splitTolerances()` 拆分 `path_tolerance/goal_tolerance`
2. 若拆分失败，直接 `setAborted(INVALID_GOAL)`。
3. 拆分成功后并发 `sendGoal()` 到所有后端。

### 5.3 执行监控与收敛

循环轮询每个后端状态：

1. 若上游 preempt：取消全部后端并 `setPreempted()`。
2. 若任一后端失败：取消其他未完成后端并 `setAborted()`。
3. 若全部成功：`setSucceeded()`。
4. ROS 退出：`setAborted()`。

---

## 6. 拆分规则细节

### 6.1 轨迹拆分 `splitTrajectory`

输入：`trajectory_msgs/JointTrajectory full`

输出：`parts[i]` 对应第 `i` 个 backend

规则：

1. 以 `full.joint_names` 为列索引基准。
2. 每个 joint 必须能在 `joint_to_backend_` 中找到归属。
3. 任意未知 joint / 重复 joint 直接失败。
4. 每个 backend 必须至少拿到一个 joint，否则失败。
5. 对每个点：
   - `time_from_start` 原样复制
   - `positions/velocities/accelerations/effort` 按索引抽取
   - 字段若非空，长度必须等于 full joint 数

### 6.2 容差拆分 `splitTolerances`

- 从 full goal 的 `path_tolerance` / `goal_tolerance` 中，按 backend joint 名集合过滤。
- `goal_time_tolerance` 原样复制给所有 backend goal。

---

## 7. 错误处理策略

### 7.1 配置错误（启动期）

直接报错并抛异常退出：

1. `backend_controllers` 缺失或为空
2. backend 字段缺失
3. 关节重复归属

### 7.2 请求错误（运行期）

返回 `INVALID_GOAL`：

1. 轨迹 joint 为空
2. 轨迹中有重复 joint
3. joint 未在配置中分配
4. 轨迹点字段长度不匹配
5. 某 backend 未分配到任何轨迹关节

### 7.3 后端执行失败

策略：失败即整体失败

1. 检测到任意后端 `state != SUCCEEDED`
2. 取消其余未结束后端
3. 返回 `PATH_TOLERANCE_VIOLATED` + 失败 backend 名称

---

## 8. 与系统其余模块的契约

1. MoveIt 控制器配置需指向 `arm_controller/follow_joint_trajectory`。
2. `can_driver` 必须提供 `pp_arm_controller` Action。
3. `yiyou_canopen` 必须提供 `canopen_arm_controller` Action。
4. 三方对关节命名必须完全一致。

---

## 9. 构建与运行

### 9.1 编译

```bash
cd ~/Robot24_catkin_ws
catkin_make --pkg arm_traj_splitter
```

### 9.2 启动

```bash
roslaunch arm_traj_splitter splitter.launch
```

### 9.3 验证检查

```bash
# 1) 上游 action 是否存在
rostopic list | grep /arm_controller/follow_joint_trajectory

# 2) 后端 action 是否存在
rostopic list | grep /pp_arm_controller/follow_joint_trajectory
rostopic list | grep /canopen/canopen_arm_controller/follow_joint_trajectory

# 3) 发送测试 goal
rosrun actionlib_tools axclient.py /arm_controller/follow_joint_trajectory
```

---

## 10. 当前实现边界

1. 当前是“任一后端失败则整体失败”策略，未做部分成功合并。
2. 启动期后端连接超时只告警不阻断，可按项目需要改为强阻断。
3. 默认假设上游轨迹关节集合与后端配置一致，不做自动补齐缺失关节。
4. 默认后端 action 名与当前控制器配置一致，如命名空间变更需同步调整 `splitter.yaml`。
