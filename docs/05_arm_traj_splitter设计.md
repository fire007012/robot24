# arm_traj_splitter 代理节点设计

## 1. 背景与目的

MoveIt 需要向一个 FollowJointTrajectory action server 发送 6 轴轨迹。
但 6 个臂关节分属两个驱动节点，各自有独立的 `controller_manager`。

代理节点的作用：
- 对 MoveIt 暴露一个 6 轴的 FollowJointTrajectory action server
- 收到轨迹后按关节名拆成 4 + 2 两组
- 同时转发给两个后端的 JointTrajectoryController
- 等两边完成后合并结果返回给 MoveIt

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

## 3. 参数配置

```yaml
# splitter.yaml
# 对 MoveIt 暴露的 action server 名称
arm_controller_name: "arm_controller"

# 后端列表（每个后端对应一个 JointTrajectoryController 的 action）
backend_controllers:
  - name: "pp_arm_controller"
    action_ns: "/can_driver_node/pp_arm_controller/follow_joint_trajectory"
    joints:
      - arm_joint_1
      - arm_joint_2
      - arm_joint_3
      - arm_joint_4
  - name: "canopen_arm_controller"
    action_ns: "/canopen/canopen_arm_controller/follow_joint_trajectory"
    joints:
      - arm_joint_5
      - arm_joint_6
```

### 字段说明

| 字段 | 含义 |
|------|------|
| `arm_controller_name` | splitter 对外暴露的 action server 名称前缀，MoveIt 连接 `/<arm_controller_name>/follow_joint_trajectory` |
| `backend_controllers[].name` | 后端 controller 的标识（仅用于日志和调试） |
| `backend_controllers[].action_ns` | 后端 JointTrajectoryController 的完整 action 路径，splitter 的 ActionClient 连接该地址 |
| `backend_controllers[].joints` | 该后端负责的关节列表，用于拆分轨迹时的索引映射 |

> `action_ns` 是必填字段。splitter 启动时会对每个后端创建 ActionClient 并等待连接。
> 如果路径错误，启动日志会报 "Waiting for action server..." 超时。

## 4. 类设计

```cpp
class SplitterNode
{
public:
    SplitterNode(ros::NodeHandle& nh, ros::NodeHandle& pnh);

private:
    actionlib::SimpleActionServer<
        control_msgs::FollowJointTrajectoryAction> server_;

    struct Backend {
        std::string name;
        std::set<std::string> joints;
        std::unique_ptr<actionlib::SimpleActionClient<
            control_msgs::FollowJointTrajectoryAction>> client;
    };
    std::vector<Backend> backends_;

    void onGoal(const control_msgs::FollowJointTrajectoryGoalConstPtr& goal);

    bool splitTrajectory(
        const trajectory_msgs::JointTrajectory& full,
        std::vector<trajectory_msgs::JointTrajectory>& parts);

    void splitTolerances(
        const control_msgs::FollowJointTrajectoryGoal& fullGoal,
        std::vector<control_msgs::FollowJointTrajectoryGoal>& partGoals);
};
```

## 5. `onGoal` 核心逻辑

```cpp
void SplitterNode::onGoal(
    const control_msgs::FollowJointTrajectoryGoalConstPtr& goal)
{
    std::vector<control_msgs::FollowJointTrajectoryGoal> partGoals(backends_.size());
    if (!splitTrajectory(goal->trajectory, /* ... */)) {
        server_.setAborted(/* error: joint name mismatch */);
        return;
    }
    splitTolerances(*goal, partGoals);

    for (size_t i = 0; i < backends_.size(); ++i) {
        backends_[i].client->sendGoal(partGoals[i]);
    }

    ros::Rate poll(50);
    while (ros::ok()) {
        if (server_.isPreemptRequested()) {
            for (auto& b : backends_) {
                b.client->cancelGoal();
            }
            server_.setPreempted();
            return;
        }

        bool allDone = true;
        bool anyFailed = false;
        for (auto& b : backends_) {
            auto state = b.client->getState();
            if (!state.isDone()) {
                allDone = false;
            } else if (state != actionlib::SimpleClientGoalState::SUCCEEDED) {
                anyFailed = true;
            }
        }

        if (anyFailed) {
            for (auto& b : backends_) {
                if (!b.client->getState().isDone()) {
                    b.client->cancelGoal();
                }
            }
            server_.setAborted();
            return;
        }

        if (allDone) {
            server_.setSucceeded();
            return;
        }

        poll.sleep();
    }
}
```

## 6. `splitTrajectory` 逻辑

对于输入轨迹中的每个 `JointTrajectoryPoint`：
- 按 `joint_names` 建立索引映射，确定哪些列属于哪个后端
- 将 `positions / velocities / accelerations / effort` 按索引分组
- `time_from_start` 保持不变（保证时间同步）

```cpp
bool SplitterNode::splitTrajectory(
    const trajectory_msgs::JointTrajectory& full,
    std::vector<trajectory_msgs::JointTrajectory>& parts)
{
    parts.resize(backends_.size());
    std::vector<std::vector<size_t>> indexMaps(backends_.size());

    for (size_t j = 0; j < full.joint_names.size(); ++j) {
        bool found = false;
        for (size_t b = 0; b < backends_.size(); ++b) {
            if (backends_[b].joints.count(full.joint_names[j])) {
                parts[b].joint_names.push_back(full.joint_names[j]);
                indexMaps[b].push_back(j);
                found = true;
                break;
            }
        }
        if (!found) {
            ROS_ERROR("Joint '%s' not assigned to any backend.", full.joint_names[j].c_str());
            return false;
        }
    }

    for (const auto& pt : full.points) {
        for (size_t b = 0; b < backends_.size(); ++b) {
            trajectory_msgs::JointTrajectoryPoint newPt;
            newPt.time_from_start = pt.time_from_start;

            for (size_t idx : indexMaps[b]) {
                if (!pt.positions.empty()) newPt.positions.push_back(pt.positions[idx]);
                if (!pt.velocities.empty()) newPt.velocities.push_back(pt.velocities[idx]);
                if (!pt.accelerations.empty()) newPt.accelerations.push_back(pt.accelerations[idx]);
                if (!pt.effort.empty()) newPt.effort.push_back(pt.effort[idx]);
            }
            parts[b].points.push_back(newPt);
        }
    }

    for (auto& p : parts) {
        p.header = full.header;
    }
    return true;
}
```

## 7. 预估代码量

| 文件 | 行数 |
|------|------|
| `SplitterNode.h` | ~50 |
| `SplitterNode.cpp` | ~250 |
| `main.cpp` | ~20 |
| `CMakeLists.txt` | ~30 |
| `package.xml` | ~20 |
| `splitter.yaml` | ~10 |
| `splitter.launch` | ~15 |
| 合计 | ~400 |
