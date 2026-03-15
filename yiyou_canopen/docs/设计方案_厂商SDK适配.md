# 设计方案：基于厂商 SDK 的 CAN 驱动适配

## 目标
- 保持 `yiyou_canopen` 包作为启动入口与配置入口（launch/YAML/URDF）。
- ROS 侧接口改为 `can_driver` 风格（RobotHW + ros_control + service/topic）。
- 底层不再依赖 `ros_canopen`，直接调用厂商 `eu_harmonic` SDK。

## 总体架构（ASCII）

```
┌────────────────────────────────────────────────────────────────────┐
│                         ROS / 上层应用                              │
│  MoveIt / ControllerManager / 用户节点 / 任务逻辑                    │
└────────────────────────────────────────────────────────────────────┘
                               │
                               ▼
┌────────────────────────────────────────────────────────────────────┐
│                         can_driver (ROS接口层)                      │
│  - CanDriverHW (RobotHW)                                            │
│  - joint_state / controller / services (init/shutdown/recover)      │
│  - 参数读取：/can_driver_node/joints 等                              │
└────────────────────────────────────────────────────────────────────┘
                               │
                               ▼
┌────────────────────────────────────────────────────────────────────┐
│                 新增：SDKDeviceManager / SDKTransport               │
│  - 负责 SDK 初始化/释放（harmonic_initDLL/free）                    │
│  - 设备索引 devIndex 管理                                            │
│  - 线程安全：设备互斥锁                                              │
└────────────────────────────────────────────────────────────────────┘
                               │
                               ▼
┌────────────────────────────────────────────────────────────────────┐
│                        EyouCan (SDK 适配协议层)                     │
│  - setMode / setPosition / setVelocity / Enable / Disable / Stop     │
│  - getPosition / getVelocity / getCurrent / isEnabled / hasFault     │
│  - 状态刷新线程（SDK 读回）                                          │
└────────────────────────────────────────────────────────────────────┘
                               │
                               ▼
┌────────────────────────────────────────────────────────────────────┐
│                    厂商 SDK (eu_harmonic) + 硬件                     │
│  - libeu_harmonic.so                                                │
│  - USB2CAN / Canable / ZCAN 等                                       │
└────────────────────────────────────────────────────────────────────┘
```

## 配置流向（保持 yiyou_canopen 入口）

```
yiyou_canopen/launch/*.launch
  └── 加载 yiyou_canopen/config/*.yaml
        └── 转为 can_driver 的 joints 参数结构
              └── can_driver_node 启动
```

## 关键改造点（高层）
- 删除 `ros_canopen` 依赖与构建。
- `can_driver` 中 SocketCAN 传输替换为 SDK 传输。
- `yiyou_canopen` 仅保留入口/配置壳子，提供到 `can_driver` 的映射参数。

## 兼容性与约束
- SDK 多设备共享主站：全局 `motor_id` 必须唯一。
- 运行平台：x86_64（对应 SDK 版本）。
- 现有 `can_driver` 话题/服务接口不变，以便上层最小改动。

