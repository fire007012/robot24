# base_cmd_node

`base_cmd_node` 是 `mobility_control` 当前唯一的正式运行节点，用来在 `wheel_controller` 前做底盘命令安全整形。

## 默认链路

```text
/cmd_vel -> base_cmd_node -> /wheel_controller/cmd_vel
```

## 参数

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `input_topic` | `/cmd_vel` | 上游速度输入 |
| `output_topic` | `/wheel_controller/cmd_vel` | 发送给控制器的话题 |
| `max_linear_x` | `0.8` | `linear.x` 限幅 |
| `max_angular_z` | `1.5` | `angular.z` 限幅 |
| `timeout_sec` | `0.3` | 超时后自动发零速度 |

## 启动

```bash
roslaunch mobility_control base_cmd.launch
```

```bash
roslaunch mobility_control base_cmd.launch \
  input_topic:=/cmd_vel \
  output_topic:=/wheel_controller/cmd_vel \
  max_linear_x:=0.6 \
  max_angular_z:=1.0 \
  timeout_sec:=0.2
```

## 测试

```bash
# 正常速度
rostopic pub -1 /cmd_vel geometry_msgs/Twist \
  '{linear: {x: 0.3}, angular: {z: 0.2}}'

# 超限输入，观察输出是否被截断
rostopic pub -1 /cmd_vel geometry_msgs/Twist \
  '{linear: {x: 1.5}, angular: {z: 2.0}}'

rostopic echo /wheel_controller/cmd_vel
```

## 边界

- 不直接写 CAN
- 不做差速逆解
- 不承担机械臂、摆臂或总控逻辑
