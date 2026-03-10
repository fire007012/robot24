# CANopen 位置/速度换算参数说明

本文说明 `yiyou_canopen/config/motor.yaml` 中换算常数 `6619136.0` 的来源、单位和重算方法。

## 1. 参数位置

- 文件：`yiyou_canopen/config/motor.yaml`
- 字段：
  - `pos_to_device`
  - `pos_from_device`
  - `vel_to_device`
  - `vel_from_device`

## 2. 物理含义与单位

- 常数 `6619136.0` 的单位是 `counts/rev`（编码器计数每关节一圈）
- 含义：关节轴转一圈，对应驱动器对象字典中的计数增量

## 3. 计算公式

设：

- `N_enc` = 电机轴每圈编码器计数（counts/motor_rev）
- `R` = 减速比（motor_rev/joint_rev）

则：

```text
counts_per_joint_rev = N_enc * R
```

当前项目使用：

```text
N_enc = 131072 (= 2^17)
R = 50.5
counts_per_joint_rev = 131072 * 50.5 = 6619136
```

## 4. 在公式中的使用

`motor.yaml` 当前使用弧度制：

```text
device_pos_counts = rint(joint_pos_rad * counts_per_joint_rev / (2*pi))
joint_pos_rad     = device_pos_counts * (2*pi) / counts_per_joint_rev
```

速度换算同理。

## 5. 变更流程（换电机/编码器/减速器时）

1. 根据新硬件规格确认 `N_enc` 和 `R`
2. 计算 `counts_per_joint_rev = N_enc * R`
3. 将 `motor.yaml` 中 `6619136.0` 同步替换为新值
4. 重新编译并做小范围联调（低速点动 + 位置闭环）

## 6. 示例

若新规格为 `N_enc=65536`、`R=100`：

```text
counts_per_joint_rev = 65536 * 100 = 6553600
```

则把 `motor.yaml` 中 `6619136.0` 改为 `6553600.0`。
