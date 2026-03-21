# CANopen 主站通信异常排查与修复报告（完整修订版）

**项目：** Eyou_Canopen_Master（Lely Core + ROS）  
**日期：** 2026-03-21  
**总线：** `can0`  
**从站：** Node ID = 5  
**模式：** CSP

---

## 1. 问题概述

现场出现过以下典型问题：

1. `dcfgen` 失败（`invalid DCF`），导致配置生成链路中断。
2. 主站与从站身份信息不一致，boot 配置阶段不稳定。
3. 总线观测出现 TPDO 反馈异常（特别是 `0x185` 稀疏/缺失阶段）。
4. 控制器加载失败：`Could not find joint 'joint_2' in URDF model.`
5. 关闭阶段存在卡住风险（联调阶段曾见 `controller_spawner` 等未及时退出）。

---

## 2. 统一根因结论

### 2.1 EDS 格式兼容问题（生成阶段）

`YiyouServo_V1.4.dcfgen.eds` 包含两类导致 `dcfgen` 严格校验失败的内容：

- 非兼容段：`[DynamicChannels]`
- 越界字段：`[2023]`、`[2025]` 中 `UNSIGNED16` 的 `HighLimit=65536`

### 2.2 身份字段不一致（boot/配置阶段）

实测设备身份如下：

| 对象 | 含义 | 实测值 |
|---|---|---|
| `0x1000:00` | Device Type | `0x00220811` |
| `0x1018:01` | Vendor ID | `0x00000668` |
| `0x1018:02` | Product Code | `0x00221701` |

旧 EDS 中 `ProductNumber` 与 `1018sub2` 不一致，造成主站期望值与从站实值可能偏离。

### 2.3 URDF 与控制器关节配置不一致（运行阶段）

`controllers.yaml` 依赖 `joint_1~joint_6`，而旧 `urdf/robot.urdf` 仅含 `joint_1`，触发控制器初始化失败。

---

## 3. 已实施修复（已落地）

### 3.1 EDS 修复

文件：`config/YiyouServo_V1.4.dcfgen.eds`

1. 删除 `[DynamicChannels]`
2. 修复越界：
   - `[2023] HighLimit: 65536 -> 65535`
   - `[2025] HighLimit: 65536 -> 65535`
3. 统一产品码：
   - `[DeviceInfo] ProductNumber = 0x00221701`
   - `[1018sub2] DefaultValue = 0x00221701`

### 3.2 重新生成 DCF 产物

执行 `dcfgen master.yaml` 后更新：

- `config/master.dcf`
- `config/master.bin`
- `config/joint_1.bin`

### 3.3 URDF 修复

文件：`urdf/robot.urdf`

- 从单关节扩展为 `joint_1~joint_6`
- 与 `config/ros/controllers.yaml` 关节列表对齐

---

## 4. 验证结论

1. `dcfgen master.yaml` 可通过（无 fatal error）。
2. `master.dcf` 中设备身份检查值与实测一致：
   - `1F84:5 = 0x00220811`
   - `1F85:5 = 0x00000668`
   - `1F86:5 = 0x00221701`
3. URDF 与控制器关节命名一致，`joint_2` 缺失问题已消除。

---

## 5. 提交记录（精确到 commit）

### 5.1 当前生效提交

- `7605dbd`  
  `feat: add full Eyou_Canopen_Master package (code, urdf, docs, dcf)`

该提交包含：

- 代码：`src/`, `include/`, `srv/`, `test/`
- 配置：`config/`（含修复后的 EDS/DCF）
- URDF：`urdf/robot.urdf`
- 文档：`docs/`（含本报告与修复摘要）

---

## 6. 后续修复计划表（精确到 commit 级别）

> 说明：以下为下一阶段计划提交（尚未实施）。

| 计划ID | 目标 commit message | 主要修改文件 | 目标 | 验收标准 |
|---|---|---|---|---|
| C01 | `fix(canopen): publish controlword in rpdo cycle` | `src/axis_logic.cpp`, `test/test_axis_logic.cpp` | 在周期路径下发 `0x6040` 控制字 | 抓包 `0x205` 前2字节出现 `06 00 -> 0F 00` |
| C02 | `fix(canopen): align shutdown flow with disable request` | `src/canopen_master.cpp`, `src/axis_logic.cpp` | 关机前先统一 `RequestDisable`，避免周期/关机竞争 | Ctrl+C 退出时状态回落稳定，无长阻塞 |
| C03 | `feat(canopen): add boot identity mismatch diagnostics` | `src/axis_driver.cpp`, `docs/usage.md` | 强化 `OnBoot` 失败原因日志（身份不匹配可直观定位） | 启动失败日志可直接定位 `1000/1018` 失配 |
| C04 | `test(canopen): add startup and operational regression` | `test/test_startup_integration.cpp` 等 | 增加启动到可运行态回归测试 | CI 能稳定覆盖启动与使能路径 |
| C05 | `docs(canopen): finalize validation checklist for field` | `docs/usage.md`, `docs/2026-03-21_canopen_pdo_boot_diagnosis_report.md` | 固化现场验证清单与抓包判据 | 按文档可复现排障流程并判定结果 |

---

## 7. 备注

- 本报告为当前执行基线；此前 2026-03-19 阶段报告已归档至 `docs/archive/2026-03-21_deprecated/`。
- 若后续固件版本变化，需优先复核 `0x1000` 与 `0x1018` 三元组，再生成 DCF。
