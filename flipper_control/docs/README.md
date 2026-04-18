# flipper_control 文档

本目录用于记录 `flipper_control` 的接口约定、预置姿态定义和安全约束说明。

当前实现已经包含：

- `flipper_manager_node`
- `SetControlProfile` / `SetLinkageMode`
- `FlipperControlState`
- `csp_position` / `csp_jog` / `csv_velocity`
- 联动模式展开、命令超时与 `q_ref` 参考生成

正式设计说明见顶层文档：

- `docs/19_控制包拆分与当前结构说明_2026-04-14.md`
- `docs/21_摆臂控制器模式与冷切换设计_2026-04-15.md`
- `docs/22_摆臂控制落地与提交路线图_2026-04-15.md`
