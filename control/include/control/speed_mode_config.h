#pragma once

#include <control/chassis_common_config.h>

namespace speed_mode_config
{
// 达妙控制模式寄存器的取值，3 表示速度模式。
constexpr uint32_t kControlMode = 3;
// 官方文档里的速度模式控制帧基准 ID。
constexpr canid_t kControlFrameBase = 0x200;
}  // namespace speed_mode_config
