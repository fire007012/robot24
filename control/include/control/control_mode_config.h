#pragma once

namespace control_mode_config
{
// 底盘控制模式枚举。
enum class ChassisControlMode
{
  kSpeedMode,
  kMitMode,
};

// 当前底盘控制模式，改这里即可切换速度模式和 MIT 模式。
constexpr ChassisControlMode kSelectedChassisControlMode = ChassisControlMode::kSpeedMode;
}  // namespace control_mode_config
