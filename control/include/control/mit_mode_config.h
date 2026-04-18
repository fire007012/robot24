#pragma once

#include <control/chassis_common_config.h>

namespace mit_mode_config
{
// 达妙控制模式寄存器的取值，1 表示 MIT 模式。
constexpr uint32_t kControlMode = 1;
// MIT 模式控制帧的基准 ID，实际帧 ID 直接等于电机 ID。
constexpr canid_t kControlFrameBase = 0x000;

// 官方文档里的 PMAX 寄存器地址。
constexpr uint8_t kPMaxRegister = 21;
// 官方文档里的 VMAX 寄存器地址。
constexpr uint8_t kVMaxRegister = 22;
// 官方文档里的 TMAX 寄存器地址。
constexpr uint8_t kTMaxRegister = 23;

// MIT 模式下位置映射范围的正向最大值，实际编码范围为 [-PMax, PMax]。
constexpr float kPositionMapMaxRad = 12.5f;
// MIT 模式下速度映射范围的正向最大值，实际编码范围为 [-VMax, VMax]。
constexpr float kVelocityMapMaxRadPerSec =
    static_cast<float>(chassis_common_config::kMaxMotorSpeedRadPerSec);
// MIT 模式下转矩映射范围的正向最大值，实际编码范围为 [-TMax, TMax]。
constexpr float kTorqueMapMax = 120.0f;

// 速度型 MIT 控制下固定的位置给定，kp 为 0 时该值不会参与位置控制。
constexpr double kHoldPositionRad = 0.0;
// 速度型 MIT 控制下的位置环 kp，按官方文档建议置 0。
constexpr double kVelocityControlKp = 0.0;
// 速度型 MIT 控制下的阻尼项 kd，需为非 0 正数。
constexpr double kVelocityControlKd = 1.0;
// 速度型 MIT 控制下的前馈转矩。
constexpr double kFeedforwardTorque = 0.0;

// MIT 控制帧里 kp 字段的最小值。
constexpr double kKpMin = 0.0;
// MIT 控制帧里 kp 字段的最大值。
constexpr double kKpMax = 500.0;
// MIT 控制帧里 kd 字段的最小值。
constexpr double kKdMin = 0.0;
// MIT 控制帧里 kd 字段的最大值。
constexpr double kKdMax = 5.0;
}  // namespace mit_mode_config
