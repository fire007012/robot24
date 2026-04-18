#pragma once

#include <cstdint>

#include <linux/can.h>

namespace chassis_common_config
{
// CANable 使用的 SocketCAN 接口名。
constexpr char kCanInterface[] = "can0";

// 左轮电机 CAN ID。
constexpr uint32_t kLeftMotorId = 1;
// 右轮电机 CAN ID。
constexpr uint32_t kRightMotorId = 2;

// 左电机方向符号，用来和底盘前进方向对齐。
constexpr int kLeftMotorDirection = 1;
// 右电机方向符号，用来和底盘前进方向对齐。
constexpr int kRightMotorDirection = -1;

// 左右轮接地点中心之间的距离，单位米。
constexpr double kWheelTrackMeters = 0.36;
// 前后轴中心之间的距离，单位米。
constexpr double kWheelbaseMeters = 0.40;
// 车轮半径，单位米，用于把线速度换算成车轮角速度。
constexpr double kWheelRadiusMeters = 0.075;

// 速度换算用的圆周率。
constexpr double kPi = 3.14159265358979323846;
// 电机速度限幅，按说明书中的空载最大转速换算得到。
constexpr double kMaxMotorSpeedRadPerSec = 200.0 * 2.0 * kPi / 60.0;
// 超过这个时间没有收到 cmd_vel 就停车。
constexpr double kCmdTimeoutSec = 0.3;
// 主控制循环周期。
constexpr double kControlPeriodSec = 0.02;
// 启动阶段配置 CAN 命令之间的等待时间。
constexpr double kStartupDelaySec = 0.05;

// 反馈帧与寄存器写入回执使用的 Master ID，需与驱动参数保持一致。
constexpr canid_t kMasterId = 0x000;
// 等待寄存器写入回执的超时时间，单位毫秒。
constexpr int kRegisterAckTimeoutMs = 100;
// 等待使能/失能状态反馈的超时时间，单位毫秒。
constexpr int kStatusFeedbackTimeoutMs = 100;

// 官方文档里的寄存器写入帧 ID。
constexpr canid_t kRegisterFrameId = 0x7FF;
// 官方文档里的寄存器写入命令字。
constexpr uint8_t kWriteRegisterCmd = 0x55;
// 官方文档里的控制模式寄存器地址。
constexpr uint8_t kControlModeRegister = 10;

// 电机使能命令字。
constexpr uint8_t kEnableCommand = 0xFC;
// 电机失能命令字。
constexpr uint8_t kDisableCommand = 0xFD;

// 反馈帧中表示使能状态的状态码。
constexpr uint8_t kEnabledStateCode = 0x1;
// 反馈帧中表示失能状态的状态码。
constexpr uint8_t kDisabledStateCode = 0x0;
}  // namespace chassis_common_config
