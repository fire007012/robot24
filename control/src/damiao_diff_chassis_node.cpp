/*
 * Archived legacy Damiao standalone chassis executor.
 *
 * Historical runtime path:
 *   /cmd_vel -> chassis diff inverse kinematics -> direct Damiao CAN writes
 *
 * This source is preserved as vendor/reference code for traceability and
 * rollback comparison, but it is no longer part of the official runtime.
 *
 * Official runtime path:
 *   /cmd_vel
 *     -> mobility_control/base_cmd_node
 *       -> wheel_controller (diff_drive_controller)
 *         -> controller_manager / HybridRobotHW
 *           -> can_driver::DamiaoCan
 *
 * The control package intentionally does not build this file as an executable.
 */

#include <cerrno>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <poll.h>
#include <string>

#include <control/chassis_common_config.h>
#include <control/control_mode_config.h>
#include <control/mit_mode_config.h>
#include <control/speed_mode_config.h>
#include <geometry_msgs/Twist.h>
#include <ros/ros.h>

#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

namespace
{
namespace common_cfg = chassis_common_config;
namespace mode_cfg = control_mode_config;

struct CommonControlConfig
{
  const char* can_interface = "";
  canid_t master_id = 0;
  uint32_t left_motor_id = 0;
  uint32_t right_motor_id = 0;
  uint32_t control_mode = 0;
  int left_motor_direction = 1;
  int right_motor_direction = 1;
  double wheel_track_meters = 0.0;
  double wheelbase_meters = 0.0;
  double wheel_radius_meters = 0.0;
  double max_motor_speed_rad_per_sec = 0.0;
  double cmd_timeout_sec = 0.0;
  double control_period_sec = 0.0;
  double startup_delay_sec = 0.0;
  int register_ack_timeout_ms = 0;
  int status_feedback_timeout_ms = 0;
  canid_t register_frame_id = 0;
  canid_t control_frame_base = 0;
  uint8_t write_register_cmd = 0;
  uint8_t control_mode_register = 0;
  uint8_t enable_command = 0;
  uint8_t disable_command = 0;
};

struct MitControlConfig
{
  uint8_t p_max_register = 0;
  uint8_t v_max_register = 0;
  uint8_t t_max_register = 0;
  float position_map_max_rad = 0.0f;
  float velocity_map_max_rad_per_sec = 0.0f;
  float torque_map_max = 0.0f;
  double hold_position_rad = 0.0;
  double velocity_control_kp = 0.0;
  double velocity_control_kd = 0.0;
  double feedforward_torque = 0.0;
  double kp_min = 0.0;
  double kp_max = 0.0;
  double kd_min = 0.0;
  double kd_max = 0.0;
};

struct WheelSpeeds
{
  double left_rad_s = 0.0;
  double right_rad_s = 0.0;
};

struct MitCommand
{
  double position_rad = 0.0;
  double velocity_rad_s = 0.0;
  double kp = 0.0;
  double kd = 0.0;
  double torque_ff = 0.0;
};

struct MitPackedCommand
{
  uint16_t position = 0;
  uint16_t velocity = 0;
  uint16_t kp = 0;
  uint16_t kd = 0;
  uint16_t torque = 0;
};

// 判断当前是否选择了 MIT 模式。
bool isMitModeSelected()
{
  return mode_cfg::kSelectedChassisControlMode == mode_cfg::ChassisControlMode::kMitMode;
}

// 返回当前底盘控制模式的中文名字。
const char* getSelectedControlModeName()
{
  return isMitModeSelected() ? "MIT模式" : "速度模式";
}

// 根据主文件当前选择的模式，生成通用底盘配置。
CommonControlConfig makeCommonControlConfig()
{
  CommonControlConfig config;

  if (isMitModeSelected())
  {
    config.can_interface = common_cfg::kCanInterface;
    config.master_id = common_cfg::kMasterId;
    config.left_motor_id = common_cfg::kLeftMotorId;
    config.right_motor_id = common_cfg::kRightMotorId;
    config.control_mode = mit_mode_config::kControlMode;
    config.left_motor_direction = common_cfg::kLeftMotorDirection;
    config.right_motor_direction = common_cfg::kRightMotorDirection;
    config.wheel_track_meters = common_cfg::kWheelTrackMeters;
    config.wheelbase_meters = common_cfg::kWheelbaseMeters;
    config.wheel_radius_meters = common_cfg::kWheelRadiusMeters;
    config.max_motor_speed_rad_per_sec = common_cfg::kMaxMotorSpeedRadPerSec;
    config.cmd_timeout_sec = common_cfg::kCmdTimeoutSec;
    config.control_period_sec = common_cfg::kControlPeriodSec;
    config.startup_delay_sec = common_cfg::kStartupDelaySec;
    config.register_ack_timeout_ms = common_cfg::kRegisterAckTimeoutMs;
    config.status_feedback_timeout_ms = common_cfg::kStatusFeedbackTimeoutMs;
    config.register_frame_id = common_cfg::kRegisterFrameId;
    config.control_frame_base = mit_mode_config::kControlFrameBase;
    config.write_register_cmd = common_cfg::kWriteRegisterCmd;
    config.control_mode_register = common_cfg::kControlModeRegister;
    config.enable_command = common_cfg::kEnableCommand;
    config.disable_command = common_cfg::kDisableCommand;
    return config;
  }

  config.can_interface = common_cfg::kCanInterface;
  config.master_id = common_cfg::kMasterId;
  config.left_motor_id = common_cfg::kLeftMotorId;
  config.right_motor_id = common_cfg::kRightMotorId;
  config.control_mode = speed_mode_config::kControlMode;
  config.left_motor_direction = common_cfg::kLeftMotorDirection;
  config.right_motor_direction = common_cfg::kRightMotorDirection;
  config.wheel_track_meters = common_cfg::kWheelTrackMeters;
  config.wheelbase_meters = common_cfg::kWheelbaseMeters;
  config.wheel_radius_meters = common_cfg::kWheelRadiusMeters;
  config.max_motor_speed_rad_per_sec = common_cfg::kMaxMotorSpeedRadPerSec;
  config.cmd_timeout_sec = common_cfg::kCmdTimeoutSec;
  config.control_period_sec = common_cfg::kControlPeriodSec;
  config.startup_delay_sec = common_cfg::kStartupDelaySec;
  config.register_ack_timeout_ms = common_cfg::kRegisterAckTimeoutMs;
  config.status_feedback_timeout_ms = common_cfg::kStatusFeedbackTimeoutMs;
  config.register_frame_id = common_cfg::kRegisterFrameId;
  config.control_frame_base = speed_mode_config::kControlFrameBase;
  config.write_register_cmd = common_cfg::kWriteRegisterCmd;
  config.control_mode_register = common_cfg::kControlModeRegister;
  config.enable_command = common_cfg::kEnableCommand;
  config.disable_command = common_cfg::kDisableCommand;
  return config;
}

// 生成 MIT 模式专用配置。
MitControlConfig makeMitControlConfig()
{
  MitControlConfig config;
  config.p_max_register = mit_mode_config::kPMaxRegister;
  config.v_max_register = mit_mode_config::kVMaxRegister;
  config.t_max_register = mit_mode_config::kTMaxRegister;
  config.position_map_max_rad = mit_mode_config::kPositionMapMaxRad;
  config.velocity_map_max_rad_per_sec = mit_mode_config::kVelocityMapMaxRadPerSec;
  config.torque_map_max = mit_mode_config::kTorqueMapMax;
  config.hold_position_rad = mit_mode_config::kHoldPositionRad;
  config.velocity_control_kp = mit_mode_config::kVelocityControlKp;
  config.velocity_control_kd = mit_mode_config::kVelocityControlKd;
  config.feedforward_torque = mit_mode_config::kFeedforwardTorque;
  config.kp_min = mit_mode_config::kKpMin;
  config.kp_max = mit_mode_config::kKpMax;
  config.kd_min = mit_mode_config::kKdMin;
  config.kd_max = mit_mode_config::kKdMax;
  return config;
}

// 将输入值限制在 [-limit, limit] 范围内。
double clampSigned(const double value, const double limit)
{
  if (value > limit)
  {
    return limit;
  }
  if (value < -limit)
  {
    return -limit;
  }
  return value;
}

// 将输入值限制在 [min_value, max_value] 范围内。
double clampToRange(const double value, const double min_value, const double max_value)
{
  if (value < min_value)
  {
    return min_value;
  }
  if (value > max_value)
  {
    return max_value;
  }
  return value;
}

// 按线性映射关系把浮点数编码成无符号定点数。
uint32_t encodeMappedUnsigned(const double value, const double min_value, const double max_value,
                              const uint8_t bits)
{
  const double clamped_value = clampToRange(value, min_value, max_value);
  const double span = max_value - min_value;
  const uint32_t max_raw = (1u << bits) - 1u;
  return static_cast<uint32_t>((clamped_value - min_value) * max_raw / span);
}

// 根据当前模式和电机 ID 计算控制帧 ID。
canid_t makeControlFrameId(const CommonControlConfig& common_config, const uint32_t motor_id)
{
  return common_config.control_frame_base + motor_id;
}

// 根据 cmd_vel 计算差速底盘左右轮的目标角速度。
WheelSpeeds inverseKinematics(const geometry_msgs::Twist& cmd_vel,
                              const CommonControlConfig& common_config)
{
  WheelSpeeds speeds;

  const double left_linear_mps =
      cmd_vel.linear.x - cmd_vel.angular.z * (common_config.wheel_track_meters * 0.5);
  const double right_linear_mps =
      cmd_vel.linear.x + cmd_vel.angular.z * (common_config.wheel_track_meters * 0.5);

  speeds.left_rad_s = left_linear_mps / common_config.wheel_radius_meters;
  speeds.right_rad_s = right_linear_mps / common_config.wheel_radius_meters;
  return speeds;
}

// 把 MIT 模式的浮点控制量按官方文档编码成 16/12 位定点数。
MitPackedCommand packMitCommand(const MitCommand& command, const MitControlConfig& mit_config)
{
  MitPackedCommand packed_command;
  packed_command.position = static_cast<uint16_t>(encodeMappedUnsigned(
      command.position_rad,
      -static_cast<double>(mit_config.position_map_max_rad),
      static_cast<double>(mit_config.position_map_max_rad), 16));
  packed_command.velocity = static_cast<uint16_t>(encodeMappedUnsigned(
      command.velocity_rad_s,
      -static_cast<double>(mit_config.velocity_map_max_rad_per_sec),
      static_cast<double>(mit_config.velocity_map_max_rad_per_sec), 12));
  packed_command.kp = static_cast<uint16_t>(encodeMappedUnsigned(
      command.kp, mit_config.kp_min, mit_config.kp_max, 12));
  packed_command.kd = static_cast<uint16_t>(encodeMappedUnsigned(
      command.kd, mit_config.kd_min, mit_config.kd_max, 12));
  packed_command.torque = static_cast<uint16_t>(encodeMappedUnsigned(
      command.torque_ff,
      -static_cast<double>(mit_config.torque_map_max),
      static_cast<double>(mit_config.torque_map_max), 12));
  return packed_command;
}

class SocketCanBus
{
public:
  // 析构时关闭 CAN socket，释放系统资源。
  ~SocketCanBus()
  {
    close();
  }

  // 打开并绑定指定的 SocketCAN 接口。
  bool open(const std::string& interface_name)
  {
    close();

    socket_fd_ = ::socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (socket_fd_ < 0)
    {
      ROS_ERROR_STREAM("创建 CAN socket 失败: " << std::strerror(errno));
      return false;
    }

    struct ifreq ifr;
    std::memset(&ifr, 0, sizeof(ifr));
    std::strncpy(ifr.ifr_name, interface_name.c_str(), IFNAMSIZ - 1);

    if (::ioctl(socket_fd_, SIOCGIFINDEX, &ifr) < 0)
    {
      ROS_ERROR_STREAM("查询 CAN 接口失败(" << interface_name
                       << "): " << std::strerror(errno));
      close();
      return false;
    }

    struct sockaddr_can addr;
    std::memset(&addr, 0, sizeof(addr));
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (::bind(socket_fd_, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr)) < 0)
    {
      ROS_ERROR_STREAM("绑定 CAN 接口失败(" << interface_name
                       << "): " << std::strerror(errno));
      close();
      return false;
    }

    return true;
  }

  // 关闭当前已经打开的 CAN socket。
  void close()
  {
    if (socket_fd_ >= 0)
    {
      ::close(socket_fd_);
      socket_fd_ = -1;
    }
  }

  // 返回 CAN socket 当前是否已经成功打开。
  bool isOpen() const
  {
    return socket_fd_ >= 0;
  }

  // 清空 socket 中尚未读取的 CAN 报文，避免使用陈旧反馈做状态判断。
  void clearPendingFrames()
  {
    struct can_frame frame;
    while (readFrame(frame, 0))
    {
    }
  }

  // 通过寄存器写入帧向指定电机写入一个 32 位整型寄存器值。
  bool writeRegisterU32(const canid_t register_frame_id, const uint8_t write_register_cmd,
                        const uint32_t motor_id, const uint8_t register_id, const uint32_t value)
  {
    uint8_t payload[8];
    payload[0] = static_cast<uint8_t>(motor_id & 0xFF);
    payload[1] = static_cast<uint8_t>((motor_id >> 8) & 0xFF);
    payload[2] = write_register_cmd;
    payload[3] = register_id;
    payload[4] = static_cast<uint8_t>(value & 0xFF);
    payload[5] = static_cast<uint8_t>((value >> 8) & 0xFF);
    payload[6] = static_cast<uint8_t>((value >> 16) & 0xFF);
    payload[7] = static_cast<uint8_t>((value >> 24) & 0xFF);
    return writeFrame(register_frame_id, payload, 8);
  }

  // 通过寄存器写入帧向指定电机写入一个 float 寄存器值。
  bool writeRegisterFloat(const canid_t register_frame_id, const uint8_t write_register_cmd,
                          const uint32_t motor_id, const uint8_t register_id, const float value)
  {
    uint32_t raw_value = 0;
    std::memcpy(&raw_value, &value, sizeof(raw_value));
    return writeRegisterU32(register_frame_id, write_register_cmd, motor_id, register_id,
                            raw_value);
  }

  // 发送电机通用 8 字节命令帧，例如使能和失能。
  bool sendMotorCommand(const canid_t frame_id, const uint8_t command)
  {
    uint8_t payload[8];
    for (int i = 0; i < 7; ++i)
    {
      payload[i] = 0xFF;
    }
    payload[7] = command;
    return writeFrame(frame_id, payload, 8);
  }

  // 按速度模式报文格式发送目标速度，单位为 rad/s。
  bool sendSpeed(const canid_t speed_frame_base, const uint32_t motor_id,
                 const float speed_rad_s)
  {
    uint32_t raw_value = 0;
    std::memcpy(&raw_value, &speed_rad_s, sizeof(raw_value));

    uint8_t payload[4];
    payload[0] = static_cast<uint8_t>(raw_value & 0xFF);
    payload[1] = static_cast<uint8_t>((raw_value >> 8) & 0xFF);
    payload[2] = static_cast<uint8_t>((raw_value >> 16) & 0xFF);
    payload[3] = static_cast<uint8_t>((raw_value >> 24) & 0xFF);

    return writeFrame(speed_frame_base + motor_id, payload, 4);
  }

  // 按官方文档的高位在前位打包格式发送 MIT 控制帧。
  bool sendMitCommand(const uint32_t motor_id, const MitPackedCommand& packed_command)
  {
    uint8_t payload[8];
    payload[0] = static_cast<uint8_t>((packed_command.position >> 8) & 0xFF);
    payload[1] = static_cast<uint8_t>(packed_command.position & 0xFF);
    payload[2] = static_cast<uint8_t>((packed_command.velocity >> 4) & 0xFF);
    payload[3] = static_cast<uint8_t>(((packed_command.velocity & 0x0F) << 4) |
                                      ((packed_command.kp >> 8) & 0x0F));
    payload[4] = static_cast<uint8_t>(packed_command.kp & 0xFF);
    payload[5] = static_cast<uint8_t>((packed_command.kd >> 4) & 0xFF);
    payload[6] = static_cast<uint8_t>(((packed_command.kd & 0x0F) << 4) |
                                      ((packed_command.torque >> 8) & 0x0F));
    payload[7] = static_cast<uint8_t>(packed_command.torque & 0xFF);
    return writeFrame(motor_id, payload, 8);
  }

  // 等待寄存器写入回执，确认驱动器已真正接受参数。
  bool waitForWriteAck(const canid_t master_id, const uint32_t motor_id,
                       const uint8_t write_register_cmd, const uint8_t register_id,
                       const uint32_t expected_value, const int timeout_ms)
  {
    const ros::WallTime deadline =
        ros::WallTime::now() + ros::WallDuration(static_cast<double>(timeout_ms) / 1000.0);

    while (ros::WallTime::now() < deadline)
    {
      struct can_frame frame;
      const int remaining_ms = std::max(
          0, static_cast<int>((deadline - ros::WallTime::now()).toSec() * 1000.0));
      if (!readFrame(frame, remaining_ms))
      {
        continue;
      }

      if (frame.can_id != master_id || frame.can_dlc < 8)
      {
        continue;
      }

      const uint32_t ack_motor_id =
          static_cast<uint32_t>(frame.data[0]) |
          (static_cast<uint32_t>(frame.data[1]) << 8);
      const uint32_t ack_value =
          static_cast<uint32_t>(frame.data[4]) |
          (static_cast<uint32_t>(frame.data[5]) << 8) |
          (static_cast<uint32_t>(frame.data[6]) << 16) |
          (static_cast<uint32_t>(frame.data[7]) << 24);

      if (ack_motor_id != motor_id)
      {
        continue;
      }
      if (frame.data[2] != write_register_cmd || frame.data[3] != register_id)
      {
        continue;
      }
      if (ack_value != expected_value)
      {
        continue;
      }
      return true;
    }

    return false;
  }

  // 等待反馈帧进入目标状态，用于确认使能或失能已生效。
  bool waitForFeedbackState(const canid_t master_id, const uint32_t motor_id,
                            const uint8_t expected_state, const int timeout_ms)
  {
    const ros::WallTime deadline =
        ros::WallTime::now() + ros::WallDuration(static_cast<double>(timeout_ms) / 1000.0);

    while (ros::WallTime::now() < deadline)
    {
      struct can_frame frame;
      const int remaining_ms = std::max(
          0, static_cast<int>((deadline - ros::WallTime::now()).toSec() * 1000.0));
      if (!readFrame(frame, remaining_ms))
      {
        continue;
      }

      if (frame.can_id != master_id || frame.can_dlc < 1)
      {
        continue;
      }

      const uint8_t feedback_state = static_cast<uint8_t>((frame.data[0] >> 4) & 0x0F);
      const uint8_t feedback_motor_id = static_cast<uint8_t>(frame.data[0] & 0x0F);
      if (feedback_motor_id != static_cast<uint8_t>(motor_id & 0x0F))
      {
        continue;
      }
      if (feedback_state != expected_state)
      {
        continue;
      }
      return true;
    }

    return false;
  }

private:
  // 带超时读取一帧 CAN 报文，超时返回 false。
  bool readFrame(struct can_frame& frame, const int timeout_ms)
  {
    if (socket_fd_ < 0)
    {
      return false;
    }

    struct pollfd poll_fd;
    poll_fd.fd = socket_fd_;
    poll_fd.events = POLLIN;
    poll_fd.revents = 0;

    const int poll_result = ::poll(&poll_fd, 1, timeout_ms);
    if (poll_result <= 0)
    {
      return false;
    }

    const ssize_t read_size = ::read(socket_fd_, &frame, sizeof(frame));
    return read_size == static_cast<ssize_t>(sizeof(frame));
  }

  // 向 CAN 总线写入一帧原始报文。
  bool writeFrame(const canid_t frame_id, const uint8_t* payload, const uint8_t dlc)
  {
    if (socket_fd_ < 0)
    {
      ROS_ERROR("CAN socket 尚未打开");
      return false;
    }

    struct can_frame frame;
    std::memset(&frame, 0, sizeof(frame));
    frame.can_id = frame_id;
    frame.can_dlc = dlc;
    std::memcpy(frame.data, payload, dlc);

    const ssize_t written = ::write(socket_fd_, &frame, sizeof(frame));
    if (written != static_cast<ssize_t>(sizeof(frame)))
    {
      ROS_ERROR_STREAM("发送 CAN 帧失败, frame_id=0x" << std::hex << frame_id
                       << std::dec << ", error=" << std::strerror(errno));
      return false;
    }
    return true;
  }

  int socket_fd_ = -1;
};

class DamiaoDiffChassisController
{
public:
  // 构造控制器对象，并加载当前选中的模式配置。
  DamiaoDiffChassisController()
    : common_config_(makeCommonControlConfig()),
      mit_config_(makeMitControlConfig()),
      last_cmd_time_(0.0)
  {
  }

  // 析构时先下发零指令，再尝试让两台电机失能。
  ~DamiaoDiffChassisController()
  {
    if (!can_bus_.isOpen())
    {
      return;
    }

    sendNeutralCommands();
    can_bus_.sendMotorCommand(
        makeControlFrameId(common_config_, common_config_.left_motor_id),
        common_config_.disable_command);
    can_bus_.sendMotorCommand(
        makeControlFrameId(common_config_, common_config_.right_motor_id),
        common_config_.disable_command);
  }

  // 初始化控制节点，完成 CAN 打开、电机配置和 ROS 通信对象创建。
  bool init()
  {
    ROS_INFO_STREAM("启动达妙差速底盘控制节点, mode=" << getSelectedControlModeName()
                    << ", can=" << common_config_.can_interface
                    << ", left_id=" << common_config_.left_motor_id
                    << ", right_id=" << common_config_.right_motor_id
                    << ", control_mode_code=" << common_config_.control_mode
                    << ", track=" << common_config_.wheel_track_meters
                    << " m, wheelbase=" << common_config_.wheelbase_meters
                    << " m, wheel_radius=" << common_config_.wheel_radius_meters << " m");

    if (isMitModeSelected())
    {
      ROS_INFO_STREAM("MIT 映射范围: PMax=" << mit_config_.position_map_max_rad
                      << " rad, VMax=" << mit_config_.velocity_map_max_rad_per_sec
                      << " rad/s, TMax=" << mit_config_.torque_map_max
                      << ", kp=" << mit_config_.velocity_control_kp
                      << ", kd=" << mit_config_.velocity_control_kd);
    }

    if ((common_config_.left_motor_id & ~0x0Fu) != 0u ||
        (common_config_.right_motor_id & ~0x0Fu) != 0u)
    {
      ROS_WARN("官方反馈帧表格将 D[0] 写为 ID|ERR<<4，本节点按低 4 位电机 ID 校验使能状态，"
               "若电机 ID 大于 15，请额外确认反馈匹配规则。");
    }

    if (!can_bus_.open(common_config_.can_interface))
    {
      return false;
    }

    if (!configureMotor(common_config_.left_motor_id) ||
        !configureMotor(common_config_.right_motor_id))
    {
      return false;
    }

    cmd_sub_ = nh_.subscribe("cmd_vel", 10, &DamiaoDiffChassisController::cmdVelCallback, this);
    control_timer_ = nh_.createTimer(
        ros::Duration(common_config_.control_period_sec),
        &DamiaoDiffChassisController::controlTimerCallback,
        this);

    return true;
  }

private:
  // 按启动顺序配置单台电机：设置模式、写入 MIT 映射参数、使能、发送零指令。
  bool configureMotor(const uint32_t motor_id)
  {
    can_bus_.clearPendingFrames();
    if (!can_bus_.writeRegisterU32(common_config_.register_frame_id,
                                   common_config_.write_register_cmd, motor_id,
                                   common_config_.control_mode_register,
                                   common_config_.control_mode))
    {
      ROS_ERROR_STREAM("设置电机 " << motor_id << " 模式失败");
      return false;
    }
    if (!can_bus_.waitForWriteAck(common_config_.master_id, motor_id,
                                  common_config_.write_register_cmd,
                                  common_config_.control_mode_register,
                                  common_config_.control_mode,
                                  common_config_.register_ack_timeout_ms))
    {
      ROS_ERROR_STREAM("未收到电机 " << motor_id << " 的模式写入回执");
      return false;
    }

    ros::Duration(common_config_.startup_delay_sec).sleep();

    if (isMitModeSelected() && !configureMitRegisters(motor_id))
    {
      return false;
    }

    can_bus_.clearPendingFrames();
    if (!can_bus_.sendMotorCommand(makeControlFrameId(common_config_, motor_id),
                                   common_config_.enable_command))
    {
      ROS_ERROR_STREAM("使能电机 " << motor_id << " 失败");
      return false;
    }

    ros::Duration(common_config_.startup_delay_sec).sleep();

    if (!sendNeutralCommand(motor_id))
    {
      ROS_ERROR_STREAM("发送电机 " << motor_id << " 初始指令失败");
      return false;
    }
    if (!can_bus_.waitForFeedbackState(common_config_.master_id, motor_id,
                                       common_cfg::kEnabledStateCode,
                                       common_config_.status_feedback_timeout_ms))
    {
      ROS_ERROR_STREAM("未收到电机 " << motor_id << " 的使能状态反馈");
      return false;
    }

    return true;
  }

  // 在 MIT 模式下写入 PMAX、VMAX 和 TMAX 映射寄存器。
  bool configureMitRegisters(const uint32_t motor_id)
  {
    can_bus_.clearPendingFrames();
    if (!can_bus_.writeRegisterFloat(common_config_.register_frame_id,
                                     common_config_.write_register_cmd, motor_id,
                                     mit_config_.p_max_register,
                                     mit_config_.position_map_max_rad))
    {
      ROS_ERROR_STREAM("设置电机 " << motor_id << " 的 PMAX 失败");
      return false;
    }
    {
      uint32_t raw_value = 0;
      std::memcpy(&raw_value, &mit_config_.position_map_max_rad, sizeof(raw_value));
      if (!can_bus_.waitForWriteAck(common_config_.master_id, motor_id,
                                    common_config_.write_register_cmd,
                                    mit_config_.p_max_register, raw_value,
                                    common_config_.register_ack_timeout_ms))
      {
        ROS_ERROR_STREAM("未收到电机 " << motor_id << " 的 PMAX 写入回执");
        return false;
      }
    }

    ros::Duration(common_config_.startup_delay_sec).sleep();

    can_bus_.clearPendingFrames();
    if (!can_bus_.writeRegisterFloat(common_config_.register_frame_id,
                                     common_config_.write_register_cmd, motor_id,
                                     mit_config_.v_max_register,
                                     mit_config_.velocity_map_max_rad_per_sec))
    {
      ROS_ERROR_STREAM("设置电机 " << motor_id << " 的 VMAX 失败");
      return false;
    }
    {
      uint32_t raw_value = 0;
      std::memcpy(&raw_value, &mit_config_.velocity_map_max_rad_per_sec, sizeof(raw_value));
      if (!can_bus_.waitForWriteAck(common_config_.master_id, motor_id,
                                    common_config_.write_register_cmd,
                                    mit_config_.v_max_register, raw_value,
                                    common_config_.register_ack_timeout_ms))
      {
        ROS_ERROR_STREAM("未收到电机 " << motor_id << " 的 VMAX 写入回执");
        return false;
      }
    }

    ros::Duration(common_config_.startup_delay_sec).sleep();

    can_bus_.clearPendingFrames();
    if (!can_bus_.writeRegisterFloat(common_config_.register_frame_id,
                                     common_config_.write_register_cmd, motor_id,
                                     mit_config_.t_max_register,
                                     mit_config_.torque_map_max))
    {
      ROS_ERROR_STREAM("设置电机 " << motor_id << " 的 TMAX 失败");
      return false;
    }
    {
      uint32_t raw_value = 0;
      std::memcpy(&raw_value, &mit_config_.torque_map_max, sizeof(raw_value));
      if (!can_bus_.waitForWriteAck(common_config_.master_id, motor_id,
                                    common_config_.write_register_cmd,
                                    mit_config_.t_max_register, raw_value,
                                    common_config_.register_ack_timeout_ms))
      {
        ROS_ERROR_STREAM("未收到电机 " << motor_id << " 的 TMAX 写入回执");
        return false;
      }
    }

    ros::Duration(common_config_.startup_delay_sec).sleep();
    return true;
  }

  // 发送单台电机的零速或 MIT 零指令，用于启动和退出时的安全状态。
  bool sendNeutralCommand(const uint32_t motor_id)
  {
    if (isMitModeSelected())
    {
      return sendMitVelocityCommand(motor_id, 0.0);
    }

    return can_bus_.sendSpeed(common_config_.control_frame_base, motor_id, 0.0f);
  }

  // 同时向左右电机发送零速或 MIT 零指令。
  void sendNeutralCommands()
  {
    sendNeutralCommand(common_config_.left_motor_id);
    sendNeutralCommand(common_config_.right_motor_id);
  }

  // 在 MIT 模式下按速度控制思路发送一帧 MIT 控制命令。
  bool sendMitVelocityCommand(const uint32_t motor_id, const double velocity_rad_s)
  {
    MitCommand command;
    command.position_rad = mit_config_.hold_position_rad;
    command.velocity_rad_s = clampSigned(
        velocity_rad_s, static_cast<double>(mit_config_.velocity_map_max_rad_per_sec));
    command.kp = mit_config_.velocity_control_kp;
    command.kd = mit_config_.velocity_control_kd;
    command.torque_ff = mit_config_.feedforward_torque;
    return can_bus_.sendMitCommand(motor_id, packMitCommand(command, mit_config_));
  }

  // 处理 cmd_vel 订阅消息，并更新左右轮目标速度。
  void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg)
  {
    if (std::fabs(msg->linear.y) > 1e-6)
    {
      ROS_WARN_THROTTLE(1.0, "差速底盘不支持 lateral velocity, 将忽略 cmd_vel.linear.y");
    }

    target_wheel_speeds_ = inverseKinematics(*msg, common_config_);
    last_cmd_time_ = ros::Time::now();
  }

  // 定时下发电机指令，并在指令超时时自动停车。
  void controlTimerCallback(const ros::TimerEvent&)
  {
    WheelSpeeds wheel_speeds = target_wheel_speeds_;

    if (!last_cmd_time_.isValid() ||
        (ros::Time::now() - last_cmd_time_).toSec() > common_config_.cmd_timeout_sec)
    {
      wheel_speeds = WheelSpeeds();
    }

    sendWheelCommands(wheel_speeds);
  }

  // 将左右轮速度换成电机速度方向后，按当前选中的模式发送到两台电机。
  void sendWheelCommands(const WheelSpeeds& wheel_speeds)
  {
    const double left_motor_speed =
        clampSigned(wheel_speeds.left_rad_s, common_config_.max_motor_speed_rad_per_sec) *
        common_config_.left_motor_direction;
    const double right_motor_speed =
        clampSigned(wheel_speeds.right_rad_s, common_config_.max_motor_speed_rad_per_sec) *
        common_config_.right_motor_direction;

    bool left_ok = false;
    bool right_ok = false;

    if (isMitModeSelected())
    {
      left_ok = sendMitVelocityCommand(common_config_.left_motor_id, left_motor_speed);
      right_ok = sendMitVelocityCommand(common_config_.right_motor_id, right_motor_speed);
    }
    else
    {
      left_ok = can_bus_.sendSpeed(common_config_.control_frame_base,
                                   common_config_.left_motor_id,
                                   static_cast<float>(left_motor_speed));
      right_ok = can_bus_.sendSpeed(common_config_.control_frame_base,
                                    common_config_.right_motor_id,
                                    static_cast<float>(right_motor_speed));
    }

    if (!left_ok)
    {
      ROS_ERROR_THROTTLE(1.0, "左电机指令发送失败");
    }

    if (!right_ok)
    {
      ROS_ERROR_THROTTLE(1.0, "右电机指令发送失败");
    }
  }

  CommonControlConfig common_config_;
  MitControlConfig mit_config_;

  ros::NodeHandle nh_;
  ros::Subscriber cmd_sub_;
  ros::Timer control_timer_;

  SocketCanBus can_bus_;
  WheelSpeeds target_wheel_speeds_;
  ros::Time last_cmd_time_;
};
}  // 命名空间

// 节点入口，初始化 ROS 后启动达妙差速底盘控制器。
int main(int argc, char** argv)
{
  ros::init(argc, argv, "damiao_diff_chassis_node");

  DamiaoDiffChassisController controller;
  if (!controller.init())
  {
    return 1;
  }

  ros::spin();
  return 0;
}
