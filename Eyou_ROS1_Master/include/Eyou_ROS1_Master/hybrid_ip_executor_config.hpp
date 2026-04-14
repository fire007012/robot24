#pragma once

#include <cstddef>
#include <optional>
#include <string>
#include <vector>

#include <xmlrpcpp/XmlRpcValue.h>

#include "canopen_hw/canopen_master.hpp"

namespace ros {
class NodeHandle;
}

namespace eyou_ros1_master {

enum class ActionVelocityHintMode {
    kDisabled,
    kSinglePoint,
    kAll,
};

struct HybridIpExecutorConfig {
    std::string action_ns{"ip_follow_joint_trajectory"};
    std::vector<std::string> joint_names{"shoulder_yaw_joint"};
    std::vector<std::size_t> joint_indices{0};
    double command_rate_hz{100.0};
    std::vector<double> max_velocities{1.0};
    std::vector<double> max_accelerations{2.0};
    std::vector<double> max_jerks{10.0};
    std::vector<double> default_goal_tolerances{1e-3};
    std::vector<double> default_path_tolerances{0.0};
    double default_stopped_velocity_tolerance{0.05};
    double default_goal_time_tolerance{2.0};
    double planning_deviation_warn_threshold{0.20};
    double tracking_fault_threshold{0.08};
    double continuous_resync_threshold{0.04};
    double continuous_resync_recovery_threshold{0.02};
    std::size_t continuous_resync_enter_cycles{2};
    std::size_t continuous_resync_recovery_cycles{2};
    ActionVelocityHintMode action_velocity_hint_mode{
        ActionVelocityHintMode::kDisabled};
};

struct HybridIpExecutorConfigOverrides {
    std::optional<double> default_goal_tolerance_position;
    double default_path_tolerance_position{0.0};
    double default_stopped_velocity_tolerance{0.05};
    double default_goal_time_tolerance{2.0};
    double planning_deviation_warn_threshold{0.20};
    double tracking_fault_threshold{0.08};
    double continuous_resync_threshold{0.04};
    double continuous_resync_recovery_threshold{0.02};
    std::size_t continuous_resync_enter_cycles{2};
    std::size_t continuous_resync_recovery_cycles{2};
    bool constraint_validation_enabled{false};
    std::string moveit_joint_limits_path;
    double constraint_validation_velocity_margin{1.05};
    double constraint_validation_acceleration_margin{1.10};
    ActionVelocityHintMode action_velocity_hint_mode{
        ActionVelocityHintMode::kDisabled};
};

bool BuildHybridIpExecutorConfig(
    const canopen_hw::CanopenMasterConfig& canopen_config,
    const XmlRpc::XmlRpcValue& can_driver_joint_list,
    const std::string& action_ns,
    double command_rate_hz,
    const HybridIpExecutorConfigOverrides& overrides,
    HybridIpExecutorConfig* config,
    std::string* error);

bool BuildHybridIpExecutorConfigFromParams(
    const canopen_hw::CanopenMasterConfig& canopen_config,
    const ros::NodeHandle& executor_pnh,
    const ros::NodeHandle& can_driver_pnh,
    const std::string& action_ns,
    double command_rate_hz,
    HybridIpExecutorConfig* config,
    std::string* error);

}  // namespace eyou_ros1_master
