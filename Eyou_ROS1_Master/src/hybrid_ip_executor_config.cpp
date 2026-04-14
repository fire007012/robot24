#include "Eyou_ROS1_Master/hybrid_ip_executor_config.hpp"

#include <algorithm>
#include <cmath>
#include <set>
#include <string>
#include <unordered_map>

#include <ros/node_handle.h>
#include <yaml-cpp/yaml.h>

#include "can_driver/AxisCommandSemantics.h"

namespace eyou_ros1_master {

namespace {

constexpr double kDefaultIpMaxVelocity = 1.0;
constexpr double kDefaultIpMaxAcceleration = 2.0;
constexpr double kDefaultIpMaxJerk = 10.0;
constexpr double kDefaultIpGoalTolerance = 1e-3;
constexpr double kEpsilon = 1e-12;

struct MoveItJointLimit {
    bool has_velocity_limits{false};
    double max_velocity{0.0};
    bool has_acceleration_limits{false};
    double max_acceleration{0.0};
};

void SetError(std::string* error, const std::string& message) {
    if (error != nullptr) {
        *error = message;
    }
}

bool ParseActionVelocityHintMode(const std::string& value,
                                 ActionVelocityHintMode* mode,
                                 std::string* error) {
    if (mode == nullptr) {
        SetError(error, "action_velocity_hint_mode output is null");
        return false;
    }
    if (value == "disabled") {
        *mode = ActionVelocityHintMode::kDisabled;
        return true;
    }
    if (value == "single_point") {
        *mode = ActionVelocityHintMode::kSinglePoint;
        return true;
    }
    if (value == "all") {
        *mode = ActionVelocityHintMode::kAll;
        return true;
    }

    SetError(error,
             "action_velocity_hint_mode must be one of "
             "{disabled,single_point,all}");
    return false;
}

bool IsFiniteNonNegative(double value) {
    return std::isfinite(value) && value >= 0.0;
}

bool IsFinitePositive(double value) {
    return std::isfinite(value) && value > 0.0;
}

bool ParseXmlRpcDouble(const XmlRpc::XmlRpcValue& value,
                       const std::string& field_name,
                       const std::string& joint_name,
                       bool allow_zero,
                       double* out,
                       std::string* error) {
    if (value.getType() != XmlRpc::XmlRpcValue::TypeInt &&
        value.getType() != XmlRpc::XmlRpcValue::TypeDouble) {
        SetError(error, "Joint '" + joint_name + "': invalid " + field_name + ".");
        return false;
    }

    const double parsed = (value.getType() == XmlRpc::XmlRpcValue::TypeInt)
                              ? static_cast<double>(static_cast<int>(value))
                              : static_cast<double>(value);
    if (!std::isfinite(parsed) || parsed < 0.0 || (!allow_zero && parsed == 0.0)) {
        SetError(error, "Joint '" + joint_name + "': invalid " + field_name + ".");
        return false;
    }

    *out = parsed;
    return true;
}

bool AppendExecutorJoint(const std::string& name,
                         double max_velocity,
                         double max_acceleration,
                         double max_jerk,
                         double default_goal_tolerance,
                         std::set<std::string>* seen,
                         HybridIpExecutorConfig* config,
                         std::string* error) {
    if (seen == nullptr || config == nullptr) {
        SetError(error, "executor config builder received null output");
        return false;
    }
    if (name.empty()) {
        SetError(error, "executor joint name must not be empty");
        return false;
    }
    if (!seen->insert(name).second) {
        SetError(error, "duplicate executor joint name: " + name);
        return false;
    }

    config->joint_names.push_back(name);
    config->joint_indices.push_back(config->joint_indices.size());
    config->max_velocities.push_back(max_velocity);
    config->max_accelerations.push_back(max_acceleration);
    config->max_jerks.push_back(max_jerk);
    config->default_goal_tolerances.push_back(default_goal_tolerance);
    return true;
}

bool ValidateOverrides(const HybridIpExecutorConfigOverrides& overrides,
                       std::string* error) {
    if (overrides.default_goal_tolerance_position.has_value() &&
        !IsFiniteNonNegative(*overrides.default_goal_tolerance_position)) {
        SetError(error, "default_goal_tolerance_position must be >= 0");
        return false;
    }
    if (!IsFiniteNonNegative(overrides.default_path_tolerance_position)) {
        SetError(error, "default_path_tolerance_position must be >= 0");
        return false;
    }
    if (!IsFiniteNonNegative(overrides.default_stopped_velocity_tolerance)) {
        SetError(error, "default_stopped_velocity_tolerance must be >= 0");
        return false;
    }
    if (!IsFiniteNonNegative(overrides.default_goal_time_tolerance)) {
        SetError(error, "default_goal_time_tolerance must be >= 0");
        return false;
    }
    if (!IsFiniteNonNegative(overrides.planning_deviation_warn_threshold)) {
        SetError(error, "planning_deviation_warn_threshold must be >= 0");
        return false;
    }
    if (!IsFinitePositive(overrides.tracking_fault_threshold)) {
        SetError(error, "tracking_fault_threshold must be > 0");
        return false;
    }
    if (!IsFinitePositive(overrides.continuous_resync_threshold)) {
        SetError(error, "continuous_mode.resync_threshold must be > 0");
        return false;
    }
    if (!IsFinitePositive(overrides.continuous_resync_recovery_threshold)) {
        SetError(error,
                 "continuous_mode.resync_recovery_threshold must be > 0");
        return false;
    }
    if (overrides.continuous_resync_recovery_threshold >
        overrides.continuous_resync_threshold) {
        SetError(
            error,
            "continuous_mode.resync_recovery_threshold must be <= "
            "continuous_mode.resync_threshold");
        return false;
    }
    if (overrides.continuous_resync_threshold >=
        overrides.tracking_fault_threshold) {
        SetError(
            error,
            "continuous_mode.resync_threshold must be < tracking_fault_threshold");
        return false;
    }
    if (overrides.continuous_resync_enter_cycles == 0u) {
        SetError(error, "continuous_mode.resync_enter_cycles must be >= 1");
        return false;
    }
    if (overrides.continuous_resync_recovery_cycles == 0u) {
        SetError(error, "continuous_mode.resync_recovery_cycles must be >= 1");
        return false;
    }
    if (!std::isfinite(overrides.constraint_validation_velocity_margin) ||
        overrides.constraint_validation_velocity_margin < 1.0) {
        SetError(error,
                 "constraint_validation.velocity_margin must be >= 1.0");
        return false;
    }
    if (!std::isfinite(overrides.constraint_validation_acceleration_margin) ||
        overrides.constraint_validation_acceleration_margin < 1.0) {
        SetError(error,
                 "constraint_validation.acceleration_margin must be >= 1.0");
        return false;
    }
    if (overrides.constraint_validation_enabled &&
        overrides.moveit_joint_limits_path.empty()) {
        SetError(error,
                 "constraint_validation.moveit_joint_limits_path is required "
                 "when validation is enabled");
        return false;
    }
    return true;
}

bool LoadMoveItJointLimits(
    const std::string& path,
    std::unordered_map<std::string, MoveItJointLimit>* limits_by_joint,
    std::string* error) {
    if (limits_by_joint == nullptr) {
        SetError(error, "MoveIt joint limit output is null");
        return false;
    }

    YAML::Node root;
    try {
        root = YAML::LoadFile(path);
    } catch (const std::exception& e) {
        SetError(error,
                 "failed to load MoveIt joint limits file '" + path + "': " +
                     e.what());
        return false;
    }

    const YAML::Node joint_limits = root["joint_limits"];
    if (!joint_limits || !joint_limits.IsMap()) {
        SetError(error, "MoveIt joint limits file missing 'joint_limits' map");
        return false;
    }

    limits_by_joint->clear();
    for (const auto& entry : joint_limits) {
        const std::string joint_name = entry.first.as<std::string>();
        const YAML::Node values = entry.second;

        MoveItJointLimit limit;
        if (const YAML::Node has_velocity = values["has_velocity_limits"]) {
            limit.has_velocity_limits = has_velocity.as<bool>();
        }
        if (limit.has_velocity_limits) {
            if (!values["max_velocity"]) {
                SetError(error, "MoveIt joint '" + joint_name +
                                    "' is missing max_velocity");
                return false;
            }
            limit.max_velocity = values["max_velocity"].as<double>();
            if (!IsFinitePositive(limit.max_velocity)) {
                SetError(error, "MoveIt joint '" + joint_name +
                                    "' has invalid max_velocity");
                return false;
            }
        }

        if (const YAML::Node has_acceleration = values["has_acceleration_limits"]) {
            limit.has_acceleration_limits = has_acceleration.as<bool>();
        }
        if (limit.has_acceleration_limits) {
            if (!values["max_acceleration"]) {
                SetError(error, "MoveIt joint '" + joint_name +
                                    "' is missing max_acceleration");
                return false;
            }
            limit.max_acceleration = values["max_acceleration"].as<double>();
            if (!IsFinitePositive(limit.max_acceleration)) {
                SetError(error, "MoveIt joint '" + joint_name +
                                    "' has invalid max_acceleration");
                return false;
            }
        }

        limits_by_joint->emplace(joint_name, limit);
    }

    return true;
}

bool ValidateMoveItConstraints(const HybridIpExecutorConfig& config,
                               const HybridIpExecutorConfigOverrides& overrides,
                               std::string* error) {
    if (!overrides.constraint_validation_enabled) {
        return true;
    }

    std::unordered_map<std::string, MoveItJointLimit> limits_by_joint;
    if (!LoadMoveItJointLimits(overrides.moveit_joint_limits_path, &limits_by_joint,
                               error)) {
        return false;
    }

    for (std::size_t axis_index = 0; axis_index < config.joint_names.size();
         ++axis_index) {
        const auto it = limits_by_joint.find(config.joint_names[axis_index]);
        if (it == limits_by_joint.end()) {
            SetError(error, "missing MoveIt joint_limits entry for '" +
                                config.joint_names[axis_index] + "'");
            return false;
        }

        if (it->second.has_velocity_limits) {
            const double required_velocity =
                it->second.max_velocity *
                overrides.constraint_validation_velocity_margin;
            if (config.max_velocities[axis_index] + kEpsilon < required_velocity) {
                SetError(error,
                         "joint '" + config.joint_names[axis_index] +
                             "' ip_max_velocity is smaller than MoveIt limit");
                return false;
            }
        }

        if (it->second.has_acceleration_limits) {
            const double required_acceleration =
                it->second.max_acceleration *
                overrides.constraint_validation_acceleration_margin;
            if (config.max_accelerations[axis_index] + kEpsilon <
                required_acceleration) {
                SetError(error,
                         "joint '" + config.joint_names[axis_index] +
                             "' ip_max_acceleration is smaller than MoveIt limit");
                return false;
            }
        }
    }

    return true;
}

}  // namespace

bool BuildHybridIpExecutorConfig(
    const canopen_hw::CanopenMasterConfig& canopen_config,
    const XmlRpc::XmlRpcValue& can_driver_joint_list,
    const std::string& action_ns,
    double command_rate_hz,
    const HybridIpExecutorConfigOverrides& overrides,
    HybridIpExecutorConfig* config,
    std::string* error) {
    if (error != nullptr) {
        error->clear();
    }
    if (config == nullptr) {
        SetError(error, "executor config output is null");
        return false;
    }
    if (!ValidateOverrides(overrides, error)) {
        return false;
    }
    if (action_ns.empty()) {
        SetError(error, "action_ns must not be empty");
        return false;
    }
    if (command_rate_hz <= 0.0) {
        SetError(error, "command_rate_hz must be > 0");
        return false;
    }
    if (can_driver_joint_list.getType() != XmlRpc::XmlRpcValue::TypeArray) {
        SetError(error, "can_driver joints config is not an array");
        return false;
    }

    HybridIpExecutorConfig merged;
    merged.joint_names.clear();
    merged.joint_indices.clear();
    merged.max_velocities.clear();
    merged.max_accelerations.clear();
    merged.max_jerks.clear();
    merged.default_goal_tolerances.clear();
    merged.default_path_tolerances.clear();
    merged.action_ns = action_ns;
    merged.command_rate_hz = command_rate_hz;
    merged.default_stopped_velocity_tolerance =
        overrides.default_stopped_velocity_tolerance;
    merged.default_goal_time_tolerance = overrides.default_goal_time_tolerance;
    merged.planning_deviation_warn_threshold =
        overrides.planning_deviation_warn_threshold;
    merged.tracking_fault_threshold = overrides.tracking_fault_threshold;
    merged.continuous_resync_threshold = overrides.continuous_resync_threshold;
    merged.continuous_resync_recovery_threshold =
        overrides.continuous_resync_recovery_threshold;
    merged.continuous_resync_enter_cycles =
        overrides.continuous_resync_enter_cycles;
    merged.continuous_resync_recovery_cycles =
        overrides.continuous_resync_recovery_cycles;
    merged.action_velocity_hint_mode = overrides.action_velocity_hint_mode;

    std::set<std::string> seen_joint_names;
    for (const auto& joint : canopen_config.joints) {
        if (!AppendExecutorJoint(joint.name,
                                 joint.ip_max_velocity,
                                 joint.ip_max_acceleration,
                                 joint.ip_max_jerk,
                                 joint.ip_goal_tolerance,
                                 &seen_joint_names,
                                 &merged,
                                 error)) {
            return false;
        }
    }

    for (int index = 0; index < can_driver_joint_list.size(); ++index) {
        const auto& joint = can_driver_joint_list[index];
        if (joint.getType() != XmlRpc::XmlRpcValue::TypeStruct) {
            SetError(error, "can_driver joints[" + std::to_string(index) +
                                "] is not a struct");
            return false;
        }
        if (!joint.hasMember("name") || !joint.hasMember("control_mode")) {
            SetError(error, "can_driver joints[" + std::to_string(index) +
                                "] missing name/control_mode");
            return false;
        }

        const std::string joint_name = static_cast<std::string>(joint["name"]);
        const std::string control_mode =
            static_cast<std::string>(joint["control_mode"]);
        const auto axis_mode = can_driver::axisControlModeFromString(control_mode);
        if (axis_mode == can_driver::AxisControlMode::Unknown) {
            SetError(error, "Joint '" + joint_name + "': unknown control_mode '" +
                                control_mode + "'.");
            return false;
        }
        if (!can_driver::controlModeUsesPositionSemantics(axis_mode)) {
            continue;
        }

        double max_velocity = kDefaultIpMaxVelocity;
        double max_acceleration = kDefaultIpMaxAcceleration;
        double max_jerk = kDefaultIpMaxJerk;
        double goal_tolerance = kDefaultIpGoalTolerance;

        if (joint.hasMember("ip_max_velocity") &&
            !ParseXmlRpcDouble(joint["ip_max_velocity"], "ip_max_velocity",
                               joint_name, false, &max_velocity, error)) {
            return false;
        }
        if (joint.hasMember("ip_max_acceleration") &&
            !ParseXmlRpcDouble(joint["ip_max_acceleration"], "ip_max_acceleration",
                               joint_name, false, &max_acceleration, error)) {
            return false;
        }
        if (joint.hasMember("ip_max_jerk") &&
            !ParseXmlRpcDouble(joint["ip_max_jerk"], "ip_max_jerk",
                               joint_name, false, &max_jerk, error)) {
            return false;
        }
        if (joint.hasMember("ip_goal_tolerance") &&
            !ParseXmlRpcDouble(joint["ip_goal_tolerance"], "ip_goal_tolerance",
                               joint_name, true, &goal_tolerance, error)) {
            return false;
        }

        if (!AppendExecutorJoint(joint_name,
                                 max_velocity,
                                 max_acceleration,
                                 max_jerk,
                                 goal_tolerance,
                                 &seen_joint_names,
                                 &merged,
                                 error)) {
            return false;
        }
    }

    if (merged.joint_names.empty()) {
        SetError(error, "executor has no configured joints");
        return false;
    }

    merged.default_path_tolerances.assign(merged.joint_names.size(),
                                          overrides.default_path_tolerance_position);
    if (overrides.default_goal_tolerance_position.has_value()) {
        merged.default_goal_tolerances.assign(
            merged.joint_names.size(), *overrides.default_goal_tolerance_position);
    }

    if (!ValidateMoveItConstraints(merged, overrides, error)) {
        return false;
    }

    *config = std::move(merged);
    return true;
}

bool BuildHybridIpExecutorConfigFromParams(
    const canopen_hw::CanopenMasterConfig& canopen_config,
    const ros::NodeHandle& executor_pnh,
    const ros::NodeHandle& can_driver_pnh,
    const std::string& action_ns,
    double command_rate_hz,
    HybridIpExecutorConfig* config,
    std::string* error) {
    XmlRpc::XmlRpcValue joint_list;
    if (!can_driver_pnh.getParam("joints", joint_list)) {
        SetError(error, "can_driver joints config is missing");
        return false;
    }

    HybridIpExecutorConfigOverrides overrides;
    if (executor_pnh.hasParam("default_goal_tolerance_position")) {
        double goal_tolerance_override = 0.0;
        executor_pnh.getParam("default_goal_tolerance_position",
                              goal_tolerance_override);
        overrides.default_goal_tolerance_position = goal_tolerance_override;
    }
    executor_pnh.param("default_path_tolerance_position",
                       overrides.default_path_tolerance_position,
                       overrides.default_path_tolerance_position);
    executor_pnh.param("default_stopped_velocity_tolerance",
                       overrides.default_stopped_velocity_tolerance,
                       overrides.default_stopped_velocity_tolerance);
    executor_pnh.param("default_goal_time_tolerance",
                       overrides.default_goal_time_tolerance,
                       overrides.default_goal_time_tolerance);
    executor_pnh.param("planning_deviation_warn_threshold",
                       overrides.planning_deviation_warn_threshold,
                       overrides.planning_deviation_warn_threshold);
    executor_pnh.param("tracking_fault_threshold",
                       overrides.tracking_fault_threshold,
                       overrides.tracking_fault_threshold);

    ros::NodeHandle continuous_pnh(executor_pnh, "continuous_mode");
    continuous_pnh.param("resync_threshold", overrides.continuous_resync_threshold,
                         overrides.continuous_resync_threshold);
    continuous_pnh.param("resync_recovery_threshold",
                         overrides.continuous_resync_recovery_threshold,
                         overrides.continuous_resync_recovery_threshold);
    int resync_enter_cycles =
        static_cast<int>(overrides.continuous_resync_enter_cycles);
    int resync_recovery_cycles =
        static_cast<int>(overrides.continuous_resync_recovery_cycles);
    continuous_pnh.param("resync_enter_cycles", resync_enter_cycles,
                         resync_enter_cycles);
    continuous_pnh.param("resync_recovery_cycles", resync_recovery_cycles,
                         resync_recovery_cycles);
    overrides.continuous_resync_enter_cycles =
        static_cast<std::size_t>(std::max(0, resync_enter_cycles));
    overrides.continuous_resync_recovery_cycles =
        static_cast<std::size_t>(std::max(0, resync_recovery_cycles));

    ros::NodeHandle validation_pnh(executor_pnh, "constraint_validation");
    validation_pnh.param("enabled", overrides.constraint_validation_enabled,
                         overrides.constraint_validation_enabled);
    validation_pnh.param("moveit_joint_limits_path",
                         overrides.moveit_joint_limits_path,
                         overrides.moveit_joint_limits_path);
    validation_pnh.param("velocity_margin",
                         overrides.constraint_validation_velocity_margin,
                         overrides.constraint_validation_velocity_margin);
    validation_pnh.param("acceleration_margin",
                         overrides.constraint_validation_acceleration_margin,
                         overrides.constraint_validation_acceleration_margin);
    std::string action_velocity_hint_mode = "disabled";
    executor_pnh.param("action_velocity_hint_mode", action_velocity_hint_mode,
                       action_velocity_hint_mode);
    if (!ParseActionVelocityHintMode(action_velocity_hint_mode,
                                     &overrides.action_velocity_hint_mode, error)) {
        return false;
    }

    return BuildHybridIpExecutorConfig(
        canopen_config, joint_list, action_ns, command_rate_hz, overrides, config,
        error);
}

}  // namespace eyou_ros1_master
