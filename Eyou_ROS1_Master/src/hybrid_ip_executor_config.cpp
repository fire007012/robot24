#include "Eyou_ROS1_Master/hybrid_ip_executor_config.hpp"

#include <set>
#include <string>

#include <ros/node_handle.h>

#include "can_driver/AxisCommandSemantics.h"

namespace eyou_ros1_master {

namespace {

constexpr double kDefaultIpMaxVelocity = 1.0;
constexpr double kDefaultIpMaxAcceleration = 2.0;
constexpr double kDefaultIpMaxJerk = 10.0;
constexpr double kDefaultIpGoalTolerance = 1e-3;

void SetError(std::string* error, const std::string& message) {
    if (error != nullptr) {
        *error = message;
    }
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
                         double goal_tolerance,
                         std::set<std::string>* seen,
                         canopen_hw::IpFollowJointTrajectoryExecutor::Config* config,
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
    config->goal_tolerances.push_back(goal_tolerance);
    return true;
}

}  // namespace

bool BuildHybridIpExecutorConfig(
    const canopen_hw::CanopenMasterConfig& canopen_config,
    const XmlRpc::XmlRpcValue& can_driver_joint_list,
    const std::string& action_ns,
    double command_rate_hz,
    canopen_hw::IpFollowJointTrajectoryExecutor::Config* config,
    std::string* error) {
    if (error != nullptr) {
        error->clear();
    }
    if (config == nullptr) {
        SetError(error, "executor config output is null");
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

    canopen_hw::IpFollowJointTrajectoryExecutor::Config merged;
    merged.joint_names.clear();
    merged.joint_indices.clear();
    merged.max_velocities.clear();
    merged.max_accelerations.clear();
    merged.max_jerks.clear();
    merged.goal_tolerances.clear();
    merged.action_ns = action_ns;
    merged.command_rate_hz = command_rate_hz;

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

    *config = std::move(merged);
    return true;
}

bool BuildHybridIpExecutorConfigFromParams(
    const canopen_hw::CanopenMasterConfig& canopen_config,
    const ros::NodeHandle& can_driver_pnh,
    const std::string& action_ns,
    double command_rate_hz,
    canopen_hw::IpFollowJointTrajectoryExecutor::Config* config,
    std::string* error) {
    XmlRpc::XmlRpcValue joint_list;
    if (!can_driver_pnh.getParam("joints", joint_list)) {
        SetError(error, "can_driver joints config is missing");
        return false;
    }
    return BuildHybridIpExecutorConfig(canopen_config, joint_list, action_ns,
                                       command_rate_hz, config, error);
}

}  // namespace eyou_ros1_master
