#include "Eyou_ROS1_Master/hybrid_tolerance_resolver.hpp"

#include <cmath>

namespace eyou_ros1_master {

namespace {

void SetError(std::string* error, const std::string& message) {
    if (error != nullptr) {
        *error = message;
    }
}

bool ValidateToleranceVector(const std::vector<double>& values,
                             std::size_t expected_size,
                             const std::string& field_name,
                             std::string* error) {
    if (values.size() != expected_size) {
        SetError(error, field_name + " size mismatch: expected " +
                            std::to_string(expected_size) + ", got " +
                            std::to_string(values.size()));
        return false;
    }
    for (std::size_t i = 0; i < values.size(); ++i) {
        if (!std::isfinite(values[i]) || values[i] < 0.0) {
            SetError(error, field_name + "[" + std::to_string(i) +
                                "] must be finite and >= 0");
            return false;
        }
    }
    return true;
}

bool ValidateScalarTolerance(double value,
                             const std::string& field_name,
                             std::string* error) {
    if (!std::isfinite(value) || value < 0.0) {
        SetError(error, field_name + " must be finite and >= 0");
        return false;
    }
    return true;
}

void ApplyToleranceMessage(const control_msgs::JointTolerance& msg,
                           JointStateTolerance* tolerance) {
    if (tolerance == nullptr) {
        return;
    }

    if (msg.position > 0.0) {
        tolerance->position = msg.position;
    } else if (msg.position < 0.0) {
        tolerance->position = 0.0;
    }

    if (msg.velocity > 0.0) {
        tolerance->velocity = msg.velocity;
    } else if (msg.velocity < 0.0) {
        tolerance->velocity = 0.0;
    }

    if (msg.acceleration > 0.0) {
        tolerance->acceleration = msg.acceleration;
    } else if (msg.acceleration < 0.0) {
        tolerance->acceleration = 0.0;
    }
}

}  // namespace

bool ResolveFollowJointTrajectoryTolerances(
    const control_msgs::FollowJointTrajectoryGoal& goal,
    const std::vector<std::string>& joint_names,
    const std::vector<double>& default_path_position_tolerances,
    const std::vector<double>& default_goal_position_tolerances,
    double default_stopped_velocity_tolerance,
    double default_goal_time_tolerance,
    FollowJointTrajectoryResolvedTolerances* resolved,
    std::string* error) {
    if (error != nullptr) {
        error->clear();
    }
    if (resolved == nullptr) {
        SetError(error, "resolved tolerances output is null");
        return false;
    }

    const std::size_t joint_count = joint_names.size();
    if (!ValidateToleranceVector(default_path_position_tolerances, joint_count,
                                 "default_path_position_tolerances", error) ||
        !ValidateToleranceVector(default_goal_position_tolerances, joint_count,
                                 "default_goal_position_tolerances", error) ||
        !ValidateScalarTolerance(default_stopped_velocity_tolerance,
                                 "default_stopped_velocity_tolerance", error) ||
        !ValidateScalarTolerance(default_goal_time_tolerance,
                                 "default_goal_time_tolerance", error)) {
        return false;
    }

    resolved->path_state_tolerance.assign(joint_count, JointStateTolerance{});
    resolved->goal_state_tolerance.assign(joint_count, JointStateTolerance{});
    resolved->goal_time_tolerance = default_goal_time_tolerance;

    for (std::size_t i = 0; i < joint_count; ++i) {
        resolved->path_state_tolerance[i].position =
            default_path_position_tolerances[i];
        resolved->goal_state_tolerance[i].position =
            default_goal_position_tolerances[i];
        resolved->goal_state_tolerance[i].velocity =
            default_stopped_velocity_tolerance;
    }

    for (std::size_t joint_index = 0; joint_index < joint_names.size(); ++joint_index) {
        const auto& joint_name = joint_names[joint_index];

        for (const auto& tolerance_msg : goal.path_tolerance) {
            if (tolerance_msg.name == joint_name) {
                ApplyToleranceMessage(tolerance_msg,
                                      &resolved->path_state_tolerance[joint_index]);
            }
        }

        for (const auto& tolerance_msg : goal.goal_tolerance) {
            if (tolerance_msg.name == joint_name) {
                ApplyToleranceMessage(tolerance_msg,
                                      &resolved->goal_state_tolerance[joint_index]);
            }
        }
    }

    if (goal.goal_time_tolerance < ros::Duration(0.0)) {
        resolved->goal_time_tolerance = 0.0;
    } else if (goal.goal_time_tolerance > ros::Duration(0.0)) {
        resolved->goal_time_tolerance = goal.goal_time_tolerance.toSec();
    }

    return true;
}

}  // namespace eyou_ros1_master
