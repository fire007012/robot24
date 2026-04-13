#include "Eyou_ROS1_Master/hybrid_trajectory_time_sampler.hpp"

#include <algorithm>
#include <cmath>

namespace eyou_ros1_master {

namespace {

void SetError(std::string* error, const std::string& message) {
    if (error != nullptr) {
        *error = message;
    }
}

double StateValueOrZero(const std::vector<double>& values, std::size_t index) {
    return index < values.size() ? values[index] : 0.0;
}

void EnsureStateArrays(HybridJointTargetExecutor::State* state, std::size_t dofs) {
    if (state == nullptr) {
        return;
    }
    state->positions.resize(dofs, 0.0);
    state->velocities.resize(dofs, 0.0);
    state->accelerations.resize(dofs, 0.0);
}

bool FillStateFromPoint(const trajectory_msgs::JointTrajectoryPoint& point,
                        const std::vector<std::size_t>& goal_to_config_indices,
                        std::size_t dofs,
                        HybridJointTargetExecutor::State* state,
                        std::string* error) {
    if (state == nullptr) {
        SetError(error, "sample output state is null");
        return false;
    }
    EnsureStateArrays(state, dofs);

    if (point.positions.size() != goal_to_config_indices.size()) {
        SetError(error, "trajectory point positions size mismatch");
        return false;
    }
    if (!point.velocities.empty() &&
        point.velocities.size() != goal_to_config_indices.size()) {
        SetError(error, "trajectory point velocities size mismatch");
        return false;
    }
    if (!point.accelerations.empty() &&
        point.accelerations.size() != goal_to_config_indices.size()) {
        SetError(error, "trajectory point accelerations size mismatch");
        return false;
    }

    std::fill(state->positions.begin(), state->positions.end(), 0.0);
    std::fill(state->velocities.begin(), state->velocities.end(), 0.0);
    std::fill(state->accelerations.begin(), state->accelerations.end(), 0.0);
    for (std::size_t goal_index = 0; goal_index < goal_to_config_indices.size();
         ++goal_index) {
        const std::size_t axis_index = goal_to_config_indices[goal_index];
        if (axis_index >= dofs) {
            SetError(error, "goal_to_config_indices contains out-of-range axis index");
            return false;
        }
        state->positions[axis_index] = point.positions[goal_index];
        state->velocities[axis_index] = StateValueOrZero(point.velocities, goal_index);
        state->accelerations[axis_index] =
            StateValueOrZero(point.accelerations, goal_index);
    }
    return true;
}

double HermitePosition(double p0, double v0, double p1, double v1,
                       double duration, double s) {
    const double s2 = s * s;
    const double s3 = s2 * s;
    const double h00 = 2.0 * s3 - 3.0 * s2 + 1.0;
    const double h10 = s3 - 2.0 * s2 + s;
    const double h01 = -2.0 * s3 + 3.0 * s2;
    const double h11 = s3 - s2;
    return h00 * p0 + h10 * duration * v0 + h01 * p1 + h11 * duration * v1;
}

double HermiteVelocity(double p0, double v0, double p1, double v1,
                       double duration, double s) {
    const double s2 = s * s;
    return ((6.0 * s2 - 6.0 * s) / duration) * p0 +
           (3.0 * s2 - 4.0 * s + 1.0) * v0 +
           ((-6.0 * s2 + 6.0 * s) / duration) * p1 +
           (3.0 * s2 - 2.0 * s) * v1;
}

double HermiteAcceleration(double p0, double v0, double p1, double v1,
                           double duration, double s) {
    return ((12.0 * s - 6.0) / (duration * duration)) * p0 +
           ((6.0 * s - 4.0) / duration) * v0 +
           ((-12.0 * s + 6.0) / (duration * duration)) * p1 +
           ((6.0 * s - 2.0) / duration) * v1;
}

void CopyState(const HybridJointTargetExecutor::State& from,
               HybridJointTargetExecutor::State* to) {
    if (to == nullptr) {
        return;
    }
    *to = from;
}

}  // namespace

bool BuildGoalToConfigIndices(const std::vector<std::string>& goal_joint_names,
                              const std::vector<std::string>& config_joint_names,
                              std::vector<std::size_t>* goal_to_config_indices,
                              std::string* error) {
    if (goal_to_config_indices == nullptr) {
        SetError(error, "goal_to_config_indices is null");
        return false;
    }

    goal_to_config_indices->assign(goal_joint_names.size(), 0);
    for (std::size_t goal_index = 0; goal_index < goal_joint_names.size();
         ++goal_index) {
        const auto it =
            std::find(config_joint_names.begin(), config_joint_names.end(),
                      goal_joint_names[goal_index]);
        if (it == config_joint_names.end()) {
            SetError(error, "goal contains unknown joint: " + goal_joint_names[goal_index]);
            return false;
        }
        (*goal_to_config_indices)[goal_index] =
            static_cast<std::size_t>(std::distance(config_joint_names.begin(), it));
    }
    return true;
}

bool SampleTrajectoryStateAtTime(
    const control_msgs::FollowJointTrajectoryGoal& goal,
    const std::vector<std::size_t>& goal_to_config_indices,
    const HybridJointTargetExecutor::State& start_state,
    double time_from_start_sec,
    HybridTrajectorySample* sample,
    std::string* error) {
    if (sample == nullptr) {
        SetError(error, "trajectory sample output is null");
        return false;
    }
    if (!std::isfinite(time_from_start_sec)) {
        SetError(error, "time_from_start_sec must be finite");
        return false;
    }
    if (goal.trajectory.points.empty()) {
        SetError(error, "trajectory contains no points");
        return false;
    }
    if (goal_to_config_indices.size() != goal.trajectory.joint_names.size()) {
        SetError(error, "goal_to_config_indices size mismatch");
        return false;
    }
    const std::size_t dofs = start_state.positions.size();
    if (dofs == 0) {
        SetError(error, "start_state.positions must not be empty");
        return false;
    }

    HybridJointTargetExecutor::State start = start_state;
    EnsureStateArrays(&start, dofs);

    for (std::size_t point_index = 0; point_index < goal.trajectory.points.size();
         ++point_index) {
        const double point_time =
            goal.trajectory.points[point_index].time_from_start.toSec();
        if (!std::isfinite(point_time) || point_time < 0.0) {
            SetError(error, "trajectory point time_from_start must be finite and >= 0");
            return false;
        }
        if (point_index > 0) {
            const double previous_time =
                goal.trajectory.points[point_index - 1].time_from_start.toSec();
            if (point_time < previous_time) {
                SetError(error, "trajectory point time_from_start must be nondecreasing");
                return false;
            }
        }
    }

    const double clamped_time =
        std::max(0.0, std::min(time_from_start_sec,
                               goal.trajectory.points.back().time_from_start.toSec()));
    sample->sample_time_from_start_sec = clamped_time;
    sample->clamped_to_final_point =
        (clamped_time >= goal.trajectory.points.back().time_from_start.toSec());

    HybridJointTargetExecutor::State lower_state;
    HybridJointTargetExecutor::State upper_state;
    double lower_time = 0.0;
    double upper_time = 0.0;

    if (sample->clamped_to_final_point) {
        const std::size_t final_index = goal.trajectory.points.size() - 1;
        if (!FillStateFromPoint(goal.trajectory.points[final_index],
                                goal_to_config_indices,
                                dofs,
                                &sample->state,
                                error)) {
            return false;
        }
        sample->lower_waypoint_index = final_index;
        sample->upper_waypoint_index = final_index;
        return true;
    }

    std::size_t upper_index = 0;
    while (upper_index + 1 < goal.trajectory.points.size() &&
           goal.trajectory.points[upper_index].time_from_start.toSec() < clamped_time) {
        ++upper_index;
    }

    if (clamped_time <= goal.trajectory.points.front().time_from_start.toSec()) {
        sample->lower_waypoint_index = 0;
        sample->upper_waypoint_index = 0;
        CopyState(start, &lower_state);
        lower_time = 0.0;
        if (!FillStateFromPoint(goal.trajectory.points.front(),
                                goal_to_config_indices,
                                dofs,
                                &upper_state,
                                error)) {
            return false;
        }
        upper_time = goal.trajectory.points.front().time_from_start.toSec();
    } else {
        sample->upper_waypoint_index = upper_index;
        sample->lower_waypoint_index = upper_index - 1;
        if (!FillStateFromPoint(goal.trajectory.points[sample->lower_waypoint_index],
                                goal_to_config_indices,
                                dofs,
                                &lower_state,
                                error) ||
            !FillStateFromPoint(goal.trajectory.points[sample->upper_waypoint_index],
                                goal_to_config_indices,
                                dofs,
                                &upper_state,
                                error)) {
            return false;
        }
        lower_time =
            goal.trajectory.points[sample->lower_waypoint_index].time_from_start.toSec();
        upper_time =
            goal.trajectory.points[sample->upper_waypoint_index].time_from_start.toSec();
    }

    EnsureStateArrays(&sample->state, dofs);
    const double duration = upper_time - lower_time;
    if (duration <= 1e-9) {
        sample->state = upper_state;
        return true;
    }

    const double s = std::max(0.0, std::min(1.0, (clamped_time - lower_time) / duration));
    for (std::size_t axis_index = 0; axis_index < dofs; ++axis_index) {
        const double p0 = lower_state.positions[axis_index];
        const double v0 = StateValueOrZero(lower_state.velocities, axis_index);
        const double p1 = upper_state.positions[axis_index];
        const double v1 = StateValueOrZero(upper_state.velocities, axis_index);

        sample->state.positions[axis_index] =
            HermitePosition(p0, v0, p1, v1, duration, s);
        sample->state.velocities[axis_index] =
            HermiteVelocity(p0, v0, p1, v1, duration, s);
        sample->state.accelerations[axis_index] =
            HermiteAcceleration(p0, v0, p1, v1, duration, s);
    }
    return true;
}

}  // namespace eyou_ros1_master
