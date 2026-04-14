#include "Eyou_ROS1_Master/hybrid_execution_diagnostics.hpp"

#include <algorithm>

namespace eyou_ros1_master {

namespace {

void ResizeState(HybridJointTargetExecutor::State* state, std::size_t dofs) {
    if (state == nullptr) {
        return;
    }
    state->positions.resize(dofs, 0.0);
    state->velocities.resize(dofs, 0.0);
    state->accelerations.resize(dofs, 0.0);
}

trajectory_msgs::JointTrajectoryPoint ToPoint(
    HybridJointTargetExecutor::State state,
    std::size_t dofs,
    double time_from_start_sec) {
    ResizeState(&state, dofs);

    trajectory_msgs::JointTrajectoryPoint point;
    point.positions = std::move(state.positions);
    point.velocities = std::move(state.velocities);
    point.accelerations = std::move(state.accelerations);
    point.time_from_start = ros::Duration(time_from_start_sec);
    return point;
}

}  // namespace

std::string ToDiagnosticString(HybridJointTargetExecutor::Source source) {
    switch (source) {
        case HybridJointTargetExecutor::Source::kAction:
            return "action";
        case HybridJointTargetExecutor::Source::kServo:
            return "servo";
    }
    return "unknown";
}

std::string ToDiagnosticString(HybridJointTargetExecutor::ExecutionStatus status) {
    switch (status) {
        case HybridJointTargetExecutor::ExecutionStatus::kHold:
            return "hold";
        case HybridJointTargetExecutor::ExecutionStatus::kTracking:
            return "tracking";
        case HybridJointTargetExecutor::ExecutionStatus::kFinished:
            return "finished";
        case HybridJointTargetExecutor::ExecutionStatus::kTrackingFault:
            return "tracking_fault";
        case HybridJointTargetExecutor::ExecutionStatus::kResyncing:
            return "resyncing";
        case HybridJointTargetExecutor::ExecutionStatus::kError:
            return "error";
    }
    return "unknown";
}

std::string ToDiagnosticString(HybridJointTargetExecutor::ContinuousModeState state) {
    switch (state) {
        case HybridJointTargetExecutor::ContinuousModeState::kInactive:
            return "inactive";
        case HybridJointTargetExecutor::ContinuousModeState::kInitFromHardware:
            return "init_from_hardware";
        case HybridJointTargetExecutor::ContinuousModeState::kFollowInternalState:
            return "follow_internal_state";
        case HybridJointTargetExecutor::ContinuousModeState::kResyncFromHardware:
            return "resync_from_hardware";
    }
    return "unknown";
}

Eyou_ROS1_Master::TrajectoryExecutionState BuildTrajectoryExecutionStateMessage(
    const TrajectoryExecutionDiagnosticsData& data,
    const ros::Time& stamp) {
    const std::size_t dofs = data.joint_names.size();

    HybridJointTargetExecutor::State nominal_reference = data.nominal_reference;
    HybridJointTargetExecutor::State ruckig_command = data.ruckig_command;
    HybridJointTargetExecutor::State actual_state = data.actual_state;
    ResizeState(&nominal_reference, dofs);
    ResizeState(&ruckig_command, dofs);
    ResizeState(&actual_state, dofs);

    Eyou_ROS1_Master::TrajectoryExecutionState message;
    message.header.stamp = stamp;
    message.joint_names = data.joint_names;
    message.active_source = data.active_source;
    message.executor_status = data.executor_status;
    message.continuous_mode_state_source = data.continuous_mode_state_source;
    message.nominal_reference =
        ToPoint(std::move(nominal_reference), dofs, data.elapsed_time_sec);
    message.ruckig_command =
        ToPoint(std::move(ruckig_command), dofs, data.elapsed_time_sec);
    message.actual_state = ToPoint(std::move(actual_state), dofs, data.elapsed_time_sec);
    message.elapsed_time = data.elapsed_time_sec;
    message.trajectory_duration = data.trajectory_duration_sec;
    message.tracking_error.resize(dofs, 0.0);
    message.planning_deviation.resize(dofs, 0.0);

    for (std::size_t axis_index = 0; axis_index < dofs; ++axis_index) {
        message.tracking_error[axis_index] =
            message.actual_state.positions[axis_index] -
            message.ruckig_command.positions[axis_index];
        message.planning_deviation[axis_index] =
            message.ruckig_command.positions[axis_index] -
            message.nominal_reference.positions[axis_index];
    }
    return message;
}

}  // namespace eyou_ros1_master
