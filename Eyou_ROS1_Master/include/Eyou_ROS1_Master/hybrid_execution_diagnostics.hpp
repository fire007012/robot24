#pragma once

#include <string>
#include <vector>

#include <ros/time.h>

#include "Eyou_ROS1_Master/TrajectoryExecutionState.h"
#include "Eyou_ROS1_Master/hybrid_joint_target_executor.hpp"

namespace eyou_ros1_master {

struct TrajectoryExecutionDiagnosticsData {
    std::vector<std::string> joint_names;
    std::string active_source;
    std::string executor_status;
    std::string continuous_mode_state_source;
    HybridJointTargetExecutor::State nominal_reference;
    HybridJointTargetExecutor::State ruckig_command;
    HybridJointTargetExecutor::State actual_state;
    double elapsed_time_sec{0.0};
    double trajectory_duration_sec{0.0};
};

std::string ToDiagnosticString(HybridJointTargetExecutor::Source source);
std::string ToDiagnosticString(HybridJointTargetExecutor::ExecutionStatus status);
std::string ToDiagnosticString(HybridJointTargetExecutor::ContinuousModeState state);

Eyou_ROS1_Master::TrajectoryExecutionState BuildTrajectoryExecutionStateMessage(
    const TrajectoryExecutionDiagnosticsData& data,
    const ros::Time& stamp);

}  // namespace eyou_ros1_master
