#pragma once

#include <cstddef>
#include <string>
#include <vector>

#include <control_msgs/FollowJointTrajectoryAction.h>

#include "Eyou_ROS1_Master/hybrid_joint_target_executor.hpp"

namespace eyou_ros1_master {

struct HybridTrajectorySample {
    HybridJointTargetExecutor::State state;
    std::size_t lower_waypoint_index{0};
    std::size_t upper_waypoint_index{0};
    double sample_time_from_start_sec{0.0};
    bool clamped_to_final_point{false};
};

bool BuildGoalToConfigIndices(const std::vector<std::string>& goal_joint_names,
                              const std::vector<std::string>& config_joint_names,
                              std::vector<std::size_t>* goal_to_config_indices,
                              std::string* error);

bool SampleTrajectoryStateAtTime(
    const control_msgs::FollowJointTrajectoryGoal& goal,
    const std::vector<std::size_t>& goal_to_config_indices,
    const HybridJointTargetExecutor::State& start_state,
    double time_from_start_sec,
    HybridTrajectorySample* sample,
    std::string* error);

}  // namespace eyou_ros1_master
