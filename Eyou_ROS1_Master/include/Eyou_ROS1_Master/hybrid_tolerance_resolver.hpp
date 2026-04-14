#pragma once

#include <string>
#include <vector>

#include <control_msgs/FollowJointTrajectoryGoal.h>

namespace eyou_ros1_master {

struct JointStateTolerance {
    double position{0.0};
    double velocity{0.0};
    double acceleration{0.0};
};

struct FollowJointTrajectoryResolvedTolerances {
    std::vector<JointStateTolerance> path_state_tolerance;
    std::vector<JointStateTolerance> goal_state_tolerance;
    double goal_time_tolerance{0.0};
};

bool ResolveFollowJointTrajectoryTolerances(
    const control_msgs::FollowJointTrajectoryGoal& goal,
    const std::vector<std::string>& joint_names,
    const std::vector<double>& default_path_position_tolerances,
    const std::vector<double>& default_goal_position_tolerances,
    double default_stopped_velocity_tolerance,
    double default_goal_time_tolerance,
    FollowJointTrajectoryResolvedTolerances* resolved,
    std::string* error);

}  // namespace eyou_ros1_master
