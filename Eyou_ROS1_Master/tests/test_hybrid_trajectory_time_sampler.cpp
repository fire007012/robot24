#include <gtest/gtest.h>

#include <trajectory_msgs/JointTrajectoryPoint.h>

#include "Eyou_ROS1_Master/hybrid_trajectory_time_sampler.hpp"

namespace {

using eyou_ros1_master::BuildGoalToConfigIndices;
using eyou_ros1_master::HybridJointTargetExecutor;
using eyou_ros1_master::HybridTrajectorySample;
using eyou_ros1_master::SampleTrajectoryStateAtTime;

trajectory_msgs::JointTrajectoryPoint MakePoint(const std::vector<double>& positions,
                                                double time_from_start_sec,
                                                const std::vector<double>& velocities = {}) {
    trajectory_msgs::JointTrajectoryPoint point;
    point.positions = positions;
    point.velocities = velocities;
    point.time_from_start = ros::Duration(time_from_start_sec);
    return point;
}

control_msgs::FollowJointTrajectoryGoal MakeGoal(
    const std::vector<std::string>& joint_names,
    const std::vector<trajectory_msgs::JointTrajectoryPoint>& points) {
    control_msgs::FollowJointTrajectoryGoal goal;
    goal.trajectory.joint_names = joint_names;
    goal.trajectory.points = points;
    return goal;
}

HybridJointTargetExecutor::State MakeStartState() {
    HybridJointTargetExecutor::State state;
    state.positions = {0.0, 0.0};
    state.velocities = {0.0, 0.0};
    state.accelerations = {0.0, 0.0};
    return state;
}

}  // namespace

TEST(HybridTrajectoryTimeSampler, BuildsGoalToConfigIndicesForReorderedGoalJoints) {
    std::vector<std::size_t> indices;
    std::string error;
    ASSERT_TRUE(BuildGoalToConfigIndices({"joint_b", "joint_a"},
                                         {"joint_a", "joint_b"},
                                         &indices,
                                         &error))
        << error;
    ASSERT_EQ(indices.size(), 2u);
    EXPECT_EQ(indices[0], 1u);
    EXPECT_EQ(indices[1], 0u);
}

TEST(HybridTrajectoryTimeSampler, SamplesBetweenStartAndFirstWaypointByAbsoluteTime) {
    const auto goal = MakeGoal({"joint_a", "joint_b"},
                               {MakePoint({1.0, -0.5}, 1.0)});
    const auto start_state = MakeStartState();
    std::vector<std::size_t> indices{0, 1};
    HybridTrajectorySample sample;
    std::string error;

    ASSERT_TRUE(SampleTrajectoryStateAtTime(goal, indices, start_state, 0.5, &sample, &error))
        << error;

    EXPECT_NEAR(sample.state.positions[0], 0.5, 1e-6);
    EXPECT_NEAR(sample.state.positions[1], -0.25, 1e-6);
    EXPECT_EQ(sample.lower_waypoint_index, 0u);
    EXPECT_EQ(sample.upper_waypoint_index, 0u);
    EXPECT_FALSE(sample.clamped_to_final_point);
}

TEST(HybridTrajectoryTimeSampler, SamplesBetweenWaypointsUsingAbsoluteTrajectoryTime) {
    const auto goal = MakeGoal({"joint_a", "joint_b"},
                               {MakePoint({0.4, -0.2}, 0.4),
                                MakePoint({0.8, -0.6}, 0.8)});
    const auto start_state = MakeStartState();
    std::vector<std::size_t> indices{0, 1};
    HybridTrajectorySample sample;
    std::string error;

    ASSERT_TRUE(SampleTrajectoryStateAtTime(goal, indices, start_state, 0.6, &sample, &error))
        << error;

    EXPECT_NEAR(sample.state.positions[0], 0.6, 1e-6);
    EXPECT_NEAR(sample.state.positions[1], -0.4, 1e-6);
    EXPECT_EQ(sample.lower_waypoint_index, 0u);
    EXPECT_EQ(sample.upper_waypoint_index, 1u);
    EXPECT_FALSE(sample.clamped_to_final_point);
}

TEST(HybridTrajectoryTimeSampler, ClampsToFinalWaypointAfterTrajectoryEnd) {
    const auto goal = MakeGoal({"joint_a", "joint_b"},
                               {MakePoint({0.4, -0.2}, 0.4),
                                MakePoint({0.8, -0.6}, 0.8)});
    const auto start_state = MakeStartState();
    std::vector<std::size_t> indices{0, 1};
    HybridTrajectorySample sample;
    std::string error;

    ASSERT_TRUE(SampleTrajectoryStateAtTime(goal, indices, start_state, 2.0, &sample, &error))
        << error;

    EXPECT_NEAR(sample.state.positions[0], 0.8, 1e-9);
    EXPECT_NEAR(sample.state.positions[1], -0.6, 1e-9);
    EXPECT_EQ(sample.lower_waypoint_index, 1u);
    EXPECT_EQ(sample.upper_waypoint_index, 1u);
    EXPECT_TRUE(sample.clamped_to_final_point);
}
