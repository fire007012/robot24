#include <gtest/gtest.h>

#include <control_msgs/FollowJointTrajectoryGoal.h>

#include "Eyou_ROS1_Master/hybrid_tolerance_resolver.hpp"

namespace {

control_msgs::JointTolerance MakeTolerance(const std::string& name,
                                           double position,
                                           double velocity,
                                           double acceleration) {
    control_msgs::JointTolerance tolerance;
    tolerance.name = name;
    tolerance.position = position;
    tolerance.velocity = velocity;
    tolerance.acceleration = acceleration;
    return tolerance;
}

control_msgs::FollowJointTrajectoryGoal MakeGoal() {
    control_msgs::FollowJointTrajectoryGoal goal;
    goal.trajectory.joint_names = {"joint_a", "joint_b"};
    return goal;
}

}  // namespace

TEST(HybridToleranceResolver, UsesDefaultsWhenGoalDoesNotOverride) {
    const auto goal = MakeGoal();
    const std::vector<std::string> joint_names = {"joint_a", "joint_b"};
    const std::vector<double> default_path_tolerances = {0.0, 0.0};
    const std::vector<double> default_goal_tolerances = {0.1, 0.2};
    eyou_ros1_master::FollowJointTrajectoryResolvedTolerances resolved;
    std::string error;
    ASSERT_TRUE(eyou_ros1_master::ResolveFollowJointTrajectoryTolerances(
        goal, joint_names, default_path_tolerances, default_goal_tolerances,
        0.01, 0.5, &resolved, &error))
        << error;

    ASSERT_EQ(resolved.path_state_tolerance.size(), 2u);
    ASSERT_EQ(resolved.goal_state_tolerance.size(), 2u);
    EXPECT_DOUBLE_EQ(resolved.path_state_tolerance[0].position, 0.0);
    EXPECT_DOUBLE_EQ(resolved.path_state_tolerance[1].position, 0.0);
    EXPECT_DOUBLE_EQ(resolved.goal_state_tolerance[0].position, 0.1);
    EXPECT_DOUBLE_EQ(resolved.goal_state_tolerance[1].position, 0.2);
    EXPECT_DOUBLE_EQ(resolved.goal_state_tolerance[0].velocity, 0.01);
    EXPECT_DOUBLE_EQ(resolved.goal_state_tolerance[1].velocity, 0.01);
    EXPECT_DOUBLE_EQ(resolved.goal_time_tolerance, 0.5);
}

TEST(HybridToleranceResolver, PositiveValuesOverrideMatchingJoints) {
    auto goal = MakeGoal();
    const std::vector<std::string> joint_names = {"joint_a", "joint_b"};
    const std::vector<double> default_path_tolerances = {0.0, 0.0};
    const std::vector<double> default_goal_tolerances = {0.1, 0.2};
    goal.path_tolerance.push_back(MakeTolerance("joint_b", 0.4, 0.3, 0.2));
    goal.goal_tolerance.push_back(MakeTolerance("joint_a", 0.8, 0.7, 0.6));
    goal.goal_tolerance.push_back(MakeTolerance("joint_b", 0.9, 0.5, 0.4));
    goal.goal_time_tolerance = ros::Duration(1.25);

    eyou_ros1_master::FollowJointTrajectoryResolvedTolerances resolved;
    std::string error;
    ASSERT_TRUE(eyou_ros1_master::ResolveFollowJointTrajectoryTolerances(
        goal, joint_names, default_path_tolerances, default_goal_tolerances,
        0.01, 0.5, &resolved, &error))
        << error;

    EXPECT_DOUBLE_EQ(resolved.path_state_tolerance[0].position, 0.0);
    EXPECT_DOUBLE_EQ(resolved.path_state_tolerance[1].position, 0.4);
    EXPECT_DOUBLE_EQ(resolved.path_state_tolerance[1].velocity, 0.3);
    EXPECT_DOUBLE_EQ(resolved.path_state_tolerance[1].acceleration, 0.2);

    EXPECT_DOUBLE_EQ(resolved.goal_state_tolerance[0].position, 0.8);
    EXPECT_DOUBLE_EQ(resolved.goal_state_tolerance[0].velocity, 0.7);
    EXPECT_DOUBLE_EQ(resolved.goal_state_tolerance[0].acceleration, 0.6);
    EXPECT_DOUBLE_EQ(resolved.goal_state_tolerance[1].position, 0.9);
    EXPECT_DOUBLE_EQ(resolved.goal_state_tolerance[1].velocity, 0.5);
    EXPECT_DOUBLE_EQ(resolved.goal_state_tolerance[1].acceleration, 0.4);
    EXPECT_DOUBLE_EQ(resolved.goal_time_tolerance, 1.25);
}

TEST(HybridToleranceResolver, ZeroValuesKeepDefaultsAndNegativeValuesErase) {
    auto goal = MakeGoal();
    const std::vector<std::string> joint_names = {"joint_a", "joint_b"};
    const std::vector<double> default_path_tolerances = {0.0, 0.2};
    const std::vector<double> default_goal_tolerances = {0.1, 0.3};
    goal.goal_tolerance.push_back(MakeTolerance("joint_a", -1.0, 0.0, -1.0));
    goal.path_tolerance.push_back(MakeTolerance("joint_a", 0.0, -1.0, 0.0));
    goal.goal_time_tolerance = ros::Duration(-1.0);

    eyou_ros1_master::FollowJointTrajectoryResolvedTolerances resolved;
    std::string error;
    ASSERT_TRUE(eyou_ros1_master::ResolveFollowJointTrajectoryTolerances(
        goal, joint_names, default_path_tolerances, default_goal_tolerances,
        0.02, 0.5, &resolved, &error))
        << error;

    EXPECT_DOUBLE_EQ(resolved.goal_state_tolerance[0].position, 0.0);
    EXPECT_DOUBLE_EQ(resolved.goal_state_tolerance[0].velocity, 0.02);
    EXPECT_DOUBLE_EQ(resolved.goal_state_tolerance[0].acceleration, 0.0);
    EXPECT_DOUBLE_EQ(resolved.path_state_tolerance[0].position, 0.0);
    EXPECT_DOUBLE_EQ(resolved.path_state_tolerance[0].velocity, 0.0);
    EXPECT_DOUBLE_EQ(resolved.path_state_tolerance[1].position, 0.2);
    EXPECT_DOUBLE_EQ(resolved.goal_time_tolerance, 0.0);
}

TEST(HybridToleranceResolver, IgnoresUnknownToleranceEntries) {
    auto goal = MakeGoal();
    const std::vector<std::string> joint_names = {"joint_a", "joint_b"};
    const std::vector<double> default_path_tolerances = {0.0, 0.0};
    const std::vector<double> default_goal_tolerances = {0.1, 0.2};
    goal.goal_tolerance.push_back(MakeTolerance("joint_x", 0.8, 0.7, 0.6));
    goal.path_tolerance.push_back(MakeTolerance("joint_y", 0.5, 0.4, 0.3));

    eyou_ros1_master::FollowJointTrajectoryResolvedTolerances resolved;
    std::string error;
    ASSERT_TRUE(eyou_ros1_master::ResolveFollowJointTrajectoryTolerances(
        goal, joint_names, default_path_tolerances, default_goal_tolerances,
        0.01, 0.5, &resolved, &error))
        << error;

    EXPECT_DOUBLE_EQ(resolved.goal_state_tolerance[0].position, 0.1);
    EXPECT_DOUBLE_EQ(resolved.goal_state_tolerance[1].position, 0.2);
    EXPECT_DOUBLE_EQ(resolved.path_state_tolerance[0].position, 0.0);
    EXPECT_DOUBLE_EQ(resolved.path_state_tolerance[1].position, 0.0);
}

TEST(HybridToleranceResolver,
     AppliesRepeatedJointOverridesSequentiallyWithJtcSemantics) {
    auto goal = MakeGoal();
    const std::vector<std::string> joint_names = {"joint_a", "joint_b"};
    const std::vector<double> default_path_tolerances = {0.0, 0.0};
    const std::vector<double> default_goal_tolerances = {0.1, 0.2};
    goal.goal_tolerance.push_back(MakeTolerance("joint_a", 0.4, 0.5, 0.0));
    goal.goal_tolerance.push_back(MakeTolerance("joint_a", 0.0, -1.0, 0.3));

    eyou_ros1_master::FollowJointTrajectoryResolvedTolerances resolved;
    std::string error;
    ASSERT_TRUE(eyou_ros1_master::ResolveFollowJointTrajectoryTolerances(
        goal, joint_names, default_path_tolerances, default_goal_tolerances,
        0.01, 0.5, &resolved, &error))
        << error;

    EXPECT_DOUBLE_EQ(resolved.goal_state_tolerance[0].position, 0.4);
    EXPECT_DOUBLE_EQ(resolved.goal_state_tolerance[0].velocity, 0.0);
    EXPECT_DOUBLE_EQ(resolved.goal_state_tolerance[0].acceleration, 0.3);
    EXPECT_DOUBLE_EQ(resolved.goal_state_tolerance[1].position, 0.2);
    EXPECT_DOUBLE_EQ(resolved.goal_state_tolerance[1].velocity, 0.01);
    EXPECT_DOUBLE_EQ(resolved.goal_state_tolerance[1].acceleration, 0.0);
}

TEST(HybridToleranceResolver, RejectsInvalidDefaultVectors) {
    const auto goal = MakeGoal();
    const std::vector<std::string> joint_names = {"joint_a", "joint_b"};
    const std::vector<double> default_path_tolerances = {0.0};
    const std::vector<double> default_goal_tolerances = {0.1, 0.2};
    eyou_ros1_master::FollowJointTrajectoryResolvedTolerances resolved;
    std::string error;
    EXPECT_FALSE(eyou_ros1_master::ResolveFollowJointTrajectoryTolerances(
        goal, joint_names, default_path_tolerances, default_goal_tolerances,
        0.01, 0.5, &resolved, &error));
    EXPECT_NE(error.find("default_path_position_tolerances size mismatch"),
              std::string::npos);
}
