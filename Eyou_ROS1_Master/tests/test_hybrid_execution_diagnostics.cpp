#include <gtest/gtest.h>

#include "Eyou_ROS1_Master/hybrid_execution_diagnostics.hpp"

namespace {

TEST(HybridExecutionDiagnostics, PreservesJointOrderAndSignedDeviation) {
    eyou_ros1_master::TrajectoryExecutionDiagnosticsData data;
    data.joint_names = {"joint_b", "joint_a"};
    data.active_source = "action";
    data.executor_status = "tracking";
    data.continuous_mode_state_source = "follow_internal_state";
    data.nominal_reference.positions = {1.0, 2.0};
    data.nominal_reference.velocities = {0.1, 0.2};
    data.nominal_reference.accelerations = {0.01, 0.02};
    data.ruckig_command.positions = {1.2, 1.7};
    data.ruckig_command.velocities = {0.15, 0.25};
    data.ruckig_command.accelerations = {0.03, 0.04};
    data.actual_state.positions = {1.1, 1.9};
    data.actual_state.velocities = {0.11, 0.21};
    data.actual_state.accelerations = {0.0, 0.0};
    data.elapsed_time_sec = 0.35;
    data.trajectory_duration_sec = 1.5;

    const auto message = eyou_ros1_master::BuildTrajectoryExecutionStateMessage(
        data, ros::Time(12.0));

    ASSERT_EQ(message.joint_names.size(), 2u);
    EXPECT_EQ(message.joint_names[0], "joint_b");
    EXPECT_EQ(message.joint_names[1], "joint_a");
    EXPECT_EQ(message.active_source, "action");
    EXPECT_EQ(message.executor_status, "tracking");
    EXPECT_EQ(message.continuous_mode_state_source, "follow_internal_state");
    EXPECT_DOUBLE_EQ(message.nominal_reference.positions[0], 1.0);
    EXPECT_DOUBLE_EQ(message.nominal_reference.positions[1], 2.0);
    EXPECT_DOUBLE_EQ(message.ruckig_command.positions[0], 1.2);
    EXPECT_DOUBLE_EQ(message.ruckig_command.positions[1], 1.7);
    EXPECT_DOUBLE_EQ(message.actual_state.positions[0], 1.1);
    EXPECT_DOUBLE_EQ(message.actual_state.positions[1], 1.9);
    EXPECT_NEAR(message.tracking_error[0], -0.1, 1e-12);
    EXPECT_NEAR(message.tracking_error[1], 0.2, 1e-12);
    EXPECT_NEAR(message.planning_deviation[0], 0.2, 1e-12);
    EXPECT_NEAR(message.planning_deviation[1], -0.3, 1e-12);
    EXPECT_DOUBLE_EQ(message.elapsed_time, 0.35);
    EXPECT_DOUBLE_EQ(message.trajectory_duration, 1.5);
    EXPECT_DOUBLE_EQ(message.nominal_reference.time_from_start.toSec(), 0.35);
    EXPECT_DOUBLE_EQ(message.ruckig_command.time_from_start.toSec(), 0.35);
    EXPECT_DOUBLE_EQ(message.actual_state.time_from_start.toSec(), 0.35);
}

}  // namespace
