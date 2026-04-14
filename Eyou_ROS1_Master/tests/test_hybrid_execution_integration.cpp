#include <gtest/gtest.h>

#include <control_msgs/FollowJointTrajectoryGoal.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

#include "Eyou_ROS1_Master/hybrid_execution_diagnostics.hpp"
#include "Eyou_ROS1_Master/hybrid_follow_joint_trajectory_executor.hpp"
#include "Eyou_ROS1_Master/hybrid_servo_bridge.hpp"

namespace {

class FakeRobotHw : public hardware_interface::RobotHW {
public:
    FakeRobotHw() {
        joints_.reserve(2);
        RegisterJoint("joint_a");
        RegisterJoint("joint_b");
        registerInterface(&state_iface_);
        registerInterface(&pos_iface_);
    }

    void SetState(const std::string& name,
                  double pos,
                  double vel = 0.0,
                  double eff = 0.0) {
        auto& joint = joints_.at(IndexFor(name));
        joint.pos = pos;
        joint.vel = vel;
        joint.eff = eff;
    }

    void TrackCommand() {
        for (auto& joint : joints_) {
            joint.pos = joint.cmd;
            joint.vel = 0.0;
        }
    }

    double command(const std::string& name) const {
        return joints_.at(IndexFor(name)).cmd;
    }

private:
    struct JointData {
        std::string name;
        double pos{0.0};
        double vel{0.0};
        double eff{0.0};
        double cmd{0.0};
    };

    void RegisterJoint(const std::string& name) {
        joints_.push_back(JointData{name});
        auto& joint = joints_.back();
        hardware_interface::JointStateHandle state_handle(
            joint.name, &joint.pos, &joint.vel, &joint.eff);
        state_iface_.registerHandle(state_handle);
        hardware_interface::JointHandle pos_handle(
            state_iface_.getHandle(joint.name), &joint.cmd);
        pos_iface_.registerHandle(pos_handle);
    }

    std::size_t IndexFor(const std::string& name) const {
        for (std::size_t i = 0; i < joints_.size(); ++i) {
            if (joints_[i].name == name) {
                return i;
            }
        }
        throw std::out_of_range("unknown joint: " + name);
    }

    hardware_interface::JointStateInterface state_iface_;
    hardware_interface::PositionJointInterface pos_iface_;
    std::vector<JointData> joints_;
};

eyou_ros1_master::HybridJointTargetExecutor::Config MakeTargetConfig() {
    eyou_ros1_master::HybridJointTargetExecutor::Config config;
    config.joint_names = {"joint_a", "joint_b"};
    config.joint_indices = {0, 1};
    config.command_rate_hz = 100.0;
    config.max_velocities = {1.0, 1.0};
    config.max_accelerations = {2.0, 2.0};
    config.max_jerks = {10.0, 10.0};
    config.goal_tolerances = {1e-3, 1e-3};
    return config;
}

eyou_ros1_master::HybridFollowJointTrajectoryExecutor::Config MakeActionConfig() {
    eyou_ros1_master::HybridFollowJointTrajectoryExecutor::Config config;
    config.action_ns = "test_follow_joint_trajectory";
    config.joint_names = {"joint_a", "joint_b"};
    config.joint_indices = {0, 1};
    config.command_rate_hz = 100.0;
    config.max_velocities = {1.0, 1.0};
    config.max_accelerations = {2.0, 2.0};
    config.max_jerks = {10.0, 10.0};
    config.default_goal_tolerances = {1e-3, 1e-3};
    config.default_path_tolerances = {0.0, 0.0};
    return config;
}

eyou_ros1_master::HybridServoBridge::Config MakeBridgeConfig() {
    eyou_ros1_master::HybridServoBridge::Config config;
    config.joint_names = {"joint_a", "joint_b"};
    config.input_topic = "servo_joint_targets";
    config.timeout_sec = 0.1;
    return config;
}

trajectory_msgs::JointTrajectoryPoint MakePoint(
    const std::vector<double>& positions, double time_from_start_sec) {
    trajectory_msgs::JointTrajectoryPoint point;
    point.positions = positions;
    point.time_from_start = ros::Duration(time_from_start_sec);
    return point;
}

control_msgs::JointTolerance MakeTolerance(const std::string& name,
                                           double position,
                                           double velocity = 0.0,
                                           double acceleration = 0.0) {
    control_msgs::JointTolerance tolerance;
    tolerance.name = name;
    tolerance.position = position;
    tolerance.velocity = velocity;
    tolerance.acceleration = acceleration;
    return tolerance;
}

control_msgs::FollowJointTrajectoryGoal MakeGoal(
    const std::vector<std::string>& joint_names,
    const std::vector<trajectory_msgs::JointTrajectoryPoint>& points) {
    control_msgs::FollowJointTrajectoryGoal goal;
    goal.trajectory.joint_names = joint_names;
    goal.trajectory.points = points;
    return goal;
}

trajectory_msgs::JointTrajectory MakeTrajectory(
    const std::vector<std::string>& joint_names,
    const std::vector<double>& positions) {
    trajectory_msgs::JointTrajectory traj;
    traj.joint_names = joint_names;
    trajectory_msgs::JointTrajectoryPoint point;
    point.positions = positions;
    traj.points.push_back(point);
    return traj;
}

Eyou_ROS1_Master::TrajectoryExecutionState BuildDiagnosticsMessage(
    const eyou_ros1_master::HybridFollowJointTrajectoryExecutor& action_executor,
    const eyou_ros1_master::HybridJointTargetExecutor& target_executor,
    const ros::Time& stamp) {
    const auto diagnostic = action_executor.getDiagnosticState();
    eyou_ros1_master::TrajectoryExecutionDiagnosticsData data;
    data.joint_names = target_executor.getJointNames();
    if (const auto source = target_executor.active_source(); source.has_value()) {
        data.active_source = eyou_ros1_master::ToDiagnosticString(*source);
    } else {
        data.active_source = "none";
    }
    data.executor_status =
        eyou_ros1_master::ToDiagnosticString(target_executor.getExecutionStatus());
    data.continuous_mode_state_source = eyou_ros1_master::ToDiagnosticString(
        target_executor.getContinuousModeState());
    data.nominal_reference = diagnostic.nominal_reference.state;
    data.ruckig_command = target_executor.getCurrentCommand();
    data.actual_state = target_executor.getMeasuredState();
    data.elapsed_time_sec = diagnostic.nominal_reference.sample_time_from_start_sec;
    data.trajectory_duration_sec = diagnostic.trajectory_duration_sec;
    return eyou_ros1_master::BuildTrajectoryExecutionStateMessage(data, stamp);
}

}  // namespace

class HybridExecutionIntegrationTest : public ::testing::Test {
protected:
    static void SetUpTestSuite() {
        if (!ros::isInitialized()) {
            int argc = 0;
            char** argv = nullptr;
            ros::init(argc, argv, "test_hybrid_execution_integration",
                      ros::init_options::AnonymousName |
                          ros::init_options::NoSigintHandler |
                          ros::init_options::NoRosout);
        }
        ros::Time::init();
    }
};

TEST_F(HybridExecutionIntegrationTest,
       MultiWaypointActionUsesElapsedTimeSamplingAndCoherentDiagnostics) {
    FakeRobotHw hw;
    std::mutex loop_mtx;
    auto target_config = MakeTargetConfig();
    target_config.max_velocities = {20.0, 20.0};
    target_config.max_accelerations = {100.0, 100.0};
    target_config.max_jerks = {1000.0, 1000.0};
    eyou_ros1_master::HybridJointTargetExecutor target_executor(&hw, &loop_mtx,
                                                                target_config);
    ASSERT_TRUE(target_executor.valid()) << target_executor.config_error();

    auto action_config = MakeActionConfig();
    action_config.max_velocities = target_config.max_velocities;
    action_config.max_accelerations = target_config.max_accelerations;
    action_config.max_jerks = target_config.max_jerks;
    eyou_ros1_master::HybridFollowJointTrajectoryExecutor action_executor(
        nullptr, &hw, &loop_mtx, &target_executor, action_config);
    ASSERT_TRUE(action_executor.config_valid()) << action_executor.config_error();

    const auto goal = MakeGoal({"joint_a", "joint_b"},
                               {MakePoint({0.4, 0.6}, 0.4),
                                MakePoint({0.8, 1.0}, 0.8)});
    std::string error;
    ASSERT_TRUE(action_executor.startGoal(goal, &error)) << error;

    ros::Time sim_now(1.0);
    for (int step = 0; step < 60; ++step) {
        sim_now += ros::Duration(0.01);
        action_executor.update(sim_now, ros::Duration(0.01));
        target_executor.update(ros::Duration(0.01));
        hw.TrackCommand();
    }

    ASSERT_TRUE(action_executor.hasActiveGoal());
    const auto diagnostic = action_executor.getDiagnosticState();
    ASSERT_TRUE(diagnostic.has_nominal_reference);
    EXPECT_GT(diagnostic.nominal_reference.state.positions[0], 0.4);
    EXPECT_LT(diagnostic.nominal_reference.state.positions[0], 0.8);
    EXPECT_GT(diagnostic.nominal_reference.state.positions[1], 0.6);
    EXPECT_LT(diagnostic.nominal_reference.state.positions[1], 1.0);

    const auto message = BuildDiagnosticsMessage(action_executor, target_executor,
                                                 sim_now);
    const auto command = target_executor.getCurrentCommand();
    const auto actual = target_executor.getMeasuredState();
    ASSERT_EQ(message.joint_names.size(), 2u);
    EXPECT_EQ(message.joint_names[0], "joint_a");
    EXPECT_EQ(message.joint_names[1], "joint_b");
    EXPECT_EQ(message.active_source, "action");
    EXPECT_EQ(message.executor_status, "tracking");
    EXPECT_EQ(message.continuous_mode_state_source, "follow_internal_state");
    EXPECT_NEAR(message.tracking_error[0],
                actual.positions[0] - command.positions[0], 1e-12);
    EXPECT_NEAR(message.tracking_error[1],
                actual.positions[1] - command.positions[1], 1e-12);
    EXPECT_NEAR(message.planning_deviation[0],
                command.positions[0] -
                    diagnostic.nominal_reference.state.positions[0],
                1e-12);
    EXPECT_NEAR(message.planning_deviation[1],
                command.positions[1] -
                    diagnostic.nominal_reference.state.positions[1],
                1e-12);
    EXPECT_DOUBLE_EQ(message.elapsed_time,
                     diagnostic.nominal_reference.sample_time_from_start_sec);
    EXPECT_DOUBLE_EQ(message.trajectory_duration,
                     diagnostic.trajectory_duration_sec);
}

TEST_F(HybridExecutionIntegrationTest,
       PathToleranceAbortReleasesActionOwnershipForServoRecovery) {
    FakeRobotHw hw;
    std::mutex loop_mtx;
    eyou_ros1_master::HybridJointTargetExecutor target_executor(&hw, &loop_mtx,
                                                                MakeTargetConfig());
    ASSERT_TRUE(target_executor.valid()) << target_executor.config_error();

    eyou_ros1_master::HybridFollowJointTrajectoryExecutor action_executor(
        nullptr, &hw, &loop_mtx, &target_executor, MakeActionConfig());
    ASSERT_TRUE(action_executor.config_valid()) << action_executor.config_error();

    eyou_ros1_master::HybridServoBridge servo_bridge(nullptr, &target_executor,
                                                     MakeBridgeConfig());
    ASSERT_TRUE(servo_bridge.valid()) << servo_bridge.config_error();

    auto goal = MakeGoal({"joint_a", "joint_b"},
                         {MakePoint({0.5, 0.5}, 0.5)});
    goal.path_tolerance.push_back(MakeTolerance("joint_a", 0.02));
    goal.path_tolerance.push_back(MakeTolerance("joint_b", 0.02));

    std::string error;
    ASSERT_TRUE(action_executor.startGoal(goal, &error)) << error;

    ros::Time sim_now(1.0);
    for (int step = 0; step < 50 && action_executor.hasActiveGoal(); ++step) {
        sim_now += ros::Duration(0.01);
        action_executor.update(sim_now, ros::Duration(0.01));
        target_executor.update(ros::Duration(0.01));
    }

    EXPECT_FALSE(action_executor.hasActiveGoal());
    ASSERT_TRUE(action_executor.getLastTerminalResultCode().has_value());
    EXPECT_EQ(*action_executor.getLastTerminalResultCode(),
              control_msgs::FollowJointTrajectoryResult::
                  PATH_TOLERANCE_VIOLATED);
    EXPECT_FALSE(target_executor.active_source().has_value());

    ASSERT_TRUE(servo_bridge.acceptTrajectory(
        MakeTrajectory({"joint_a", "joint_b"}, {0.2, -0.1}), sim_now));
    servo_bridge.update(sim_now + ros::Duration(0.01));
    target_executor.update(ros::Duration(0.01));

    ASSERT_TRUE(target_executor.active_source().has_value());
    EXPECT_EQ(*target_executor.active_source(),
              eyou_ros1_master::HybridJointTargetExecutor::Source::kServo);
}

TEST_F(HybridExecutionIntegrationTest,
       ActionPreemptsServoAndServoReacquiresAfterGoalCompletion) {
    FakeRobotHw hw;
    std::mutex loop_mtx;
    auto target_config = MakeTargetConfig();
    target_config.max_velocities = {2.0, 2.0};
    target_config.max_accelerations = {4.0, 4.0};
    target_config.max_jerks = {20.0, 20.0};
    eyou_ros1_master::HybridJointTargetExecutor target_executor(&hw, &loop_mtx,
                                                                target_config);
    ASSERT_TRUE(target_executor.valid()) << target_executor.config_error();

    auto action_config = MakeActionConfig();
    action_config.max_velocities = target_config.max_velocities;
    action_config.max_accelerations = target_config.max_accelerations;
    action_config.max_jerks = target_config.max_jerks;
    eyou_ros1_master::HybridFollowJointTrajectoryExecutor action_executor(
        nullptr, &hw, &loop_mtx, &target_executor, action_config);
    ASSERT_TRUE(action_executor.config_valid()) << action_executor.config_error();

    eyou_ros1_master::HybridServoBridge servo_bridge(nullptr, &target_executor,
                                                     MakeBridgeConfig());
    ASSERT_TRUE(servo_bridge.valid()) << servo_bridge.config_error();

    ros::Time sim_now(1.0);
    ASSERT_TRUE(servo_bridge.acceptTrajectory(
        MakeTrajectory({"joint_a", "joint_b"}, {0.1, 0.1}), sim_now));
    servo_bridge.update(sim_now);
    target_executor.update(ros::Duration(0.01));
    hw.TrackCommand();

    ASSERT_TRUE(target_executor.active_source().has_value());
    EXPECT_EQ(*target_executor.active_source(),
              eyou_ros1_master::HybridJointTargetExecutor::Source::kServo);

    auto goal = MakeGoal({"joint_a", "joint_b"},
                         {MakePoint({0.5, 0.4}, 0.5)});
    goal.goal_time_tolerance = ros::Duration(1.0);
    std::string error;
    ASSERT_TRUE(action_executor.startGoal(goal, &error)) << error;

    sim_now += ros::Duration(0.01);
    action_executor.update(sim_now, ros::Duration(0.01));
    target_executor.update(ros::Duration(0.01));
    hw.TrackCommand();

    ASSERT_TRUE(target_executor.active_source().has_value());
    EXPECT_EQ(*target_executor.active_source(),
              eyou_ros1_master::HybridJointTargetExecutor::Source::kAction);

    ASSERT_TRUE(servo_bridge.acceptTrajectory(
        MakeTrajectory({"joint_a", "joint_b"}, {-0.2, -0.3}),
        sim_now + ros::Duration(0.01)));
    servo_bridge.update(sim_now + ros::Duration(0.01));
    ASSERT_TRUE(target_executor.active_source().has_value());
    EXPECT_EQ(*target_executor.active_source(),
              eyou_ros1_master::HybridJointTargetExecutor::Source::kAction);

    for (int step = 0; step < 200 && action_executor.hasActiveGoal(); ++step) {
        sim_now += ros::Duration(0.01);
        action_executor.update(sim_now, ros::Duration(0.01));
        target_executor.update(ros::Duration(0.01));
        hw.TrackCommand();
    }

    EXPECT_FALSE(action_executor.hasActiveGoal());
    ASSERT_TRUE(action_executor.getLastTerminalResultCode().has_value());
    EXPECT_EQ(*action_executor.getLastTerminalResultCode(),
              control_msgs::FollowJointTrajectoryResult::SUCCESSFUL);
    EXPECT_FALSE(target_executor.active_source().has_value());

    servo_bridge.update(sim_now + ros::Duration(0.02));
    target_executor.update(ros::Duration(0.01));

    ASSERT_TRUE(target_executor.active_source().has_value());
    EXPECT_EQ(*target_executor.active_source(),
              eyou_ros1_master::HybridJointTargetExecutor::Source::kServo);
}

TEST_F(HybridExecutionIntegrationTest,
       GoalTimeToleranceAllowsLateSuccessAcrossIntegratedExecutors) {
    FakeRobotHw hw;
    std::mutex loop_mtx;
    auto target_config = MakeTargetConfig();
    target_config.max_velocities = {0.5, 0.5};
    target_config.max_accelerations = {2.0, 2.0};
    target_config.max_jerks = {20.0, 20.0};
    eyou_ros1_master::HybridJointTargetExecutor target_executor(&hw, &loop_mtx,
                                                                target_config);
    ASSERT_TRUE(target_executor.valid()) << target_executor.config_error();

    auto action_config = MakeActionConfig();
    action_config.max_velocities = target_config.max_velocities;
    action_config.max_accelerations = target_config.max_accelerations;
    action_config.max_jerks = target_config.max_jerks;
    eyou_ros1_master::HybridFollowJointTrajectoryExecutor action_executor(
        nullptr, &hw, &loop_mtx, &target_executor, action_config);
    ASSERT_TRUE(action_executor.config_valid()) << action_executor.config_error();

    auto goal = MakeGoal({"joint_a", "joint_b"},
                         {MakePoint({0.2, 0.2}, 0.1)});
    goal.goal_time_tolerance = ros::Duration(1.0);

    std::string error;
    ASSERT_TRUE(action_executor.startGoal(goal, &error)) << error;

    ros::Time sim_now(1.0);
    for (int step = 0; step < 15; ++step) {
        sim_now += ros::Duration(0.01);
        action_executor.update(sim_now, ros::Duration(0.01));
        target_executor.update(ros::Duration(0.01));
        hw.TrackCommand();
    }

    EXPECT_TRUE(action_executor.hasActiveGoal());
    const auto diagnostic = action_executor.getDiagnosticState();
    ASSERT_TRUE(diagnostic.has_nominal_reference);
    EXPECT_DOUBLE_EQ(diagnostic.trajectory_duration_sec, 0.1);
    EXPECT_DOUBLE_EQ(diagnostic.nominal_reference.sample_time_from_start_sec, 0.1);

    for (int step = 0; step < 200 && action_executor.hasActiveGoal(); ++step) {
        sim_now += ros::Duration(0.01);
        action_executor.update(sim_now, ros::Duration(0.01));
        target_executor.update(ros::Duration(0.01));
        hw.TrackCommand();
    }

    EXPECT_FALSE(action_executor.hasActiveGoal());
    ASSERT_TRUE(action_executor.getLastTerminalResultCode().has_value());
    EXPECT_EQ(*action_executor.getLastTerminalResultCode(),
              control_msgs::FollowJointTrajectoryResult::SUCCESSFUL);
    EXPECT_FALSE(target_executor.active_source().has_value());
}
