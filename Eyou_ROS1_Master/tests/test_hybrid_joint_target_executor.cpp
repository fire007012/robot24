#include <gtest/gtest.h>

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/ros.h>

#include "Eyou_ROS1_Master/hybrid_joint_target_executor.hpp"

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

    void SetState(const std::string& name, double pos, double vel = 0.0, double eff = 0.0) {
        auto& joint = joints_.at(IndexFor(name));
        joint.pos = pos;
        joint.vel = vel;
        joint.eff = eff;
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

eyou_ros1_master::HybridJointTargetExecutor::Config MakeConfig() {
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

}  // namespace

class HybridJointTargetExecutorTest : public ::testing::Test {
protected:
    static void SetUpTestSuite() {
        if (!ros::isInitialized()) {
            int argc = 0;
            char** argv = nullptr;
            ros::init(argc, argv, "test_hybrid_joint_target_executor",
                      ros::init_options::AnonymousName |
                          ros::init_options::NoSigintHandler |
                          ros::init_options::NoRosout);
        }
        ros::Time::init();
    }
};

TEST_F(HybridJointTargetExecutorTest, RejectsDuplicateJointNamesInConfig) {
    FakeRobotHw hw;
    std::mutex loop_mtx;
    auto config = MakeConfig();
    config.joint_names[1] = "joint_a";

    eyou_ros1_master::HybridJointTargetExecutor executor(&hw, &loop_mtx, config);
    EXPECT_FALSE(executor.valid());
    EXPECT_NE(executor.config_error().find("duplicate joint"),
              std::string::npos);
}

TEST_F(HybridJointTargetExecutorTest,
       RejectsConfigWhenTrackingFaultThresholdDoesNotExceedResyncThreshold) {
    FakeRobotHw hw;
    std::mutex loop_mtx;
    auto config = MakeConfig();
    config.continuous_resync_threshold = 0.05;
    config.tracking_fault_threshold = 0.05;

    eyou_ros1_master::HybridJointTargetExecutor executor(&hw, &loop_mtx, config);
    EXPECT_FALSE(executor.valid());
    EXPECT_NE(executor.config_error().find("tracking_fault_threshold"),
              std::string::npos);
}

TEST_F(HybridJointTargetExecutorTest, HoldsCurrentPositionWhenNoTargetIsSet) {
    FakeRobotHw hw;
    std::mutex loop_mtx;
    hw.SetState("joint_a", 0.25);
    hw.SetState("joint_b", -0.5);

    eyou_ros1_master::HybridJointTargetExecutor executor(&hw, &loop_mtx, MakeConfig());
    ASSERT_TRUE(executor.valid()) << executor.config_error();

    executor.update(ros::Duration(0.01));
    EXPECT_DOUBLE_EQ(hw.command("joint_a"), 0.25);
    EXPECT_DOUBLE_EQ(hw.command("joint_b"), -0.5);
    EXPECT_EQ(executor.getExecutionStatus(),
              eyou_ros1_master::HybridJointTargetExecutor::ExecutionStatus::kHold);

    const auto measured = executor.getMeasuredState();
    const auto command = executor.getCurrentCommand();
    EXPECT_DOUBLE_EQ(measured.positions[0], 0.25);
    EXPECT_DOUBLE_EQ(measured.positions[1], -0.5);
    EXPECT_DOUBLE_EQ(command.positions[0], 0.25);
    EXPECT_DOUBLE_EQ(command.positions[1], -0.5);
}

TEST_F(HybridJointTargetExecutorTest, RejectsTargetWithMismatchedDegreesOfFreedom) {
    FakeRobotHw hw;
    std::mutex loop_mtx;
    eyou_ros1_master::HybridJointTargetExecutor executor(&hw, &loop_mtx, MakeConfig());
    ASSERT_TRUE(executor.valid()) << executor.config_error();

    eyou_ros1_master::HybridJointTargetExecutor::State target;
    target.positions = {1.0};

    std::string error;
    EXPECT_FALSE(executor.setTarget(target, &error));
    EXPECT_NE(error.find("size mismatch"), std::string::npos);
}

TEST_F(HybridJointTargetExecutorTest, AdvancesTowardConfiguredTarget) {
    FakeRobotHw hw;
    std::mutex loop_mtx;
    hw.SetState("joint_a", 0.0);
    hw.SetState("joint_b", 0.0);

    eyou_ros1_master::HybridJointTargetExecutor executor(&hw, &loop_mtx, MakeConfig());
    ASSERT_TRUE(executor.valid()) << executor.config_error();

    eyou_ros1_master::HybridJointTargetExecutor::State target;
    target.positions = {0.4, 0.8};
    ASSERT_TRUE(executor.setTarget(target));

    executor.update(ros::Duration(0.01));

    EXPECT_GT(hw.command("joint_a"), 0.0);
    EXPECT_GT(hw.command("joint_b"), 0.0);
    EXPECT_LT(hw.command("joint_a"), 0.4);
    EXPECT_LT(hw.command("joint_b"), 0.8);
    EXPECT_EQ(executor.getExecutionStatus(),
              eyou_ros1_master::HybridJointTargetExecutor::ExecutionStatus::kTracking);

    const auto command = executor.getCurrentCommand();
    EXPECT_DOUBLE_EQ(command.positions[0], hw.command("joint_a"));
    EXPECT_DOUBLE_EQ(command.positions[1], hw.command("joint_b"));
}

TEST_F(HybridJointTargetExecutorTest,
       ActionPreemptsServoButServoCannotStealOwnership) {
    FakeRobotHw hw;
    std::mutex loop_mtx;
    hw.SetState("joint_a", 0.0);
    hw.SetState("joint_b", 0.0);

    eyou_ros1_master::HybridJointTargetExecutor executor(&hw, &loop_mtx, MakeConfig());
    ASSERT_TRUE(executor.valid()) << executor.config_error();

    eyou_ros1_master::HybridJointTargetExecutor::State servo_target;
    servo_target.positions = {0.2, 0.4};
    ASSERT_TRUE(executor.setTargetFrom(
        eyou_ros1_master::HybridJointTargetExecutor::Source::kServo,
        servo_target));
    ASSERT_TRUE(executor.active_source().has_value());
    EXPECT_EQ(*executor.active_source(),
              eyou_ros1_master::HybridJointTargetExecutor::Source::kServo);

    eyou_ros1_master::HybridJointTargetExecutor::State action_target;
    action_target.positions = {0.5, 0.7};
    ASSERT_TRUE(executor.setTargetFrom(
        eyou_ros1_master::HybridJointTargetExecutor::Source::kAction,
        action_target));
    ASSERT_TRUE(executor.active_source().has_value());
    EXPECT_EQ(*executor.active_source(),
              eyou_ros1_master::HybridJointTargetExecutor::Source::kAction);

    std::string error;
    EXPECT_FALSE(executor.setTargetFrom(
        eyou_ros1_master::HybridJointTargetExecutor::Source::kServo,
        servo_target, &error));
    EXPECT_NE(error.find("owned by another source"), std::string::npos);
}

TEST_F(HybridJointTargetExecutorTest,
       ContinuousReferenceUpdatesAdvanceInsteadOfResetting) {
    FakeRobotHw hw;
    std::mutex loop_mtx;
    auto config = MakeConfig();
    config.max_velocities = {20.0, 20.0};
    config.max_accelerations = {100.0, 100.0};
    config.max_jerks = {1000.0, 1000.0};

    eyou_ros1_master::HybridJointTargetExecutor executor(&hw, &loop_mtx, config);
    ASSERT_TRUE(executor.valid()) << executor.config_error();

    eyou_ros1_master::HybridJointTargetExecutor::Target target;
    target.state.positions = {0.1, 0.1};
    target.state.velocities = {0.0, 0.0};
    target.state.accelerations = {0.0, 0.0};
    target.continuous_reference = true;

    ASSERT_TRUE(executor.setTarget(target));
    executor.update(ros::Duration(0.01));
    const double first_cmd = hw.command("joint_a");
    ASSERT_GT(first_cmd, 0.0);

    hw.SetState("joint_a", first_cmd);
    hw.SetState("joint_b", first_cmd);

    target.state.positions = {0.2, 0.2};
    ASSERT_TRUE(executor.setTarget(target));
    executor.update(ros::Duration(0.01));

    EXPECT_GT(hw.command("joint_a"), first_cmd);
    EXPECT_GT(hw.command("joint_b"), first_cmd);
}

TEST_F(HybridJointTargetExecutorTest,
       SwitchingIntoContinuousModeReinitializesFromMeasuredHardwareState) {
    FakeRobotHw hw;
    std::mutex loop_mtx;
    auto config = MakeConfig();
    config.max_velocities = {20.0, 20.0};
    config.max_accelerations = {100.0, 100.0};
    config.max_jerks = {1000.0, 1000.0};

    eyou_ros1_master::HybridJointTargetExecutor executor(&hw, &loop_mtx, config);
    ASSERT_TRUE(executor.valid()) << executor.config_error();

    eyou_ros1_master::HybridJointTargetExecutor::State waypoint_target;
    waypoint_target.positions = {0.5, 0.5};
    ASSERT_TRUE(executor.setTarget(waypoint_target));
    executor.update(ros::Duration(0.01));
    ASSERT_GT(hw.command("joint_a"), 0.0);

    hw.SetState("joint_a", 2.0);
    hw.SetState("joint_b", 2.0);

    eyou_ros1_master::HybridJointTargetExecutor::Target continuous_target;
    continuous_target.state.positions = {2.2, 2.2};
    continuous_target.continuous_reference = true;
    ASSERT_TRUE(executor.setTarget(continuous_target));
    executor.update(ros::Duration(0.01));

    EXPECT_GT(hw.command("joint_a"), 1.5);
    EXPECT_GT(hw.command("joint_b"), 1.5);
    EXPECT_EQ(executor.getExecutionStatus(),
              eyou_ros1_master::HybridJointTargetExecutor::ExecutionStatus::kTracking);
    EXPECT_EQ(
        executor.getContinuousModeState(),
        eyou_ros1_master::HybridJointTargetExecutor::ContinuousModeState::
            kFollowInternalState);
}

TEST_F(HybridJointTargetExecutorTest,
       ContinuousModeResyncsAfterConsecutiveTrackingErrorAndRecoversWithHysteresis) {
    FakeRobotHw hw;
    std::mutex loop_mtx;
    auto config = MakeConfig();
    config.max_velocities = {20.0, 20.0};
    config.max_accelerations = {100.0, 100.0};
    config.max_jerks = {1000.0, 1000.0};
    config.continuous_resync_threshold = 0.01;
    config.continuous_resync_recovery_threshold = 0.005;
    config.continuous_resync_enter_cycles = 2;
    config.continuous_resync_recovery_cycles = 2;

    eyou_ros1_master::HybridJointTargetExecutor executor(&hw, &loop_mtx, config);
    ASSERT_TRUE(executor.valid()) << executor.config_error();

    eyou_ros1_master::HybridJointTargetExecutor::Target target;
    target.state.positions = {1.0, 1.0};
    target.continuous_reference = true;
    ASSERT_TRUE(executor.setTarget(target));

    executor.update(ros::Duration(0.01));
    const double first_cmd = hw.command("joint_a");
    ASSERT_GT(first_cmd, 0.0);
    hw.SetState("joint_a", -0.03);
    hw.SetState("joint_b", -0.03);

    double resync_cmd = first_cmd;
    bool reached_resync = false;
    for (int step = 0; step < 5; ++step) {
        executor.update(ros::Duration(0.01));
        resync_cmd = hw.command("joint_a");
        if (executor.getExecutionStatus() ==
            eyou_ros1_master::HybridJointTargetExecutor::ExecutionStatus::
                kResyncing) {
            reached_resync = true;
            break;
        }
    }
    ASSERT_TRUE(reached_resync);
    EXPECT_EQ(
        executor.getContinuousModeState(),
        eyou_ros1_master::HybridJointTargetExecutor::ContinuousModeState::
            kResyncFromHardware);
    EXPECT_FALSE(executor.getTrackingFault().has_value());

    hw.SetState("joint_a", resync_cmd);
    hw.SetState("joint_b", resync_cmd);
    executor.update(ros::Duration(0.01));
    EXPECT_NE(executor.getExecutionStatus(),
              eyou_ros1_master::HybridJointTargetExecutor::ExecutionStatus::
                  kTrackingFault);

    for (int step = 0; step < 5; ++step) {
        const double recovery_cmd = hw.command("joint_a");
        hw.SetState("joint_a", recovery_cmd);
        hw.SetState("joint_b", recovery_cmd);
        executor.update(ros::Duration(0.01));
        if (executor.getContinuousModeState() ==
            eyou_ros1_master::HybridJointTargetExecutor::ContinuousModeState::
                kFollowInternalState) {
            break;
        }
    }
    EXPECT_EQ(executor.getExecutionStatus(),
              eyou_ros1_master::HybridJointTargetExecutor::ExecutionStatus::kTracking);
    EXPECT_EQ(
        executor.getContinuousModeState(),
        eyou_ros1_master::HybridJointTargetExecutor::ContinuousModeState::
            kFollowInternalState);
    EXPECT_FALSE(executor.getTrackingFault().has_value());
}

TEST_F(HybridJointTargetExecutorTest,
       ResyncTakesPriorityOverTrackingFaultWhenContinuousTargetFallsBehind) {
    FakeRobotHw hw;
    std::mutex loop_mtx;
    auto config = MakeConfig();
    config.max_velocities = {20.0, 20.0};
    config.max_accelerations = {100.0, 100.0};
    config.max_jerks = {1000.0, 1000.0};
    config.continuous_resync_threshold = 0.01;
    config.continuous_resync_recovery_threshold = 0.005;
    config.continuous_resync_enter_cycles = 1;
    config.continuous_resync_recovery_cycles = 2;
    config.tracking_fault_threshold = 0.02;

    eyou_ros1_master::HybridJointTargetExecutor executor(&hw, &loop_mtx, config);
    ASSERT_TRUE(executor.valid()) << executor.config_error();

    eyou_ros1_master::HybridJointTargetExecutor::Target target;
    target.state.positions = {1.0, 1.0};
    target.continuous_reference = true;
    ASSERT_TRUE(executor.setTarget(target));

    executor.update(ros::Duration(0.01));
    const double first_cmd = hw.command("joint_a");
    ASSERT_GT(first_cmd, 0.0);

    hw.SetState("joint_a", -0.2);
    hw.SetState("joint_b", -0.2);
    executor.update(ros::Duration(0.01));

    EXPECT_EQ(executor.getExecutionStatus(),
              eyou_ros1_master::HybridJointTargetExecutor::ExecutionStatus::
                  kResyncing);
    EXPECT_EQ(
        executor.getContinuousModeState(),
        eyou_ros1_master::HybridJointTargetExecutor::ContinuousModeState::
            kResyncFromHardware);
    EXPECT_FALSE(executor.getTrackingFault().has_value());

    const double resync_cmd = hw.command("joint_a");
    hw.SetState("joint_a", resync_cmd);
    hw.SetState("joint_b", resync_cmd);
    executor.update(ros::Duration(0.01));

    EXPECT_NE(executor.getExecutionStatus(),
              eyou_ros1_master::HybridJointTargetExecutor::ExecutionStatus::
                  kTrackingFault);
    EXPECT_FALSE(executor.getTrackingFault().has_value());

    hw.SetState("joint_a", hw.command("joint_a"));
    hw.SetState("joint_b", hw.command("joint_b"));
    executor.update(ros::Duration(0.01));

    EXPECT_NE(executor.getExecutionStatus(),
              eyou_ros1_master::HybridJointTargetExecutor::ExecutionStatus::
                  kTrackingFault);
    EXPECT_FALSE(executor.getTrackingFault().has_value());
}

TEST_F(HybridJointTargetExecutorTest, TrackingFaultLatchesUntilTargetIsCleared) {
    FakeRobotHw hw;
    std::mutex loop_mtx;
    auto config = MakeConfig();
    config.max_velocities = {20.0, 20.0};
    config.max_accelerations = {100.0, 100.0};
    config.max_jerks = {1000.0, 1000.0};
    config.continuous_resync_threshold = 0.01;
    config.continuous_resync_recovery_threshold = 0.005;
    config.continuous_resync_enter_cycles = 2;
    config.continuous_resync_recovery_cycles = 2;
    config.tracking_fault_threshold = 0.05;

    eyou_ros1_master::HybridJointTargetExecutor executor(&hw, &loop_mtx, config);
    ASSERT_TRUE(executor.valid()) << executor.config_error();

    eyou_ros1_master::HybridJointTargetExecutor::Target target;
    target.state.positions = {1.0, 1.0};
    target.continuous_reference = true;
    ASSERT_TRUE(executor.setTarget(target));

    executor.update(ros::Duration(0.01));
    ASSERT_GT(hw.command("joint_a"), 0.0);

    hw.SetState("joint_a", -0.2);
    hw.SetState("joint_b", -0.2);
    executor.update(ros::Duration(0.01));

    EXPECT_EQ(executor.getExecutionStatus(),
              eyou_ros1_master::HybridJointTargetExecutor::ExecutionStatus::
                  kTrackingFault);
    const auto fault = executor.getTrackingFault();
    ASSERT_TRUE(fault.has_value());
    EXPECT_EQ(fault->joint_name, "joint_a");
    EXPECT_LT(fault->position_error, -config.tracking_fault_threshold);

    hw.SetState("joint_a", hw.command("joint_a"));
    hw.SetState("joint_b", hw.command("joint_b"));
    executor.update(ros::Duration(0.01));
    EXPECT_EQ(executor.getExecutionStatus(),
              eyou_ros1_master::HybridJointTargetExecutor::ExecutionStatus::
                  kTrackingFault);
    ASSERT_TRUE(executor.getTrackingFault().has_value());

    executor.clearTarget();
    executor.update(ros::Duration(0.01));
    EXPECT_EQ(executor.getExecutionStatus(),
              eyou_ros1_master::HybridJointTargetExecutor::ExecutionStatus::kHold);
    EXPECT_FALSE(executor.getTrackingFault().has_value());
}

TEST_F(HybridJointTargetExecutorTest, FinishedStatusReportsCompletion) {
    FakeRobotHw hw;
    std::mutex loop_mtx;
    auto config = MakeConfig();
    config.max_velocities = {20.0, 20.0};
    config.max_accelerations = {100.0, 100.0};
    config.max_jerks = {1000.0, 1000.0};

    eyou_ros1_master::HybridJointTargetExecutor executor(&hw, &loop_mtx, config);
    ASSERT_TRUE(executor.valid()) << executor.config_error();

    EXPECT_EQ(executor.getExecutionStatus(),
              eyou_ros1_master::HybridJointTargetExecutor::ExecutionStatus::kHold);

    eyou_ros1_master::HybridJointTargetExecutor::State target;
    target.positions = {0.01, 0.01};
    ASSERT_TRUE(executor.setTarget(target));

    // Run enough cycles for Ruckig to finish a tiny move.
    for (int step = 0; step < 200; ++step) {
        hw.SetState("joint_a", hw.command("joint_a"));
        hw.SetState("joint_b", hw.command("joint_b"));
        executor.update(ros::Duration(0.01));
        if (executor.getExecutionStatus() ==
            eyou_ros1_master::HybridJointTargetExecutor::ExecutionStatus::kFinished) {
            break;
        }
    }

    EXPECT_EQ(executor.getExecutionStatus(),
              eyou_ros1_master::HybridJointTargetExecutor::ExecutionStatus::kFinished);

    const auto command = executor.getCurrentCommand();
    EXPECT_NEAR(command.positions[0], hw.command("joint_a"), 1e-9);
    EXPECT_NEAR(command.positions[1], hw.command("joint_b"), 1e-9);
}

TEST_F(HybridJointTargetExecutorTest, ExposesConfiguredJointOrder) {
    FakeRobotHw hw;
    std::mutex loop_mtx;

    eyou_ros1_master::HybridJointTargetExecutor executor(&hw, &loop_mtx, MakeConfig());
    ASSERT_TRUE(executor.valid()) << executor.config_error();

    const auto& joint_names = executor.getJointNames();
    ASSERT_EQ(joint_names.size(), 2u);
    EXPECT_EQ(joint_names[0], "joint_a");
    EXPECT_EQ(joint_names[1], "joint_b");
}
