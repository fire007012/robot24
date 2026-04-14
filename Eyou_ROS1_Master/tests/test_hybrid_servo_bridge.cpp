#include <gtest/gtest.h>

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

#include "Eyou_ROS1_Master/hybrid_joint_target_executor.hpp"
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

    void SetState(const std::string& name, double pos, double vel = 0.0) {
        auto& joint = joints_.at(IndexFor(name));
        joint.pos = pos;
        joint.vel = vel;
    }

    void TrackCommand() {
        for (auto& joint : joints_) {
            const double previous = joint.pos;
            joint.pos = joint.cmd;
            joint.vel = joint.pos - previous;
        }
    }

    void TrackCommandWithLag(double ratio) {
        for (auto& joint : joints_) {
            const double previous = joint.pos;
            joint.pos += (joint.cmd - joint.pos) * ratio;
            joint.vel = joint.pos - previous;
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
        throw std::out_of_range("unknown joint");
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

eyou_ros1_master::HybridServoBridge::Config MakeBridgeConfig() {
    eyou_ros1_master::HybridServoBridge::Config config;
    config.joint_names = {"joint_a", "joint_b"};
    config.input_topic = "servo_joint_targets";
    config.timeout_sec = 0.1;
    return config;
}

trajectory_msgs::JointTrajectory MakeTrajectory(const std::vector<std::string>& joint_names,
                                                const std::vector<double>& positions) {
    trajectory_msgs::JointTrajectory traj;
    traj.joint_names = joint_names;
    trajectory_msgs::JointTrajectoryPoint point;
    point.positions = positions;
    traj.points.push_back(point);
    return traj;
}

trajectory_msgs::JointTrajectory MakeTrajectory(
    const std::vector<std::string>& joint_names,
    const std::vector<double>& positions,
    const std::vector<double>& velocities) {
    trajectory_msgs::JointTrajectory traj;
    traj.joint_names = joint_names;
    trajectory_msgs::JointTrajectoryPoint point;
    point.positions = positions;
    point.velocities = velocities;
    traj.points.push_back(point);
    return traj;
}

}  // namespace

class HybridServoBridgeTest : public ::testing::Test {
protected:
    static void SetUpTestSuite() {
        if (!ros::isInitialized()) {
            int argc = 0;
            char** argv = nullptr;
            ros::init(argc, argv, "test_hybrid_servo_bridge",
                      ros::init_options::AnonymousName |
                          ros::init_options::NoSigintHandler |
                          ros::init_options::NoRosout);
        }
        ros::Time::init();
    }
};

TEST_F(HybridServoBridgeTest, RejectsTrajectoryWithUnknownJoint) {
    FakeRobotHw hw;
    std::mutex loop_mtx;
    eyou_ros1_master::HybridJointTargetExecutor target_executor(&hw, &loop_mtx,
                                                                MakeTargetConfig());
    ASSERT_TRUE(target_executor.valid()) << target_executor.config_error();

    eyou_ros1_master::HybridServoBridge bridge(nullptr, &target_executor,
                                               MakeBridgeConfig());
    ASSERT_TRUE(bridge.valid()) << bridge.config_error();

    const auto traj = MakeTrajectory({"joint_a", "joint_x"}, {0.2, 0.4});
    std::string error;
    EXPECT_FALSE(bridge.acceptTrajectory(traj, ros::Time(1.0), &error));
    EXPECT_NE(error.find("unknown joint"), std::string::npos);
}

TEST_F(HybridServoBridgeTest,
       RejectsDuplicateJointNamesWithoutOverwritingLastValidTarget) {
    FakeRobotHw hw;
    std::mutex loop_mtx;
    eyou_ros1_master::HybridJointTargetExecutor target_executor(&hw, &loop_mtx,
                                                                MakeTargetConfig());
    ASSERT_TRUE(target_executor.valid()) << target_executor.config_error();

    eyou_ros1_master::HybridServoBridge bridge(nullptr, &target_executor,
                                               MakeBridgeConfig());
    ASSERT_TRUE(bridge.valid()) << bridge.config_error();

    ASSERT_TRUE(bridge.acceptTrajectory(
        MakeTrajectory({"joint_a", "joint_b"}, {0.2, 0.4}), ros::Time(1.0)));

    std::string error;
    EXPECT_FALSE(bridge.acceptTrajectory(
        MakeTrajectory({"joint_a", "joint_a"}, {0.8, 0.9}), ros::Time(1.01),
        &error));
    EXPECT_NE(error.find("duplicate joint"), std::string::npos);

    bridge.update(ros::Time(1.02));
    const auto active_target = target_executor.getActiveTarget();
    ASSERT_TRUE(active_target.has_value());
    EXPECT_EQ(active_target->state.positions, std::vector<double>({0.2, 0.4}));
}

TEST_F(HybridServoBridgeTest, MapsLatestPointIntoSharedTargetExecutor) {
    FakeRobotHw hw;
    std::mutex loop_mtx;
    eyou_ros1_master::HybridJointTargetExecutor target_executor(&hw, &loop_mtx,
                                                                MakeTargetConfig());
    ASSERT_TRUE(target_executor.valid()) << target_executor.config_error();

    eyou_ros1_master::HybridServoBridge bridge(nullptr, &target_executor,
                                               MakeBridgeConfig());
    ASSERT_TRUE(bridge.valid()) << bridge.config_error();

    auto traj = MakeTrajectory({"joint_b", "joint_a"}, {0.8, 0.4});
    trajectory_msgs::JointTrajectoryPoint later_point;
    later_point.positions = {0.9, 0.5};
    traj.points.push_back(later_point);

    ASSERT_TRUE(bridge.acceptTrajectory(traj, ros::Time(1.0)));
    bridge.update(ros::Time(1.02));
    target_executor.update(ros::Duration(0.01));
    hw.TrackCommand();

    EXPECT_GT(hw.command("joint_a"), 0.0);
    EXPECT_GT(hw.command("joint_b"), 0.0);
    EXPECT_LT(hw.command("joint_a"), 0.5);
    EXPECT_LT(hw.command("joint_b"), 0.9);
}

TEST_F(HybridServoBridgeTest,
       MapsOptionalVelocityFeedforwardIntoExecutorJointOrder) {
    FakeRobotHw hw;
    std::mutex loop_mtx;
    eyou_ros1_master::HybridJointTargetExecutor target_executor(&hw, &loop_mtx,
                                                                MakeTargetConfig());
    ASSERT_TRUE(target_executor.valid()) << target_executor.config_error();

    eyou_ros1_master::HybridServoBridge bridge(nullptr, &target_executor,
                                               MakeBridgeConfig());
    ASSERT_TRUE(bridge.valid()) << bridge.config_error();

    ASSERT_TRUE(bridge.acceptTrajectory(
        MakeTrajectory({"joint_b", "joint_a"}, {0.8, 0.4}, {0.6, -0.2}),
        ros::Time(1.0)));
    bridge.update(ros::Time(1.0));

    const auto active_target = target_executor.getActiveTarget();
    ASSERT_TRUE(active_target.has_value());
    EXPECT_TRUE(active_target->continuous_reference);
    EXPECT_EQ(active_target->state.positions,
              std::vector<double>({0.4, 0.8}));
    EXPECT_EQ(active_target->state.velocities,
              std::vector<double>({-0.2, 0.6}));
}

TEST_F(HybridServoBridgeTest,
       TimeoutUsesMeasuredStateStopTargetForSmoothDeceleration) {
    FakeRobotHw hw;
    std::mutex loop_mtx;
    auto target_config = MakeTargetConfig();
    target_config.max_velocities = {20.0, 20.0};
    target_config.max_accelerations = {100.0, 100.0};
    target_config.max_jerks = {1000.0, 1000.0};
    target_config.tracking_fault_threshold = 1.0;
    hw.SetState("joint_a", 0.25, 0.4);
    hw.SetState("joint_b", -0.5, 0.4);
    eyou_ros1_master::HybridJointTargetExecutor target_executor(&hw, &loop_mtx,
                                                                target_config);
    ASSERT_TRUE(target_executor.valid()) << target_executor.config_error();

    eyou_ros1_master::HybridServoBridge bridge(nullptr, &target_executor,
                                               MakeBridgeConfig());
    ASSERT_TRUE(bridge.valid()) << bridge.config_error();

    const auto traj = MakeTrajectory({"joint_a", "joint_b"}, {0.6, -0.2});
    ASSERT_TRUE(bridge.acceptTrajectory(traj, ros::Time(1.0)));
    bridge.update(ros::Time(1.02));
    target_executor.update(ros::Duration(0.01));
    const double active_cmd = hw.command("joint_a");
    ASSERT_GT(active_cmd, 0.25);

    hw.SetState("joint_a", 0.35, 0.3);
    hw.SetState("joint_b", -0.35, 0.3);
    bridge.update(ros::Time(1.08));
    target_executor.update(ros::Duration(0.01));

    bridge.update(ros::Time(1.2));
    target_executor.update(ros::Duration(0.01));

    EXPECT_GT(hw.command("joint_a"), 0.35);
    EXPECT_LT(hw.command("joint_a"), 0.6);

    const auto active_target = target_executor.getActiveTarget();
    ASSERT_TRUE(active_target.has_value());
    EXPECT_FALSE(active_target->continuous_reference);
    EXPECT_EQ(active_target->state.positions,
              std::vector<double>({0.35, -0.35}));
    EXPECT_EQ(active_target->state.velocities,
              std::vector<double>({0.0, 0.0}));
    EXPECT_EQ(active_target->state.accelerations,
              std::vector<double>({0.0, 0.0}));
}

TEST_F(HybridServoBridgeTest,
       ClearsServoOwnershipAfterTrackingFaultAndAllowsReacquire) {
    FakeRobotHw hw;
    std::mutex loop_mtx;
    auto target_config = MakeTargetConfig();
    target_config.max_velocities = {20.0, 20.0};
    target_config.max_accelerations = {100.0, 100.0};
    target_config.max_jerks = {1000.0, 1000.0};
    target_config.continuous_resync_threshold = 0.01;
    target_config.continuous_resync_recovery_threshold = 0.005;
    target_config.continuous_resync_enter_cycles = 2;
    target_config.continuous_resync_recovery_cycles = 2;
    target_config.tracking_fault_threshold = 0.05;
    eyou_ros1_master::HybridJointTargetExecutor target_executor(&hw, &loop_mtx,
                                                                target_config);
    ASSERT_TRUE(target_executor.valid()) << target_executor.config_error();

    eyou_ros1_master::HybridServoBridge bridge(nullptr, &target_executor,
                                               MakeBridgeConfig());
    ASSERT_TRUE(bridge.valid()) << bridge.config_error();

    ros::Time sim_now(1.0);
    ASSERT_TRUE(bridge.acceptTrajectory(
        MakeTrajectory({"joint_a", "joint_b"}, {1.0, 1.0}), sim_now));

    bridge.update(sim_now);
    target_executor.update(ros::Duration(0.01));
    ASSERT_TRUE(target_executor.active_source().has_value());

    hw.SetState("joint_a", -0.2);
    hw.SetState("joint_b", -0.2);
    sim_now += ros::Duration(0.01);
    bridge.update(sim_now);
    target_executor.update(ros::Duration(0.01));

    EXPECT_EQ(target_executor.getExecutionStatus(),
              eyou_ros1_master::HybridJointTargetExecutor::ExecutionStatus::
                  kTrackingFault);
    ASSERT_TRUE(target_executor.active_source().has_value());
    EXPECT_EQ(*target_executor.active_source(),
              eyou_ros1_master::HybridJointTargetExecutor::Source::kServo);

    sim_now += ros::Duration(0.01);
    bridge.update(sim_now);
    EXPECT_FALSE(target_executor.active_source().has_value());
    EXPECT_EQ(target_executor.getExecutionStatus(),
              eyou_ros1_master::HybridJointTargetExecutor::ExecutionStatus::kHold);
    EXPECT_FALSE(target_executor.getTrackingFault().has_value());

    target_executor.update(ros::Duration(0.01));
    EXPECT_EQ(target_executor.getExecutionStatus(),
              eyou_ros1_master::HybridJointTargetExecutor::ExecutionStatus::kHold);

    ASSERT_TRUE(bridge.acceptTrajectory(
        MakeTrajectory({"joint_a", "joint_b"}, {0.2, -0.1}),
        sim_now + ros::Duration(0.01)));
    bridge.update(sim_now + ros::Duration(0.01));
    target_executor.update(ros::Duration(0.01));

    ASSERT_TRUE(target_executor.active_source().has_value());
    EXPECT_EQ(*target_executor.active_source(),
              eyou_ros1_master::HybridJointTargetExecutor::Source::kServo);
    EXPECT_NE(target_executor.getExecutionStatus(),
              eyou_ros1_master::HybridJointTargetExecutor::ExecutionStatus::
                  kTrackingFault);
}

TEST_F(HybridServoBridgeTest, ContinuousServoUpdatesAdvanceWithoutResettingMotion) {
    FakeRobotHw hw;
    std::mutex loop_mtx;
    auto target_config = MakeTargetConfig();
    target_config.max_velocities = {20.0, 20.0};
    target_config.max_accelerations = {100.0, 100.0};
    target_config.max_jerks = {1000.0, 1000.0};
    eyou_ros1_master::HybridJointTargetExecutor target_executor(&hw, &loop_mtx,
                                                                target_config);
    ASSERT_TRUE(target_executor.valid()) << target_executor.config_error();

    eyou_ros1_master::HybridServoBridge bridge(nullptr, &target_executor,
                                               MakeBridgeConfig());
    ASSERT_TRUE(bridge.valid()) << bridge.config_error();

    ASSERT_TRUE(bridge.acceptTrajectory(
        MakeTrajectory({"joint_a", "joint_b"}, {0.1, 0.1}), ros::Time(1.0)));
    bridge.update(ros::Time(1.0));
    target_executor.update(ros::Duration(0.01));
    const double first_cmd = hw.command("joint_a");
    ASSERT_GT(first_cmd, 0.0);

    hw.TrackCommandWithLag(0.2);

    ASSERT_TRUE(bridge.acceptTrajectory(
        MakeTrajectory({"joint_a", "joint_b"}, {0.2, 0.2}), ros::Time(1.01)));
    bridge.update(ros::Time(1.01));
    target_executor.update(ros::Duration(0.01));

    EXPECT_GT(hw.command("joint_a"), first_cmd);
    EXPECT_GT(hw.command("joint_b"), first_cmd);
}
