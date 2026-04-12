#include <gtest/gtest.h>

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>

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

TEST(HybridJointTargetExecutor, RejectsDuplicateJointNamesInConfig) {
    FakeRobotHw hw;
    std::mutex loop_mtx;
    auto config = MakeConfig();
    config.joint_names[1] = "joint_a";

    eyou_ros1_master::HybridJointTargetExecutor executor(&hw, &loop_mtx, config);
    EXPECT_FALSE(executor.valid());
    EXPECT_NE(executor.config_error().find("duplicate joint"),
              std::string::npos);
}

TEST(HybridJointTargetExecutor, HoldsCurrentPositionWhenNoTargetIsSet) {
    FakeRobotHw hw;
    std::mutex loop_mtx;
    hw.SetState("joint_a", 0.25);
    hw.SetState("joint_b", -0.5);

    eyou_ros1_master::HybridJointTargetExecutor executor(&hw, &loop_mtx, MakeConfig());
    ASSERT_TRUE(executor.valid()) << executor.config_error();

    executor.update(ros::Duration(0.01));
    EXPECT_DOUBLE_EQ(hw.command("joint_a"), 0.25);
    EXPECT_DOUBLE_EQ(hw.command("joint_b"), -0.5);
}

TEST(HybridJointTargetExecutor, RejectsTargetWithMismatchedDegreesOfFreedom) {
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

TEST(HybridJointTargetExecutor, AdvancesTowardConfiguredTarget) {
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
}

TEST(HybridJointTargetExecutor, ActionPreemptsServoButServoCannotStealOwnership) {
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
