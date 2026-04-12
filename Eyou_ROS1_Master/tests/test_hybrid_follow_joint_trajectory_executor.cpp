#include <gtest/gtest.h>

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

#include "Eyou_ROS1_Master/hybrid_follow_joint_trajectory_executor.hpp"

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
    config.goal_tolerances = {1e-3, 1e-3};
    return config;
}

trajectory_msgs::JointTrajectoryPoint MakePoint(
    const std::vector<double>& positions, double time_from_start_sec) {
    trajectory_msgs::JointTrajectoryPoint point;
    point.positions = positions;
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

}  // namespace

class HybridFollowJointTrajectoryExecutorTest : public ::testing::Test {
protected:
    static void SetUpTestSuite() {
        if (!ros::isInitialized()) {
            int argc = 0;
            char** argv = nullptr;
            ros::init(argc, argv, "test_hybrid_follow_joint_trajectory_executor",
                      ros::init_options::AnonymousName |
                          ros::init_options::NoSigintHandler |
                          ros::init_options::NoRosout);
        }
        ros::Time::init();
    }
};

TEST_F(HybridFollowJointTrajectoryExecutorTest, RejectsGoalWithUnknownJoint) {
    FakeRobotHw hw;
    std::mutex loop_mtx;
    eyou_ros1_master::HybridJointTargetExecutor target_executor(&hw, &loop_mtx,
                                                                MakeTargetConfig());
    ASSERT_TRUE(target_executor.valid()) << target_executor.config_error();

    eyou_ros1_master::HybridFollowJointTrajectoryExecutor executor(
        nullptr, &hw, &loop_mtx, &target_executor, MakeActionConfig());
    ASSERT_TRUE(executor.config_valid()) << executor.config_error();

    const auto goal = MakeGoal({"joint_a", "joint_x"},
                               {MakePoint({0.2, 0.3}, 0.5)});
    std::string error;
    EXPECT_FALSE(executor.startGoal(goal, &error));
    EXPECT_NE(error.find("unknown joint"), std::string::npos);
}

TEST_F(HybridFollowJointTrajectoryExecutorTest, DrivesGoalThroughSharedTargetExecutor) {
    FakeRobotHw hw;
    std::mutex loop_mtx;
    eyou_ros1_master::HybridJointTargetExecutor target_executor(&hw, &loop_mtx,
                                                                MakeTargetConfig());
    ASSERT_TRUE(target_executor.valid()) << target_executor.config_error();

    eyou_ros1_master::HybridFollowJointTrajectoryExecutor executor(
        nullptr, &hw, &loop_mtx, &target_executor, MakeActionConfig());
    ASSERT_TRUE(executor.config_valid()) << executor.config_error();

    const auto goal = MakeGoal({"joint_b", "joint_a"},
                               {MakePoint({0.8, 0.4}, 0.5)});
    std::string error;
    ASSERT_TRUE(executor.startGoal(goal, &error)) << error;

    for (int step = 0; step < 200 && executor.hasActiveGoal(); ++step) {
        executor.update(ros::Time::now(), ros::Duration(0.01));
        target_executor.update(ros::Duration(0.01));
        hw.TrackCommand();
    }

    EXPECT_FALSE(executor.hasActiveGoal());
    EXPECT_NEAR(hw.command("joint_a"), 0.4, 1e-3);
    EXPECT_NEAR(hw.command("joint_b"), 0.8, 1e-3);
}
