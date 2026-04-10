#include <gtest/gtest.h>

#include <xmlrpcpp/XmlRpcValue.h>

#include "Eyou_ROS1_Master/hybrid_ip_executor_config.hpp"

namespace {

canopen_hw::CanopenMasterConfig MakeCanopenConfig() {
    canopen_hw::CanopenMasterConfig config;

    canopen_hw::CanopenMasterConfig::JointConfig yaw;
    yaw.name = "shoulder_yaw_joint";
    yaw.ip_max_velocity = 1.0;
    yaw.ip_max_acceleration = 2.0;
    yaw.ip_max_jerk = 10.0;
    yaw.ip_goal_tolerance = 0.001;
    config.joints.push_back(yaw);

    canopen_hw::CanopenMasterConfig::JointConfig pitch;
    pitch.name = "shoulder_pitch_joint";
    pitch.ip_max_velocity = 1.1;
    pitch.ip_max_acceleration = 2.1;
    pitch.ip_max_jerk = 10.1;
    pitch.ip_goal_tolerance = 0.0011;
    config.joints.push_back(pitch);

    return config;
}

XmlRpc::XmlRpcValue MakeCanDriverJointList(const std::string& joint_name,
                                           const std::string& control_mode) {
    XmlRpc::XmlRpcValue joint_list;
    joint_list.setSize(1);
    joint_list[0]["name"] = joint_name;
    joint_list[0]["motor_id"] = 0x13;
    joint_list[0]["protocol"] = std::string("PP");
    joint_list[0]["can_device"] = std::string("can1");
    joint_list[0]["control_mode"] = control_mode;
    joint_list[0]["ip_max_velocity"] = 1.2;
    joint_list[0]["ip_max_acceleration"] = 2.2;
    joint_list[0]["ip_max_jerk"] = 10.2;
    joint_list[0]["ip_goal_tolerance"] = 0.0012;
    return joint_list;
}

}  // namespace

TEST(HybridIpExecutorConfig, MergesCanopenAndCanDriverTrajectoryJoints) {
    const auto canopen_config = MakeCanopenConfig();
    const auto can_driver_joint_list =
        MakeCanDriverJointList("elbow_pitch_joint", "csp");

    canopen_hw::IpFollowJointTrajectoryExecutor::Config config;
    std::string error;
    ASSERT_TRUE(eyou_ros1_master::BuildHybridIpExecutorConfig(
        canopen_config, can_driver_joint_list,
        "arm_position_controller/follow_joint_trajectory", 200.0, &config, &error));
    EXPECT_TRUE(error.empty());

    ASSERT_EQ(config.joint_names.size(), 3u);
    EXPECT_EQ(config.joint_names[0], "shoulder_yaw_joint");
    EXPECT_EQ(config.joint_names[1], "shoulder_pitch_joint");
    EXPECT_EQ(config.joint_names[2], "elbow_pitch_joint");
    EXPECT_EQ(config.joint_indices[0], 0u);
    EXPECT_EQ(config.joint_indices[1], 1u);
    EXPECT_EQ(config.joint_indices[2], 2u);
    EXPECT_DOUBLE_EQ(config.max_velocities[2], 1.2);
    EXPECT_DOUBLE_EQ(config.max_accelerations[2], 2.2);
    EXPECT_DOUBLE_EQ(config.max_jerks[2], 10.2);
    EXPECT_DOUBLE_EQ(config.goal_tolerances[2], 0.0012);
}

TEST(HybridIpExecutorConfig, RejectsDuplicateJointNameAcrossBackends) {
    const auto canopen_config = MakeCanopenConfig();
    const auto can_driver_joint_list =
        MakeCanDriverJointList("shoulder_pitch_joint", "csp");

    canopen_hw::IpFollowJointTrajectoryExecutor::Config config;
    std::string error;
    EXPECT_FALSE(eyou_ros1_master::BuildHybridIpExecutorConfig(
        canopen_config, can_driver_joint_list,
        "arm_position_controller/follow_joint_trajectory", 200.0, &config, &error));
    EXPECT_NE(error.find("duplicate executor joint name"),
              std::string::npos);
}

TEST(HybridIpExecutorConfig, SkipsVelocitySemanticCanDriverJoints) {
    const auto canopen_config = MakeCanopenConfig();
    const auto can_driver_joint_list =
        MakeCanDriverJointList("left_track_joint", "velocity");

    canopen_hw::IpFollowJointTrajectoryExecutor::Config config;
    std::string error;
    ASSERT_TRUE(eyou_ros1_master::BuildHybridIpExecutorConfig(
        canopen_config, can_driver_joint_list,
        "arm_position_controller/follow_joint_trajectory", 200.0, &config, &error));
    EXPECT_TRUE(error.empty());
    ASSERT_EQ(config.joint_names.size(), 2u);
    EXPECT_EQ(config.joint_names[0], "shoulder_yaw_joint");
    EXPECT_EQ(config.joint_names[1], "shoulder_pitch_joint");
}
