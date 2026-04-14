#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>

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

eyou_ros1_master::HybridIpExecutorConfigOverrides MakeOverrides() {
    eyou_ros1_master::HybridIpExecutorConfigOverrides overrides;
    overrides.constraint_validation_enabled = false;
    return overrides;
}

std::string WriteMoveItJointLimitsFile(const std::string& basename,
                                       const std::string& contents) {
    const auto path = std::filesystem::temp_directory_path() / basename;
    std::ofstream out(path);
    out << contents;
    out.close();
    return path.string();
}

}  // namespace

TEST(HybridIpExecutorConfig, MergesCanopenAndCanDriverTrajectoryJoints) {
    const auto canopen_config = MakeCanopenConfig();
    const auto can_driver_joint_list =
        MakeCanDriverJointList("elbow_pitch_joint", "csp");

    eyou_ros1_master::HybridIpExecutorConfig config;
    std::string error;
    ASSERT_TRUE(eyou_ros1_master::BuildHybridIpExecutorConfig(
        canopen_config, can_driver_joint_list,
        "arm_position_controller/follow_joint_trajectory", 200.0, MakeOverrides(),
        &config, &error));
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
    EXPECT_DOUBLE_EQ(config.default_goal_tolerances[2], 0.0012);
    EXPECT_DOUBLE_EQ(config.default_goal_time_tolerance, 2.0);
    EXPECT_DOUBLE_EQ(config.default_stopped_velocity_tolerance, 0.05);
    EXPECT_DOUBLE_EQ(config.planning_deviation_warn_threshold, 0.20);
    EXPECT_EQ(config.action_velocity_hint_mode,
              eyou_ros1_master::ActionVelocityHintMode::kDisabled);
}

TEST(HybridIpExecutorConfig, RejectsDuplicateJointNameAcrossBackends) {
    const auto canopen_config = MakeCanopenConfig();
    const auto can_driver_joint_list =
        MakeCanDriverJointList("shoulder_pitch_joint", "csp");

    eyou_ros1_master::HybridIpExecutorConfig config;
    std::string error;
    EXPECT_FALSE(eyou_ros1_master::BuildHybridIpExecutorConfig(
        canopen_config, can_driver_joint_list,
        "arm_position_controller/follow_joint_trajectory", 200.0, MakeOverrides(),
        &config, &error));
    EXPECT_NE(error.find("duplicate executor joint name"),
              std::string::npos);
}

TEST(HybridIpExecutorConfig, SkipsVelocitySemanticCanDriverJoints) {
    const auto canopen_config = MakeCanopenConfig();
    const auto can_driver_joint_list =
        MakeCanDriverJointList("left_track_joint", "velocity");

    eyou_ros1_master::HybridIpExecutorConfig config;
    std::string error;
    ASSERT_TRUE(eyou_ros1_master::BuildHybridIpExecutorConfig(
        canopen_config, can_driver_joint_list,
        "arm_position_controller/follow_joint_trajectory", 200.0, MakeOverrides(),
        &config, &error));
    EXPECT_TRUE(error.empty());
    ASSERT_EQ(config.joint_names.size(), 2u);
    EXPECT_EQ(config.joint_names[0], "shoulder_yaw_joint");
    EXPECT_EQ(config.joint_names[1], "shoulder_pitch_joint");
}

TEST(HybridIpExecutorConfig, RejectsInvalidThresholdRelationships) {
    const auto canopen_config = MakeCanopenConfig();
    const auto can_driver_joint_list =
        MakeCanDriverJointList("elbow_pitch_joint", "csp");

    auto overrides = MakeOverrides();
    overrides.continuous_resync_threshold = 0.08;
    overrides.tracking_fault_threshold = 0.08;

    eyou_ros1_master::HybridIpExecutorConfig config;
    std::string error;
    EXPECT_FALSE(eyou_ros1_master::BuildHybridIpExecutorConfig(
        canopen_config, can_driver_joint_list,
        "arm_position_controller/follow_joint_trajectory", 200.0, overrides,
        &config, &error));
    EXPECT_NE(error.find("resync_threshold"), std::string::npos);
}

TEST(HybridIpExecutorConfig, RejectsMoveItConstraintsThatExceedRuckigLimits) {
    const auto canopen_config = MakeCanopenConfig();
    const auto can_driver_joint_list =
        MakeCanDriverJointList("elbow_pitch_joint", "csp");

    auto overrides = MakeOverrides();
    overrides.constraint_validation_enabled = true;
    overrides.moveit_joint_limits_path = WriteMoveItJointLimitsFile(
        "eyou_ruckig_limits_fail.yaml",
        "joint_limits:\n"
        "  shoulder_yaw_joint:\n"
        "    has_velocity_limits: true\n"
        "    max_velocity: 1.0\n"
        "    has_acceleration_limits: true\n"
        "    max_acceleration: 2.0\n"
        "  shoulder_pitch_joint:\n"
        "    has_velocity_limits: true\n"
        "    max_velocity: 1.0\n"
        "    has_acceleration_limits: true\n"
        "    max_acceleration: 2.0\n"
        "  elbow_pitch_joint:\n"
        "    has_velocity_limits: true\n"
        "    max_velocity: 2.0\n"
        "    has_acceleration_limits: true\n"
        "    max_acceleration: 2.0\n");

    eyou_ros1_master::HybridIpExecutorConfig config;
    std::string error;
    EXPECT_FALSE(eyou_ros1_master::BuildHybridIpExecutorConfig(
        canopen_config, can_driver_joint_list,
        "arm_position_controller/follow_joint_trajectory", 200.0, overrides,
        &config, &error));
    EXPECT_NE(error.find("ip_max_velocity"), std::string::npos);
}

TEST(HybridIpExecutorConfig, AcceptsMoveItConstraintsWithinConfiguredMargins) {
    const auto canopen_config = MakeCanopenConfig();
    const auto can_driver_joint_list =
        MakeCanDriverJointList("elbow_pitch_joint", "csp");

    auto overrides = MakeOverrides();
    overrides.constraint_validation_enabled = true;
    overrides.constraint_validation_velocity_margin = 1.0;
    overrides.constraint_validation_acceleration_margin = 1.0;
    overrides.default_goal_tolerance_position = 0.02;
    overrides.default_path_tolerance_position = 0.01;
    overrides.moveit_joint_limits_path = WriteMoveItJointLimitsFile(
        "eyou_ruckig_limits_ok.yaml",
        "joint_limits:\n"
        "  shoulder_yaw_joint:\n"
        "    has_velocity_limits: true\n"
        "    max_velocity: 0.9\n"
        "    has_acceleration_limits: true\n"
        "    max_acceleration: 1.9\n"
        "  shoulder_pitch_joint:\n"
        "    has_velocity_limits: true\n"
        "    max_velocity: 1.0\n"
        "    has_acceleration_limits: true\n"
        "    max_acceleration: 2.0\n"
        "  elbow_pitch_joint:\n"
        "    has_velocity_limits: true\n"
        "    max_velocity: 1.1\n"
        "    has_acceleration_limits: true\n"
        "    max_acceleration: 2.1\n");

    eyou_ros1_master::HybridIpExecutorConfig config;
    std::string error;
    ASSERT_TRUE(eyou_ros1_master::BuildHybridIpExecutorConfig(
        canopen_config, can_driver_joint_list,
        "arm_position_controller/follow_joint_trajectory", 200.0, overrides,
        &config, &error))
        << error;
    EXPECT_EQ(config.default_goal_tolerances,
              std::vector<double>({0.02, 0.02, 0.02}));
    EXPECT_EQ(config.default_path_tolerances,
              std::vector<double>({0.01, 0.01, 0.01}));
}

TEST(HybridIpExecutorConfig, StoresActionVelocityHintModeOverride) {
    const auto canopen_config = MakeCanopenConfig();
    const auto can_driver_joint_list =
        MakeCanDriverJointList("elbow_pitch_joint", "csp");

    auto overrides = MakeOverrides();
    overrides.action_velocity_hint_mode =
        eyou_ros1_master::ActionVelocityHintMode::kSinglePoint;

    eyou_ros1_master::HybridIpExecutorConfig config;
    std::string error;
    ASSERT_TRUE(eyou_ros1_master::BuildHybridIpExecutorConfig(
        canopen_config, can_driver_joint_list,
        "arm_position_controller/follow_joint_trajectory", 200.0, overrides,
        &config, &error))
        << error;
    EXPECT_EQ(config.action_velocity_hint_mode,
              eyou_ros1_master::ActionVelocityHintMode::kSinglePoint);
}
