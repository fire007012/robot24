// hybrid_motor_hw_node.cpp — 统一外观节点
//
// 在单进程内同时托管 CanDriverHW (can_driver) 和 CanopenRobotHwRos
// (Eyou_Canopen_Master) 两套后端，共享一个 controller_manager，
// 对外暴露统一服务接口和 /joint_states。

#include <filesystem>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include <ros/ros.h>
#include <controller_manager/controller_manager.h>

#include "Eyou_ROS1_Master/JointRuntimeState.h"
#include "Eyou_ROS1_Master/JointRuntimeStateArray.h"
#include "Eyou_ROS1_Master/hybrid_robot_hw.hpp"
#include "Eyou_ROS1_Master/hybrid_auto_startup.hpp"
#include "Eyou_ROS1_Master/hybrid_follow_joint_trajectory_executor.hpp"
#include "Eyou_ROS1_Master/hybrid_ip_executor_config.hpp"
#include "Eyou_ROS1_Master/hybrid_joint_target_executor.hpp"
#include "Eyou_ROS1_Master/hybrid_mode_router.hpp"
#include "Eyou_ROS1_Master/hybrid_operational_coordinator.hpp"
#include "Eyou_ROS1_Master/hybrid_service_gateway.hpp"
#include "Eyou_ROS1_Master/hybrid_servo_bridge.hpp"

#include "can_driver/CanDriverHW.h"
#include "can_driver/motor_maintenance_service.hpp"
#include "canopen_hw/canopen_aux_services.hpp"
#include "canopen_hw/canopen_robot_hw_ros.hpp"
#include "canopen_hw/joints_config.hpp"
#include "canopen_hw/lifecycle_manager.hpp"
#include "canopen_hw/operational_coordinator.hpp"

namespace {

std::string MakeAbsolutePath(const std::string& path) {
    if (path.empty()) return path;
    std::filesystem::path p(path);
    return p.is_absolute() ? p.string() : std::filesystem::absolute(p).string();
}

bool FileExists(const std::string& path) {
    return !path.empty() && std::filesystem::exists(path);
}

eyou_ros1_master::HybridJointTargetExecutor::Config
MakeJointTargetExecutorConfig(
    const eyou_ros1_master::HybridFollowJointTrajectoryExecutor::Config& config) {
    eyou_ros1_master::HybridJointTargetExecutor::Config converted;
    converted.joint_names = config.joint_names;
    converted.joint_indices = config.joint_indices;
    converted.command_rate_hz = config.command_rate_hz;
    converted.max_velocities = config.max_velocities;
    converted.max_accelerations = config.max_accelerations;
    converted.max_jerks = config.max_jerks;
    converted.goal_tolerances = config.goal_tolerances;
    return converted;
}

eyou_ros1_master::HybridServoBridge::Config
MakeServoBridgeConfig(
    const eyou_ros1_master::HybridFollowJointTrajectoryExecutor::Config& config,
    const ros::NodeHandle& pnh) {
    eyou_ros1_master::HybridServoBridge::Config bridge_cfg;
    bridge_cfg.joint_names = config.joint_names;
    pnh.param<std::string>("servo_joint_target_topic", bridge_cfg.input_topic,
                           bridge_cfg.input_topic);
    pnh.param("servo_target_timeout_sec", bridge_cfg.timeout_sec,
              bridge_cfg.timeout_sec);
    return bridge_cfg;
}

}  // namespace

int main(int argc, char** argv) {
    ros::init(argc, argv, "hybrid_motor_hw_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    // ======================================================================
    // 1. CANopen 侧：解析配置 → LifecycleManager → ROS 适配层
    // ======================================================================
    std::string dcf_path, joints_path;
    pnh.param<std::string>("canopen_dcf_path", dcf_path, "config/master.dcf");
    pnh.param<std::string>("canopen_joints_path", joints_path, "config/joints.yaml");
    dcf_path = MakeAbsolutePath(dcf_path);
    joints_path = MakeAbsolutePath(joints_path);

    if (!FileExists(dcf_path)) {
        ROS_FATAL("[hybrid] canopen dcf not found: %s", dcf_path.c_str());
        return 1;
    }
    if (!FileExists(joints_path)) {
        ROS_FATAL("[hybrid] canopen joints.yaml not found: %s", joints_path.c_str());
        return 1;
    }

    canopen_hw::CanopenMasterConfig master_cfg;
    master_cfg.master_dcf_path = dcf_path;
    std::string error;
    if (!canopen_hw::LoadJointsYaml(joints_path, &error, &master_cfg)) {
        ROS_FATAL("[hybrid] LoadJointsYaml failed: %s", error.c_str());
        return 1;
    }

    std::vector<std::string> canopen_joint_names;
    canopen_joint_names.reserve(master_cfg.joints.size());
    for (const auto& jcfg : master_cfg.joints) {
        canopen_joint_names.push_back(jcfg.name);
    }

    canopen_hw::LifecycleManager lifecycle;
    if (!lifecycle.Configure(master_cfg)) {
        ROS_FATAL("[hybrid] CANopen LifecycleManager::Configure failed");
        return 1;
    }

    canopen_hw::CanopenRobotHwRos canopen_robot_hw(lifecycle.robot_hw(),
                                                    canopen_joint_names);
    for (std::size_t i = 0; i < master_cfg.joints.size(); ++i) {
        canopen_robot_hw.SetMode(i, master_cfg.joints[i].default_mode);
    }

    // ======================================================================
    // 2. can_driver 侧：CanDriverHW (init 在 HybridRobotHW::init 中调用)
    // ======================================================================
    CanDriverHW can_hw;

    // ======================================================================
    // 3. HybridRobotHW — 组合 + 注册到共享 controller_manager
    // ======================================================================
    eyou_ros1_master::HybridRobotHW hybrid_hw(&can_hw, &canopen_robot_hw);
    if (!hybrid_hw.init(nh, pnh)) {
        ROS_FATAL("[hybrid] HybridRobotHW::init failed");
        return 1;
    }

    controller_manager::ControllerManager cm(&hybrid_hw, nh);

    // ======================================================================
    // 4. 协调器 + 服务网关
    // ======================================================================
    std::mutex loop_mtx;
    canopen_hw::OperationalCoordinator canopen_coord(
        lifecycle.master(), lifecycle.shared_state(), master_cfg.joints.size());
    canopen_coord.SetConfigured();

    eyou_ros1_master::HybridOperationalCoordinator hybrid_coord(
        &can_hw.operationalCoordinator(), &canopen_coord);

    ros::NodeHandle can_driver_pnh(pnh, "can_driver_node");
    eyou_ros1_master::HybridServiceGateway service_gateway(
        pnh, can_driver_pnh, &hybrid_coord, &loop_mtx);
    MotorMaintenanceService can_driver_maintenance_service;
    can_hw.configureMotorMaintenanceService(can_driver_maintenance_service);
    MotorMaintenanceService::AdvertiseOptions can_driver_service_options;
    can_driver_service_options.motorCommand = false;
    can_driver_service_options.setZeroLimit = true;
    can_driver_maintenance_service.initialize(pnh, can_driver_service_options);

    // ======================================================================
    // 5. CANopen 辅助服务（set_mode、set_zero、软限位）
    // ======================================================================
    canopen_hw::CanopenAuxServices canopen_aux(
        &pnh, &canopen_robot_hw, &canopen_coord, &master_cfg,
        lifecycle.master(), &loop_mtx);
    std::unique_ptr<eyou_ros1_master::HybridModeRouter> hybrid_mode_router;
    try {
        hybrid_mode_router = std::make_unique<eyou_ros1_master::HybridModeRouter>(
            pnh, can_driver_pnh, master_cfg, &canopen_aux,
            &can_driver_maintenance_service);
    } catch (const std::exception& e) {
        ROS_FATAL("[hybrid] failed to initialize mode router: %s", e.what());
        return 1;
    }
    service_gateway.SetPostInitHook(
        [&](std::string* detail) { return canopen_aux.ApplySoftLimitAll(detail); });

    ros::Publisher joint_runtime_pub =
        pnh.advertise<Eyou_ROS1_Master::JointRuntimeStateArray>("joint_runtime_states", 1);

    // ======================================================================
    // 6. IP 轨迹执行器（可选）
    // ======================================================================
    bool use_ip_executor = false;
    bool use_servo_bridge = false;
    double ip_executor_rate_hz = master_cfg.loop_hz;
    std::string ip_executor_action_ns =
        "/arm_position_controller/follow_joint_trajectory";
    pnh.param("use_ip_executor", use_ip_executor, false);
    pnh.param("use_servo_bridge", use_servo_bridge, false);
    pnh.param("ip_executor_rate_hz", ip_executor_rate_hz, ip_executor_rate_hz);
    pnh.param("ip_executor_action_ns", ip_executor_action_ns,
              ip_executor_action_ns);

    std::unique_ptr<eyou_ros1_master::HybridJointTargetExecutor> joint_target_executor;
    std::unique_ptr<eyou_ros1_master::HybridFollowJointTrajectoryExecutor> ip_executor;
    std::unique_ptr<eyou_ros1_master::HybridServoBridge> servo_bridge;
    if (use_ip_executor || use_servo_bridge) {
        eyou_ros1_master::HybridFollowJointTrajectoryExecutor::Config exec_cfg;
        std::string exec_cfg_error;
        if (!eyou_ros1_master::BuildHybridIpExecutorConfigFromParams(
                master_cfg, can_driver_pnh, ip_executor_action_ns,
                ip_executor_rate_hz, &exec_cfg, &exec_cfg_error)) {
            ROS_FATAL("[hybrid] failed to build IP executor config: %s",
                      exec_cfg_error.c_str());
            return 1;
        }

        joint_target_executor =
            std::make_unique<eyou_ros1_master::HybridJointTargetExecutor>(
                &hybrid_hw, &loop_mtx, MakeJointTargetExecutorConfig(exec_cfg));
        if (!joint_target_executor->valid()) {
            ROS_FATAL("[hybrid] failed to initialize joint target executor: %s",
                      joint_target_executor->config_error().c_str());
            return 1;
        }

        if (use_ip_executor) {
            ip_executor =
                std::make_unique<eyou_ros1_master::HybridFollowJointTrajectoryExecutor>(
                    &pnh, &hybrid_hw, &loop_mtx, joint_target_executor.get(),
                    exec_cfg);
            if (!ip_executor->config_valid()) {
                ROS_FATAL("[hybrid] failed to initialize action executor: %s",
                          ip_executor->config_error().c_str());
                return 1;
            }
        }

        if (use_servo_bridge) {
            servo_bridge = std::make_unique<eyou_ros1_master::HybridServoBridge>(
                &pnh, joint_target_executor.get(), MakeServoBridgeConfig(exec_cfg, pnh));
            if (!servo_bridge->valid()) {
                ROS_FATAL("[hybrid] failed to initialize servo bridge: %s",
                          servo_bridge->config_error().c_str());
                return 1;
            }
        }
    }

    // ======================================================================
    // 7. Hybrid 启动序列（统一走 facade authority）
    // ======================================================================
    {
        std::string auto_start_error;
        if (!eyou_ros1_master::RunHybridAutoStartupFromParams(service_gateway,
                                                              hybrid_coord,
                                                              pnh,
                                                              &auto_start_error)) {
            ROS_FATAL("[hybrid] auto startup failed: %s", auto_start_error.c_str());
            return 1;
        }
    }

    // ======================================================================
    // 8. 主循环
    // ======================================================================
    double loop_hz = master_cfg.loop_hz;
    pnh.param("loop_hz", loop_hz, loop_hz);

    ros::AsyncSpinner spinner(2);
    spinner.start();
    ros::Rate rate(loop_hz);
    ros::Time last_time = ros::Time::now();
    ros::Time last_joint_runtime_pub = ros::Time(0);

    while (ros::ok()) {
        const ros::Time now = ros::Time::now();
        const ros::Duration period = now - last_time;
        last_time = now;

        {
            std::lock_guard<std::mutex> lk(loop_mtx);

            // CANopen 侧需要显式驱动反馈与 intent
            canopen_coord.UpdateFromFeedback();
            canopen_coord.ComputeIntents();

            // can_driver 侧的 UpdateFromFeedback 在其 write() 内部自行调用

            hybrid_hw.read(now, period);
            cm.update(now, period);
            if (ip_executor) {
                ip_executor->update(now, period);
            }
            if (servo_bridge) {
                servo_bridge->update(now);
            }
            if (joint_target_executor) {
                joint_target_executor->update(period);
            }
            hybrid_hw.write(now, period);
        }

        if ((now - last_joint_runtime_pub).toSec() >= 0.1) {
            Eyou_ROS1_Master::JointRuntimeStateArray msg;
            msg.header.stamp = now;

            const auto global_mode = hybrid_coord.mode();
            const std::string lifecycle_state =
                can_driver::SystemOpModeName(global_mode);
            const auto can_driver_states = can_hw.snapshotJointRuntimeStates();
            msg.states.reserve(can_driver_states.size() + master_cfg.joints.size());

            for (const auto& state : can_driver_states) {
                Eyou_ROS1_Master::JointRuntimeState item;
                item.joint_name = state.jointName;
                item.backend = "can_driver";
                item.lifecycle_state = lifecycle_state;
                item.online = state.deviceReady && state.feedbackFresh;
                item.enabled = state.enabled;
                item.fault = state.fault;
                msg.states.push_back(std::move(item));
            }

            const auto canopen_snapshot = lifecycle.shared_state()->Snapshot();
            for (std::size_t i = 0; i < master_cfg.joints.size(); ++i) {
                Eyou_ROS1_Master::JointRuntimeState item;
                item.joint_name = master_cfg.joints[i].name;
                item.backend = "canopen";
                const auto& fb = canopen_snapshot.feedback[i];
                const bool enabled =
                    fb.is_operational ||
                    fb.state == canopen_hw::CiA402State::OperationEnabled;
                item.lifecycle_state = lifecycle_state;
                item.online = !fb.heartbeat_lost;
                item.enabled = enabled;
                item.fault = fb.is_fault;
                msg.states.push_back(std::move(item));
            }

            joint_runtime_pub.publish(msg);
            last_joint_runtime_pub = now;
        }

        rate.sleep();
    }

    spinner.stop();
    {
        std::lock_guard<std::mutex> lk(loop_mtx);
        const auto shutdown_result = hybrid_coord.RequestShutdown(false);
        if (!shutdown_result.ok) {
            ROS_ERROR("[hybrid] shutdown completed with errors: %s",
                      shutdown_result.message.c_str());
        }
        lifecycle.Shutdown();
    }
    return 0;
}
