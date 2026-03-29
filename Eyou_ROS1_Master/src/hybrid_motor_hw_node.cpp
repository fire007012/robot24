// hybrid_motor_hw_node.cpp — 统一外观节点
//
// 在单进程内同时托管 CanDriverHW (can_driver) 和 CanopenRobotHwRos
// (Eyou_Canopen_Master) 两套后端，共享一个 controller_manager，
// 对外暴露统一服务接口和 /joint_states。

#include <filesystem>
#include <mutex>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <controller_manager/controller_manager.h>

#include "Eyou_ROS1_Master/hybrid_robot_hw.hpp"
#include "Eyou_ROS1_Master/hybrid_operational_coordinator.hpp"
#include "Eyou_ROS1_Master/hybrid_service_gateway.hpp"

#include "can_driver/CanDriverHW.h"
#include "canopen_hw/canopen_aux_services.hpp"
#include "canopen_hw/canopen_robot_hw_ros.hpp"
#include "canopen_hw/canopen_startup_sequence.hpp"
#include "canopen_hw/controllers/ip_follow_joint_trajectory_executor.hpp"
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

    eyou_ros1_master::HybridServiceGateway service_gateway(
        pnh, &hybrid_coord, &loop_mtx);

    // ======================================================================
    // 5. CANopen 辅助服务（set_mode、set_zero、软限位）
    // ======================================================================
    canopen_hw::CanopenAuxServices canopen_aux(
        &pnh, &canopen_robot_hw, &canopen_coord, &master_cfg,
        lifecycle.master(), &loop_mtx);

    // ======================================================================
    // 6. IP 轨迹执行器（可选）
    // ======================================================================
    bool use_ip_executor = false;
    double ip_executor_rate_hz = master_cfg.loop_hz;
    std::string ip_executor_action_ns =
        "arm_position_controller/follow_joint_trajectory";
    pnh.param("use_ip_executor", use_ip_executor, false);
    pnh.param("ip_executor_rate_hz", ip_executor_rate_hz, ip_executor_rate_hz);
    pnh.param("ip_executor_action_ns", ip_executor_action_ns,
              ip_executor_action_ns);

    std::unique_ptr<canopen_hw::IpFollowJointTrajectoryExecutor> ip_executor;
    if (use_ip_executor) {
        canopen_hw::IpFollowJointTrajectoryExecutor::Config exec_cfg;
        exec_cfg.joint_names.clear();
        exec_cfg.joint_indices.clear();
        exec_cfg.max_velocities.clear();
        exec_cfg.max_accelerations.clear();
        exec_cfg.max_jerks.clear();
        exec_cfg.goal_tolerances.clear();
        exec_cfg.action_ns = ip_executor_action_ns;
        exec_cfg.command_rate_hz = ip_executor_rate_hz;

        for (std::size_t i = 0; i < master_cfg.joints.size(); ++i) {
            const auto& jcfg = master_cfg.joints[i];
            exec_cfg.joint_names.push_back(jcfg.name);
            exec_cfg.joint_indices.push_back(i);
            exec_cfg.max_velocities.push_back(jcfg.ip_max_velocity);
            exec_cfg.max_accelerations.push_back(jcfg.ip_max_acceleration);
            exec_cfg.max_jerks.push_back(jcfg.ip_max_jerk);
            exec_cfg.goal_tolerances.push_back(jcfg.ip_goal_tolerance);
        }

        ip_executor = std::make_unique<canopen_hw::IpFollowJointTrajectoryExecutor>(
            &pnh, &canopen_robot_hw, &loop_mtx, std::move(exec_cfg));
    }

    // ======================================================================
    // 7. CANopen 启动序列（auto_init / auto_enable / auto_release）
    // ======================================================================
    {
        std::lock_guard<std::mutex> lk(loop_mtx);
        if (!canopen_hw::CanopenStartupSequence::Run(canopen_coord, canopen_aux, pnh)) {
            ROS_FATAL("[hybrid] CANopen startup sequence failed");
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
            hybrid_hw.write(now, period);
        }

        rate.sleep();
    }

    spinner.stop();
    {
        std::lock_guard<std::mutex> lk(loop_mtx);
        can_hw.operationalCoordinator().RequestShutdown(false);
        lifecycle.Shutdown();
    }
    return 0;
}
