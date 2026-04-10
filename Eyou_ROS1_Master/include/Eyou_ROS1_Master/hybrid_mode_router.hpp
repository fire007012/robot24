#pragma once

#include <cstdint>
#include <map>
#include <string>

#include <ros/ros.h>
#include <xmlrpcpp/XmlRpcValue.h>

#include "can_driver/motor_maintenance_service.hpp"
#include "canopen_hw/canopen_aux_services.hpp"
#include "canopen_hw/canopen_master.hpp"
#include "Eyou_ROS1_Master/SetJointMode.h"

namespace eyou_ros1_master {

struct HybridModeMapping {
    bool has_canopen{false};
    int canopen_mode{0};
    bool has_can_driver{false};
    int can_driver_mode{0};
};

struct HybridModeJointTarget {
    enum class Backend {
        kCanopen,
        kCanDriver,
    };

    Backend backend{Backend::kCanopen};
    std::size_t axis_index{0};
    uint16_t motor_id{0};
};

bool LoadHybridModeMappings(const ros::NodeHandle& pnh,
                            std::map<std::string, HybridModeMapping>* mappings,
                            std::string* error);

bool BuildHybridModeJointTargets(
    const canopen_hw::CanopenMasterConfig& canopen_config,
    const XmlRpc::XmlRpcValue& can_driver_joint_list,
    std::map<std::string, HybridModeJointTarget>* joint_targets,
    std::string* error);

class HybridModeRouter {
public:
    HybridModeRouter(ros::NodeHandle& pnh,
                     const ros::NodeHandle& can_driver_pnh,
                     const canopen_hw::CanopenMasterConfig& canopen_config,
                     canopen_hw::CanopenAuxServices* canopen_aux,
                     MotorMaintenanceService* can_driver_maintenance,
                     bool advertise_service = true);

    bool HandleSetJointMode(const std::string& joint_name,
                            const std::string& mode_name,
                            std::string* backend,
                            int* mapped_mode,
                            std::string* message) const;

private:
    bool OnSetJointMode(Eyou_ROS1_Master::SetJointMode::Request& req,
                        Eyou_ROS1_Master::SetJointMode::Response& res);

    canopen_hw::CanopenAuxServices* canopen_aux_{nullptr};
    MotorMaintenanceService* can_driver_maintenance_{nullptr};
    std::map<std::string, HybridModeMapping> mappings_;
    std::map<std::string, HybridModeJointTarget> joint_targets_;
    ros::ServiceServer set_joint_mode_srv_;
};

}  // namespace eyou_ros1_master
