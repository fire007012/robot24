#pragma once

#include <string>

#include <xmlrpcpp/XmlRpcValue.h>

#include "canopen_hw/canopen_master.hpp"
#include "canopen_hw/controllers/ip_follow_joint_trajectory_executor.hpp"

namespace ros {
class NodeHandle;
}

namespace eyou_ros1_master {

bool BuildHybridIpExecutorConfig(
    const canopen_hw::CanopenMasterConfig& canopen_config,
    const XmlRpc::XmlRpcValue& can_driver_joint_list,
    const std::string& action_ns,
    double command_rate_hz,
    canopen_hw::IpFollowJointTrajectoryExecutor::Config* config,
    std::string* error);

bool BuildHybridIpExecutorConfigFromParams(
    const canopen_hw::CanopenMasterConfig& canopen_config,
    const ros::NodeHandle& can_driver_pnh,
    const std::string& action_ns,
    double command_rate_hz,
    canopen_hw::IpFollowJointTrajectoryExecutor::Config* config,
    std::string* error);

}  // namespace eyou_ros1_master
