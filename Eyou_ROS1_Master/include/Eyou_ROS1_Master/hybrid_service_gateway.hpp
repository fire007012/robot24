#pragma once

#include <functional>
#include <mutex>
#include <string>

#include <ros/ros.h>
#include <std_srvs/Trigger.h>

#include "Eyou_ROS1_Master/hybrid_operational_coordinator.hpp"

namespace eyou_ros1_master {

// 注册 7 个统一 ROS 服务（std_srvs/Trigger），
// 转发到 HybridOperationalCoordinator。
// init 所需的 device / loopback 参数通过 rosparam 读取。
class HybridServiceGateway {
public:
    HybridServiceGateway(ros::NodeHandle& pnh,
                         HybridOperationalCoordinator* coordinator,
                         std::mutex* loop_mtx);
    void SetPostInitHook(std::function<bool(std::string*)> hook);

private:
    bool OnInit(std_srvs::Trigger::Request& req,
                std_srvs::Trigger::Response& res);
    bool OnEnable(std_srvs::Trigger::Request& req,
                  std_srvs::Trigger::Response& res);
    bool OnDisable(std_srvs::Trigger::Request& req,
                   std_srvs::Trigger::Response& res);
    bool OnHalt(std_srvs::Trigger::Request& req,
                std_srvs::Trigger::Response& res);
    bool OnResume(std_srvs::Trigger::Request& req,
                  std_srvs::Trigger::Response& res);
    bool OnRecover(std_srvs::Trigger::Request& req,
                   std_srvs::Trigger::Response& res);
    bool OnShutdown(std_srvs::Trigger::Request& req,
                    std_srvs::Trigger::Response& res);

    HybridOperationalCoordinator* coordinator_;
    std::mutex* loop_mtx_;
    std::function<bool(std::string*)> post_init_hook_;

    ros::ServiceServer init_srv_;
    ros::ServiceServer enable_srv_;
    ros::ServiceServer disable_srv_;
    ros::ServiceServer halt_srv_;
    ros::ServiceServer resume_srv_;
    ros::ServiceServer recover_srv_;
    ros::ServiceServer shutdown_srv_;
};

}  // namespace eyou_ros1_master
