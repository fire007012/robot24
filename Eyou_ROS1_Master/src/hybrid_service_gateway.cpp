#include "Eyou_ROS1_Master/hybrid_service_gateway.hpp"

#include <ros/ros.h>

namespace eyou_ros1_master {

void HybridServiceGateway::SetPostInitHook(std::function<bool(std::string*)> hook) {
    post_init_hook_ = std::move(hook);
}

HybridServiceGateway::HybridServiceGateway(
    ros::NodeHandle& pnh,
    HybridOperationalCoordinator* coordinator,
    std::mutex* loop_mtx,
    bool advertise_services)
    : coordinator_(coordinator), loop_mtx_(loop_mtx) {
    if (advertise_services) {
        init_srv_     = pnh.advertiseService("init",     &HybridServiceGateway::OnInit,     this);
        enable_srv_   = pnh.advertiseService("enable",   &HybridServiceGateway::OnEnable,   this);
        disable_srv_  = pnh.advertiseService("disable",  &HybridServiceGateway::OnDisable,  this);
        halt_srv_     = pnh.advertiseService("halt",     &HybridServiceGateway::OnHalt,     this);
        resume_srv_   = pnh.advertiseService("resume",   &HybridServiceGateway::OnResume,   this);
        recover_srv_  = pnh.advertiseService("recover",  &HybridServiceGateway::OnRecover,  this);
        shutdown_srv_ = pnh.advertiseService("shutdown", &HybridServiceGateway::OnShutdown, this);

        ROS_INFO("[HybridServiceGateway] 7 services registered under %s",
                 pnh.getNamespace().c_str());
    }
}

bool HybridServiceGateway::RunInitSequence(const std::string& device,
                                           bool loopback,
                                           std::string* message,
                                           bool* already_initialized) {
    if (message != nullptr) {
        message->clear();
    }
    if (already_initialized != nullptr) {
        *already_initialized = false;
    }

    if (coordinator_ == nullptr || loop_mtx_ == nullptr) {
        if (message != nullptr) {
            *message = "service gateway not initialized";
        }
        return false;
    }

    std::lock_guard<std::mutex> lk(*loop_mtx_);
    auto r = coordinator_->RequestInit(device, loopback);
    if (!r.ok) {
        if (message != nullptr) {
            *message = r.message.empty() ? "init failed" : r.message;
        }
        return false;
    }

    const bool already = r.already;
    if (already_initialized != nullptr) {
        *already_initialized = already;
    }

    if (!already && post_init_hook_) {
        std::string hook_detail;
        if (!post_init_hook_(&hook_detail)) {
            const auto shutdown_r = coordinator_->RequestShutdown(/*force=*/false);
            if (message != nullptr) {
                *message = hook_detail.empty() ? "post-init hook failed" : hook_detail;
                if (!shutdown_r.ok) {
                    *message += "; rollback shutdown failed: " + shutdown_r.message;
                }
            }
            return false;
        }
    }

    if (message != nullptr) {
        *message = already ? "already initialized" : "initialized (armed)";
    }
    return true;
}

bool HybridServiceGateway::OnInit(std_srvs::Trigger::Request&,
                                  std_srvs::Trigger::Response& res) {
    std::string device;
    bool loopback = false;
    ros::NodeHandle pnh("~");
    pnh.param<std::string>("can_device", device, "can0");
    pnh.param<bool>("loopback", loopback, false);
    bool already = false;
    res.success = RunInitSequence(device, loopback, &res.message, &already);
    return true;
}

bool HybridServiceGateway::OnEnable(std_srvs::Trigger::Request&,
                                    std_srvs::Trigger::Response& res) {
    std::lock_guard<std::mutex> lk(*loop_mtx_);
    auto r = coordinator_->RequestEnable();
    res.success = r.ok;
    res.message = r.message;
    return true;
}

bool HybridServiceGateway::OnDisable(std_srvs::Trigger::Request&,
                                     std_srvs::Trigger::Response& res) {
    std::lock_guard<std::mutex> lk(*loop_mtx_);
    auto r = coordinator_->RequestDisable();
    res.success = r.ok;
    res.message = r.message;
    return true;
}

bool HybridServiceGateway::OnHalt(std_srvs::Trigger::Request&,
                                  std_srvs::Trigger::Response& res) {
    std::lock_guard<std::mutex> lk(*loop_mtx_);
    auto r = coordinator_->RequestHalt();
    res.success = r.ok;
    res.message = r.message;
    return true;
}

bool HybridServiceGateway::OnResume(std_srvs::Trigger::Request&,
                                    std_srvs::Trigger::Response& res) {
    std::lock_guard<std::mutex> lk(*loop_mtx_);
    auto r = coordinator_->RequestRelease();  // resume = Armed → Running
    res.success = r.ok;
    res.message = r.message;
    return true;
}

bool HybridServiceGateway::OnRecover(std_srvs::Trigger::Request&,
                                     std_srvs::Trigger::Response& res) {
    std::lock_guard<std::mutex> lk(*loop_mtx_);
    auto r = coordinator_->RequestRecover();
    res.success = r.ok;
    res.message = r.message;
    return true;
}

bool HybridServiceGateway::OnShutdown(std_srvs::Trigger::Request&,
                                      std_srvs::Trigger::Response& res) {
    std::lock_guard<std::mutex> lk(*loop_mtx_);
    auto r = coordinator_->RequestShutdown(/*force=*/false);
    res.success = r.ok;
    res.message = r.message;
    return true;
}

}  // namespace eyou_ros1_master
