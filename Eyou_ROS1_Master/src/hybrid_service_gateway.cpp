#include "Eyou_ROS1_Master/hybrid_service_gateway.hpp"

#include <algorithm>
#include <dirent.h>

#include <fstream>
#include <set>
#include <sstream>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <xmlrpcpp/XmlRpcValue.h>

namespace eyou_ros1_master {

namespace {

std::string TrimCopy(const std::string& input) {
    std::size_t begin = 0U;
    while (begin < input.size() && std::isspace(static_cast<unsigned char>(input[begin])) != 0) {
        ++begin;
    }
    std::size_t end = input.size();
    while (end > begin && std::isspace(static_cast<unsigned char>(input[end - 1U])) != 0) {
        --end;
    }
    return input.substr(begin, end - begin);
}

bool ReadTextFile(const std::string& path, std::string* content) {
    if (content == nullptr) {
        return false;
    }
    std::ifstream input(path.c_str());
    if (!input.is_open()) {
        return false;
    }
    std::ostringstream oss;
    oss << input.rdbuf();
    *content = TrimCopy(oss.str());
    return true;
}

std::vector<std::string> DetectSocketCanDevices(bool only_up) {
    std::vector<std::string> devices;
    DIR* dir = opendir("/sys/class/net");
    if (dir == nullptr) {
        return devices;
    }

    struct dirent* entry = nullptr;
    while ((entry = readdir(dir)) != nullptr) {
        if (entry->d_name == nullptr) {
            continue;
        }
        const std::string name(entry->d_name);
        if (name == "." || name == "..") {
            continue;
        }
        if (name.rfind("can", 0) != 0) {
            continue;
        }

        std::string type;
        if (!ReadTextFile("/sys/class/net/" + name + "/type", &type) || type != "280") {
            continue;
        }

        if (only_up) {
            std::string operstate;
            if (!ReadTextFile("/sys/class/net/" + name + "/operstate", &operstate) ||
                operstate != "up") {
                continue;
            }
        }
        devices.push_back(name);
    }

    closedir(dir);
    std::sort(devices.begin(), devices.end());
    return devices;
}

void CollectCanDevices(const XmlRpc::XmlRpcValue& joint_list,
                       std::set<std::string>* devices,
                       std::vector<std::string>* non_ecb_devices,
                       std::vector<std::string>* ecb_devices) {
    if (devices != nullptr) {
        devices->clear();
    }
    if (non_ecb_devices != nullptr) {
        non_ecb_devices->clear();
    }
    if (ecb_devices != nullptr) {
        ecb_devices->clear();
    }

    std::set<std::string> local_devices;
    for (int i = 0; i < joint_list.size(); ++i) {
        if (joint_list[i].getType() != XmlRpc::XmlRpcValue::TypeStruct ||
            !joint_list[i].hasMember("can_device")) {
            continue;
        }
        const std::string can_device = static_cast<std::string>(joint_list[i]["can_device"]);
        if (!can_device.empty()) {
            local_devices.insert(can_device);
        }
    }

    for (const auto& candidate : local_devices) {
        if (candidate.rfind("ecb://", 0) == 0) {
            if (ecb_devices != nullptr) {
                ecb_devices->push_back(candidate);
            }
        } else {
            if (non_ecb_devices != nullptr) {
                non_ecb_devices->push_back(candidate);
            }
        }
    }

    if (devices != nullptr) {
        *devices = local_devices;
    }
}

bool TryAutoRemapSingleSocketCan(const ros::NodeHandle& can_driver_pnh,
                                 XmlRpc::XmlRpcValue* joint_list,
                                 std::string* primary_device) {
    if (joint_list == nullptr) {
        return false;
    }

    bool auto_remap = true;
    can_driver_pnh.param("auto_remap_single_socketcan_device", auto_remap, true);
    if (!auto_remap) {
        return false;
    }

    std::set<std::string> devices;
    std::vector<std::string> non_ecb_devices;
    std::vector<std::string> ecb_devices;
    CollectCanDevices(*joint_list, &devices, &non_ecb_devices, &ecb_devices);

    std::vector<std::string> configured_socketcan;
    for (const auto& candidate : non_ecb_devices) {
        if (candidate.rfind("can", 0) == 0) {
            configured_socketcan.push_back(candidate);
        }
    }
    if (configured_socketcan.size() != 1U) {
        return false;
    }

    auto detected = DetectSocketCanDevices(/*only_up=*/true);
    if (detected.empty()) {
        detected = DetectSocketCanDevices(/*only_up=*/false);
    }
    if (detected.size() != 1U) {
        return false;
    }

    const std::string from = configured_socketcan.front();
    const std::string to = detected.front();
    if (from == to) {
        return false;
    }

    bool changed = false;
    for (int i = 0; i < joint_list->size(); ++i) {
        if ((*joint_list)[i].getType() != XmlRpc::XmlRpcValue::TypeStruct ||
            !(*joint_list)[i].hasMember("can_device")) {
            continue;
        }
        const std::string can_device = static_cast<std::string>((*joint_list)[i]["can_device"]);
        if (can_device == from) {
            (*joint_list)[i]["can_device"] = XmlRpc::XmlRpcValue(to);
            changed = true;
        }
    }

    if (!changed) {
        return false;
    }

    can_driver_pnh.setParam("joints", *joint_list);
    if (primary_device != nullptr && *primary_device == from) {
        *primary_device = to;
    }

    ROS_WARN("[HybridServiceGateway] Auto-remapped can_driver joints can_device from '%s' to detected '%s'.",
             from.c_str(), to.c_str());
    return true;
}

bool ResolveCanDriverInitRequest(const ros::NodeHandle& can_driver_pnh,
                                 std::string* device_spec,
                                 bool* loopback,
                                 std::string* error) {
    if (device_spec != nullptr) {
        device_spec->clear();
    }
    if (loopback != nullptr) {
        *loopback = false;
    }
    if (error != nullptr) {
        error->clear();
    }

    XmlRpc::XmlRpcValue joint_list;
    if (!can_driver_pnh.getParam("joints", joint_list)) {
        if (error != nullptr) {
            *error = "can_driver joints config is missing";
        }
        return false;
    }
    if (joint_list.getType() != XmlRpc::XmlRpcValue::TypeArray) {
        if (error != nullptr) {
            *error = "can_driver joints config is not an array";
        }
        return false;
    }

    std::set<std::string> devices;
    std::vector<std::string> non_ecb_devices;
    std::vector<std::string> ecb_devices;
    CollectCanDevices(joint_list, &devices, &non_ecb_devices, &ecb_devices);

    if (devices.empty()) {
        if (error != nullptr) {
            *error = "can_driver joints config does not define any can_device";
        }
        return false;
    }
    bool enable_ecb_control = false;
    can_driver_pnh.param("enable_ecb_control", enable_ecb_control, false);

    bool optional_ecb_init = true;
    can_driver_pnh.param("optional_ecb_init", optional_ecb_init, true);

    std::string primary_device;
    can_driver_pnh.param<std::string>("primary_can_device", primary_device, std::string(""));

    bool auto_detect_primary = true;
    can_driver_pnh.param("auto_detect_primary_can_device", auto_detect_primary, true);

    if (TryAutoRemapSingleSocketCan(can_driver_pnh, &joint_list, &primary_device)) {
        CollectCanDevices(joint_list, &devices, &non_ecb_devices, &ecb_devices);
    }

    std::vector<std::string> selected_specs;

    if (non_ecb_devices.empty()) {
        if (ecb_devices.empty() || !enable_ecb_control) {
            if (error != nullptr) {
                *error = ecb_devices.empty()
                             ? "can_driver joints config has no non-ECB can_device for hybrid primary init"
                             : "can_driver joints config has only ECB devices; set enable_ecb_control=true";
            }
            return false;
        }
        ROS_INFO("[HybridServiceGateway] No non-ECB device configured; use ECB-only lifecycle init.");
    } else {
        if (primary_device.empty() && auto_detect_primary) {
            auto detected = DetectSocketCanDevices(/*only_up=*/true);
            if (detected.empty()) {
                detected = DetectSocketCanDevices(/*only_up=*/false);
            }
            for (const auto& candidate : detected) {
                for (const auto& configured : non_ecb_devices) {
                    if (configured == candidate) {
                        primary_device = configured;
                        ROS_INFO("[HybridServiceGateway] Auto-selected primary CAN device: %s",
                                 primary_device.c_str());
                        break;
                    }
                }
                if (!primary_device.empty()) {
                    break;
                }
            }
        }

        if (!primary_device.empty()) {
            bool found = false;
            for (const auto& candidate : non_ecb_devices) {
                if (candidate == primary_device) {
                    found = true;
                    break;
                }
            }
            if (!found) {
                ROS_WARN("[HybridServiceGateway] primary_can_device='%s' not found in non-ECB devices, fallback to '%s'.",
                         primary_device.c_str(),
                         non_ecb_devices.front().c_str());
                primary_device = non_ecb_devices.front();
            }
        } else {
            primary_device = non_ecb_devices.front();
        }

        selected_specs.push_back(primary_device);

        for (const auto& candidate : non_ecb_devices) {
            if (candidate == primary_device) {
                continue;
            }
            selected_specs.push_back(candidate);
        }
    }

    if (enable_ecb_control) {
        for (const auto& candidate : ecb_devices) {
            if (optional_ecb_init) {
                selected_specs.push_back("optional:" + candidate);
            } else {
                selected_specs.push_back(candidate);
            }
        }
    } else if (!ecb_devices.empty()) {
        std::string detail;
        for (std::size_t i = 0; i < ecb_devices.size(); ++i) {
            if (i > 0U) {
                detail += ", ";
            }
            detail += ecb_devices[i];
        }
        ROS_WARN("[HybridServiceGateway] ECB devices configured but enable_ecb_control=false; skip ECB init: %s",
                 detail.c_str());
    }

    if (selected_specs.empty()) {
        if (error != nullptr) {
            *error = "no can_driver device selected for init";
        }
        return false;
    }

    std::ostringstream oss;
    for (std::size_t i = 0; i < selected_specs.size(); ++i) {
        if (i > 0U) {
            oss << ",";
        }
        oss << selected_specs[i];
    }

    if (device_spec != nullptr) {
        *device_spec = oss.str();
    }

    bool init_loopback = false;
    can_driver_pnh.param("init_loopback", init_loopback, false);
    if (loopback != nullptr) {
        *loopback = init_loopback;
    }
    return true;
}

HybridOperationalCoordinator::Result RequestResumeWithRetry(
    HybridOperationalCoordinator* coordinator,
    std::mutex* loop_mtx,
    const ros::NodeHandle& pnh) {
    if (coordinator == nullptr || loop_mtx == nullptr) {
        return {false, "service gateway not initialized", false};
    }

    double retry_timeout_sec = 3.0;
    double retry_interval_sec = 0.1;
    pnh.param("auto_release_timeout_sec", retry_timeout_sec, retry_timeout_sec);
    pnh.param("auto_release_retry_interval_sec", retry_interval_sec, retry_interval_sec);

    const ros::WallDuration retry_interval(retry_interval_sec > 0.0 ? retry_interval_sec : 0.1);
    const ros::WallTime deadline =
        ros::WallTime::now() + ros::WallDuration(std::max(0.0, retry_timeout_sec));

    HybridOperationalCoordinator::Result last_result;
    while (true) {
        {
            std::lock_guard<std::mutex> lk(*loop_mtx);
            last_result = coordinator->RequestRelease();
            if (last_result.ok ||
                coordinator->mode() == can_driver::SystemOpMode::Running) {
                return last_result.ok
                           ? last_result
                           : HybridOperationalCoordinator::Result{true, "already running", true};
            }
        }

        if (ros::WallTime::now() >= deadline) {
            break;
        }
        retry_interval.sleep();
    }

    return last_result;
}

}  // namespace

void HybridServiceGateway::SetPostInitHook(std::function<bool(std::string*)> hook) {
    post_init_hook_ = std::move(hook);
}

HybridServiceGateway::HybridServiceGateway(
    ros::NodeHandle& pnh,
    const ros::NodeHandle& can_driver_pnh,
    HybridOperationalCoordinator* coordinator,
    std::mutex* loop_mtx,
    bool advertise_services)
    : coordinator_(coordinator),
      loop_mtx_(loop_mtx),
            pnh_(pnh),
      can_driver_pnh_(can_driver_pnh) {
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

bool HybridServiceGateway::RunConfiguredInitSequence(std::string* message,
                                                     bool* already_initialized) {
    std::string device_spec;
    bool loopback = false;
    std::string error;
    if (!ResolveCanDriverInitRequest(can_driver_pnh_, &device_spec, &loopback, &error)) {
        if (message != nullptr) {
            *message = error;
        }
        if (already_initialized != nullptr) {
            *already_initialized = false;
        }
        return false;
    }
    return RunInitSequence(device_spec, loopback, message, already_initialized);
}

bool HybridServiceGateway::OnInit(std_srvs::Trigger::Request&,
                                  std_srvs::Trigger::Response& res) {
    bool already = false;
    res.success = RunConfiguredInitSequence(&res.message, &already);
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
    auto r = RunResumeSequence(&res.message);
    res.success = r;
    return true;
}

bool HybridServiceGateway::RunResumeSequence(std::string* message) {
    auto r = RequestResumeWithRetry(coordinator_, loop_mtx_, pnh_);
    if (message != nullptr) {
        *message = r.message;
    }
    return r.ok;
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
