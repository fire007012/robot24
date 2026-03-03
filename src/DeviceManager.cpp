#include "can_driver/DeviceManager.h"

#include <ros/ros.h>

bool DeviceManager::ensureTransport(const std::string &device, bool loopback)
{
    std::unique_lock<std::shared_mutex> lock(mutex_);
    auto it = transports_.find(device);
    if (it != transports_.end()) {
        return true;
    }

    auto transport = std::make_shared<SocketCanController>();
    if (!transport->initialize(device, loopback)) {
        ROS_ERROR("[CanDriverHW] Failed to initialize CAN device '%s'.", device.c_str());
        return false;
    }

    transports_[device] = transport;
    deviceCmdMutexes_[device] = std::make_shared<std::mutex>();
    ROS_INFO("[CanDriverHW] Opened CAN device '%s'.", device.c_str());
    return true;
}

bool DeviceManager::ensureProtocol(const std::string &device, CanType type)
{
    std::unique_lock<std::shared_mutex> lock(mutex_);
    auto transportIt = transports_.find(device);
    if (transportIt == transports_.end()) {
        return false;
    }
    auto transport = transportIt->second;

    if (type == CanType::MT) {
        if (mtProtocols_.find(device) == mtProtocols_.end()) {
            mtProtocols_[device] = std::make_shared<MtCan>(transport);
        }
    } else {
        if (eyouProtocols_.find(device) == eyouProtocols_.end()) {
            eyouProtocols_[device] = std::make_shared<EyouCan>(transport);
        }
    }
    return true;
}

bool DeviceManager::initDevice(const std::string &device,
                               const std::vector<std::pair<CanType, MotorID>> &motors,
                               bool loopback)
{
    std::unique_lock<std::shared_mutex> lock(mutex_);
    auto transportIt = transports_.find(device);
    if (transportIt == transports_.end()) {
        auto transport = std::make_shared<SocketCanController>();
        if (!transport->initialize(device, loopback)) {
            ROS_ERROR("[CanDriverHW] Failed to init '%s'.", device.c_str());
            return false;
        }
        transports_[device] = transport;
        deviceCmdMutexes_[device] = std::make_shared<std::mutex>();
        transportIt = transports_.find(device);
    } else {
        const auto &transport = transportIt->second;
        transport->shutdown();
        if (!transport->initialize(device, loopback)) {
            ROS_ERROR("[CanDriverHW] Re-init of '%s' failed.", device.c_str());
            return false;
        }
    }

    if (deviceCmdMutexes_.find(device) == deviceCmdMutexes_.end()) {
        deviceCmdMutexes_[device] = std::make_shared<std::mutex>();
    }

    std::vector<MotorID> mtIds;
    std::vector<MotorID> ppIds;
    for (const auto &entry : motors) {
        if (entry.first == CanType::MT) {
            mtIds.push_back(entry.second);
        } else {
            ppIds.push_back(entry.second);
        }
    }
    if (!mtIds.empty() && mtProtocols_.find(device) == mtProtocols_.end()) {
        mtProtocols_[device] = std::make_shared<MtCan>(transportIt->second);
    }
    if (!ppIds.empty() && eyouProtocols_.find(device) == eyouProtocols_.end()) {
        eyouProtocols_[device] = std::make_shared<EyouCan>(transportIt->second);
    }
    if (!mtIds.empty()) {
        mtProtocols_[device]->initializeMotorRefresh(mtIds);
    }
    if (!ppIds.empty()) {
        eyouProtocols_[device]->initializeMotorRefresh(ppIds);
    }

    ROS_INFO("[CanDriverHW] Initialized '%s'.", device.c_str());
    return true;
}

void DeviceManager::startRefresh(const std::string &device,
                                 CanType type,
                                 const std::vector<MotorID> &ids)
{
    std::shared_lock<std::shared_mutex> lock(mutex_);
    if (ids.empty()) {
        return;
    }
    if (type == CanType::MT) {
        auto it = mtProtocols_.find(device);
        if (it != mtProtocols_.end()) {
            it->second->initializeMotorRefresh(ids);
        }
    } else {
        auto it = eyouProtocols_.find(device);
        if (it != eyouProtocols_.end()) {
            it->second->initializeMotorRefresh(ids);
        }
    }
}

void DeviceManager::shutdownAll()
{
    std::unique_lock<std::shared_mutex> lock(mutex_);
    mtProtocols_.clear();
    eyouProtocols_.clear();
    for (auto &kv : transports_) {
        kv.second->shutdown();
    }
    transports_.clear();
    deviceCmdMutexes_.clear();
}

std::shared_ptr<CanProtocol> DeviceManager::getProtocol(const std::string &device, CanType type) const
{
    std::shared_lock<std::shared_mutex> lock(mutex_);
    if (type == CanType::MT) {
        auto it = mtProtocols_.find(device);
        if (it != mtProtocols_.end()) {
            return std::static_pointer_cast<CanProtocol>(it->second);
        }
    } else {
        auto it = eyouProtocols_.find(device);
        if (it != eyouProtocols_.end()) {
            return std::static_pointer_cast<CanProtocol>(it->second);
        }
    }
    return nullptr;
}

std::shared_ptr<std::mutex> DeviceManager::getDeviceMutex(const std::string &device) const
{
    std::shared_lock<std::shared_mutex> lock(mutex_);
    auto it = deviceCmdMutexes_.find(device);
    return (it != deviceCmdMutexes_.end()) ? it->second : nullptr;
}

std::shared_ptr<SocketCanController> DeviceManager::getTransport(const std::string &device) const
{
    std::shared_lock<std::shared_mutex> lock(mutex_);
    auto it = transports_.find(device);
    return (it != transports_.end()) ? it->second : nullptr;
}

std::size_t DeviceManager::deviceCount() const
{
    std::shared_lock<std::shared_mutex> lock(mutex_);
    return transports_.size();
}
