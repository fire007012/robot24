#ifndef CAN_DRIVER_DEVICE_MANAGER_H
#define CAN_DRIVER_DEVICE_MANAGER_H

#include "can_driver/CanProtocol.h"
#include "can_driver/CanType.h"
#include "can_driver/EyouCan.h"
#include "can_driver/MotorID.h"
#include "can_driver/MtCan.h"
#include "can_driver/SocketCanController.h"

#include <map>
#include <memory>
#include <mutex>
#include <shared_mutex>
#include <string>
#include <vector>

class DeviceManager {
public:
    bool ensureTransport(const std::string &device, bool loopback = false);
    bool ensureProtocol(const std::string &device, CanType type);
    bool initDevice(const std::string &device,
                    const std::vector<std::pair<CanType, MotorID>> &motors,
                    bool loopback = false);
    void startRefresh(const std::string &device,
                      CanType type,
                      const std::vector<MotorID> &ids);
    void shutdownAll();

    std::shared_ptr<CanProtocol> getProtocol(const std::string &device, CanType type) const;
    std::shared_ptr<std::mutex> getDeviceMutex(const std::string &device) const;
    std::shared_ptr<SocketCanController> getTransport(const std::string &device) const;
    std::size_t deviceCount() const;

private:
    mutable std::shared_mutex mutex_;
    std::map<std::string, std::shared_ptr<SocketCanController>> transports_;
    std::map<std::string, std::shared_ptr<MtCan>> mtProtocols_;
    std::map<std::string, std::shared_ptr<EyouCan>> eyouProtocols_;
    std::map<std::string, std::shared_ptr<std::mutex>> deviceCmdMutexes_;
};

#endif // CAN_DRIVER_DEVICE_MANAGER_H
