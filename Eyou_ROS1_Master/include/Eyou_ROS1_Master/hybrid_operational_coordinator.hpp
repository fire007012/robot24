#pragma once

#include <string>

#include "can_driver/operational_coordinator.hpp"
#include "canopen_hw/operational_coordinator.hpp"

namespace eyou_ros1_master {

// 串行协调 can_driver 和 canopen_hw 两侧状态机。
// 每个请求先调 can 侧，再调 canopen 侧；任一失败则整体失败。
// mode() 取两侧悲观值（较低的那个）。
//
// 两个子协调器的生命周期由调用方管理，本类只持有非拥有指针。
class HybridOperationalCoordinator {
public:
    struct Result {
        bool ok{false};
        std::string message;
        bool already{false};
    };

    HybridOperationalCoordinator(can_driver::OperationalCoordinator* can_coord,
                                  canopen_hw::OperationalCoordinator* canopen_coord);

    // 取两侧悲观值（数值较大 == 状态更差）。
    can_driver::SystemOpMode mode() const;

    // 生命周期请求。
    // can_driver::Init 需要 device + loopback 参数，通过 rosparam 传入，
    // 此处封装为无参版本（内部读取 rosparam）。
    Result RequestInit(const std::string& device, bool loopback);
    Result RequestEnable();
    Result RequestDisable();
    Result RequestRelease();
    Result RequestHalt();
    Result RequestRecover();
    Result RequestShutdown(bool force);

private:
    can_driver::OperationalCoordinator* can_coord_;
    canopen_hw::OperationalCoordinator* canopen_coord_;
};

}  // namespace eyou_ros1_master
