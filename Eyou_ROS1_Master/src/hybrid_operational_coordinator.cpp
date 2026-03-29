#include "Eyou_ROS1_Master/hybrid_operational_coordinator.hpp"

#include <algorithm>
#include <ros/ros.h>

namespace eyou_ros1_master {

namespace {

// 两个命名空间的 SystemOpMode 枚举值完全对齐（Inactive=0 … ShuttingDown=7），
// 可安全互转。
can_driver::SystemOpMode FromCanopen(canopen_hw::SystemOpMode m) {
    return static_cast<can_driver::SystemOpMode>(static_cast<unsigned char>(m));
}

// 悲观合并：fault / recovering / shuttingDown 优先，
// 否则取正常序中较低的那个。
can_driver::SystemOpMode PessimisticMerge(can_driver::SystemOpMode a,
                                           can_driver::SystemOpMode b) {
    using M = can_driver::SystemOpMode;
    // fault 状态优先级：Faulted > Recovering > ShuttingDown
    for (auto s : {M::Faulted, M::Recovering, M::ShuttingDown}) {
        if (a == s || b == s) return s;
    }
    // 正常序 Inactive < Configured < Standby < Armed < Running → 取较小
    return static_cast<M>(std::min(static_cast<unsigned char>(a),
                                   static_cast<unsigned char>(b)));
}

}  // namespace

HybridOperationalCoordinator::HybridOperationalCoordinator(
    can_driver::OperationalCoordinator* can_coord,
    canopen_hw::OperationalCoordinator* canopen_coord)
    : can_coord_(can_coord), canopen_coord_(canopen_coord) {}

can_driver::SystemOpMode HybridOperationalCoordinator::mode() const {
    return PessimisticMerge(can_coord_->mode(),
                            FromCanopen(canopen_coord_->mode()));
}

// ----------- 生命周期请求 -----------

HybridOperationalCoordinator::Result
HybridOperationalCoordinator::RequestInit(const std::string& device,
                                           bool loopback) {
    auto r1 = can_coord_->RequestInit(device, loopback);
    if (!r1.ok) return {false, "[can_driver] " + r1.message};

    auto r2 = canopen_coord_->RequestInit();
    if (!r2.ok) return {false, "[canopen] " + r2.message};

    return {true, "both backends initialized"};
}

HybridOperationalCoordinator::Result
HybridOperationalCoordinator::RequestEnable() {
    auto r1 = can_coord_->RequestEnable();
    if (!r1.ok) return {false, "[can_driver] " + r1.message};

    auto r2 = canopen_coord_->RequestEnable();
    if (!r2.ok) return {false, "[canopen] " + r2.message};

    return {true, "both backends enabled"};
}

HybridOperationalCoordinator::Result
HybridOperationalCoordinator::RequestDisable() {
    auto r1 = can_coord_->RequestDisable();
    if (!r1.ok) return {false, "[can_driver] " + r1.message};

    auto r2 = canopen_coord_->RequestDisable();
    if (!r2.ok) return {false, "[canopen] " + r2.message};

    return {true, "both backends disabled"};
}

HybridOperationalCoordinator::Result
HybridOperationalCoordinator::RequestRelease() {
    auto r1 = can_coord_->RequestRelease();
    if (!r1.ok) return {false, "[can_driver] " + r1.message};

    auto r2 = canopen_coord_->RequestRelease();
    if (!r2.ok) return {false, "[canopen] " + r2.message};

    return {true, "both backends released"};
}

HybridOperationalCoordinator::Result
HybridOperationalCoordinator::RequestHalt() {
    auto r1 = can_coord_->RequestHalt();
    if (!r1.ok) return {false, "[can_driver] " + r1.message};

    auto r2 = canopen_coord_->RequestHalt();
    if (!r2.ok) return {false, "[canopen] " + r2.message};

    return {true, "both backends halted"};
}

HybridOperationalCoordinator::Result
HybridOperationalCoordinator::RequestRecover() {
    auto r1 = can_coord_->RequestRecover();
    if (!r1.ok) return {false, "[can_driver] " + r1.message};

    auto r2 = canopen_coord_->RequestRecover();
    if (!r2.ok) return {false, "[canopen] " + r2.message};

    return {true, "both backends recovered"};
}

HybridOperationalCoordinator::Result
HybridOperationalCoordinator::RequestShutdown(bool force) {
    auto r1 = can_coord_->RequestShutdown(force);
    if (!r1.ok) return {false, "[can_driver] " + r1.message};

    auto r2 = canopen_coord_->RequestShutdown();
    if (!r2.ok) return {false, "[canopen] " + r2.message};

    return {true, "both backends shut down"};
}

void HybridOperationalCoordinator::UpdateFromFeedback(bool can_unhealthy) {
    can_coord_->UpdateFromFeedback(can_unhealthy);
    canopen_coord_->UpdateFromFeedback();
}

}  // namespace eyou_ros1_master
