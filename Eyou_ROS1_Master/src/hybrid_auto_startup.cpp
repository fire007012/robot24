#include "Eyou_ROS1_Master/hybrid_auto_startup.hpp"

#include <utility>

#include "can_driver/operational_coordinator.hpp"

namespace eyou_ros1_master {

namespace {

bool SetError(const std::string& message, std::string* error) {
    if (error != nullptr) {
        *error = message;
    }
    return false;
}

bool EnsureHybridArmed(HybridOperationalCoordinator& hybrid_coord,
                       std::string* error) {
    switch (hybrid_coord.mode()) {
    case can_driver::SystemOpMode::Standby: {
        const auto enable_result = hybrid_coord.RequestEnable();
        if (!enable_result.ok) {
            return SetError(enable_result.message, error);
        }
        return true;
    }
    case can_driver::SystemOpMode::Armed:
    case can_driver::SystemOpMode::Running:
        return true;
    default:
        return SetError("auto startup expected hybrid lifecycle in Standby/Armed/Running after init",
                        error);
    }
}

bool EnsureHybridRunning(HybridOperationalCoordinator& hybrid_coord,
                         const HybridAutoStartupOptions& options,
                         std::string* error) {
    if (!EnsureHybridArmed(hybrid_coord, error)) {
        return false;
    }

    switch (hybrid_coord.mode()) {
    case can_driver::SystemOpMode::Running:
        return true;
    case can_driver::SystemOpMode::Armed: {
        const ros::WallDuration retry_interval(
            options.auto_release_retry_interval_sec > 0.0
                ? options.auto_release_retry_interval_sec
                : 0.1);
        const ros::WallTime deadline =
            ros::WallTime::now() + ros::WallDuration(std::max(0.0, options.auto_release_timeout_sec));

        std::string last_error;
        while (true) {
            const auto release_result = hybrid_coord.RequestRelease();
            if (release_result.ok) {
                return true;
            }
            last_error = release_result.message;
            if (hybrid_coord.mode() == can_driver::SystemOpMode::Running) {
                return true;
            }
            if (ros::WallTime::now() >= deadline) {
                break;
            }
            retry_interval.sleep();
        }
        return SetError(last_error.empty() ? "auto startup timed out before hybrid lifecycle reached running"
                                           : last_error,
                        error);
    }
    default:
        return SetError("auto startup could not drive hybrid lifecycle to running",
                        error);
    }
}

}  // namespace

HybridAutoStartupOptions LoadHybridAutoStartupOptions(const ros::NodeHandle& pnh) {
    HybridAutoStartupOptions options;
    pnh.param("auto_init", options.auto_init, false);
    pnh.param("auto_enable", options.auto_enable, false);
    pnh.param("auto_release", options.auto_release, false);
    pnh.param("auto_release_timeout_sec", options.auto_release_timeout_sec, 3.0);
    pnh.param("auto_release_retry_interval_sec",
              options.auto_release_retry_interval_sec,
              0.1);
    return options;
}

bool RunHybridAutoStartup(const HybridAutoStartupOptions& options,
                          HybridServiceGateway& service_gateway,
                          HybridOperationalCoordinator& hybrid_coord,
                          std::string* error) {
    if (error != nullptr) {
        error->clear();
    }

    if ((options.auto_enable || options.auto_release) && !options.auto_init) {
        return SetError("auto_enable/auto_release require auto_init=true", error);
    }
    if (options.auto_release && !options.auto_enable) {
        return SetError("auto_release requires auto_enable=true", error);
    }
    if (!options.auto_init) {
        return true;
    }

    bool already_initialized = false;
    std::string detail;
    if (!service_gateway.RunConfiguredInitSequence(&detail, &already_initialized)) {
        return SetError(detail, error);
    }

    if (options.auto_enable && !EnsureHybridArmed(hybrid_coord, error)) {
        return false;
    }
    if (options.auto_release && !EnsureHybridRunning(hybrid_coord, options, error)) {
        return false;
    }

    return true;
}

bool RunHybridAutoStartupFromParams(HybridServiceGateway& service_gateway,
                                    HybridOperationalCoordinator& hybrid_coord,
                                    const ros::NodeHandle& pnh,
                                    std::string* error) {
    return RunHybridAutoStartup(LoadHybridAutoStartupOptions(pnh),
                                service_gateway,
                                hybrid_coord,
                                error);
}

}  // namespace eyou_ros1_master
