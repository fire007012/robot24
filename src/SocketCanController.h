#ifndef CAN_DRIVER_SOCKETCAN_CONTROLLER_H
#define CAN_DRIVER_SOCKETCAN_CONTROLLER_H

#include "can_driver/CanTransport.h"

#include <socketcan_interface/socketcan.h>

#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>

/**
 * @brief Transport implementation backed by ros_canopen's socketcan_interface.
 */
class SocketCanController : public CanTransport {
public:
    SocketCanController();
    ~SocketCanController() override;

    /**
     * @brief Initialize the CAN interface.
     * @param device SocketCAN device name (e.g. "can0").
     * @param loopback Enable CAN loopback frames.
     */
    bool initialize(const std::string &device, bool loopback = false);

    void shutdown();

    void send(const std::uint8_t *data, std::size_t size) override;

    std::size_t addReceiveHandler(ReceiveHandler handler) override;
    void removeReceiveHandler(std::size_t handlerId) override;

    bool isReady() const;
    std::string device() const;

private:
    static constexpr std::size_t kFrameBytes = 13;

    void handleFrame(const can::Frame &frame);
    void dispatchReceive(const Buffer &buffer);
    bool parseBufferToFrame(const std::uint8_t *data, can::Frame &frame) const;
    Buffer encodeFrame(const can::Frame &frame) const;

    can::ThreadedSocketCANInterfaceSharedPtr interface_;
    can::CommInterface::FrameListenerConstSharedPtr frameListener_;
    can::StateInterface::StateListenerConstSharedPtr stateListener_;

    std::atomic<bool> initialized_{false};
    std::atomic<std::size_t> nextHandlerId_{1};
    std::unordered_map<std::size_t, ReceiveHandler> handlers_;
    mutable std::mutex handlerMutex_;
    std::string deviceName_;
};

#endif // CAN_DRIVER_SOCKETCAN_CONTROLLER_H
