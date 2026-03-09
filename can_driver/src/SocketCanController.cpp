#include "can_driver/SocketCanController.h"

#include <ros/console.h>
#include <socketcan_interface/threading.h>
#include <socketcan_interface/settings.h>

#include <algorithm>
#include <cstring>
#include <functional>
#include <vector>

SocketCanController::SocketCanController()
    : interface_(std::make_shared<can::ThreadedSocketCANInterface>())
{
}

SocketCanController::SocketCanController(can::ThreadedSocketCANInterfaceSharedPtr injectedInterface)
    : interface_(std::move(injectedInterface))
{
}

SocketCanController::~SocketCanController()
{
    shutdown();
}

bool SocketCanController::initialize(const std::string &device, bool loopback)
{
    // 允许重复初始化：先清理旧监听器和旧接口状态。
    shutdown();

    if (!interface_) {
        interface_ = std::make_shared<can::ThreadedSocketCANInterface>();
    }

    if (!interface_->init(device, loopback, can::NoSettings::create())) {
        ROS_ERROR_STREAM("[SocketCanController] Failed to init device " << device);
        return false;
    }

    // 注册帧回调，将 socketcan 帧统一转换后分发给上层协议。
    frameListener_ = interface_->createMsgListener(
        std::bind(&SocketCanController::handleFrame, this, std::placeholders::_1));

    // 仅记录非 ready 状态，便于现场排障。
    stateListener_ = interface_->createStateListener(
        [device](const can::State &state) {
            if (!state.isReady()) {
                ROS_WARN_STREAM("[SocketCanController] Device " << device
                                << " entered state " << state.driver_state
                                << " error=" << state.error_code.message()
                                << " internal=" << state.internal_error);
            }
        });

    deviceName_ = device;
    initialized_.store(true);
    return true;
}

void SocketCanController::shutdown()
{
    // 先关闭 initialized 标志，阻止并发 send 继续写入。
    initialized_.store(false);
    frameListener_.reset();
    stateListener_.reset();

    if (interface_) {
        interface_->shutdown();
    }

    // handler ID 从 1 重新开始，便于测试中验证生命周期。
    {
        std::lock_guard<std::mutex> lock(handlerMutex_);
        handlers_.clear();
        nextHandlerId_.store(1);
    }
    deviceName_.clear();
}

bool SocketCanController::isReady() const
{
    return initialized_.load();
}

std::string SocketCanController::device() const
{
    return deviceName_;
}

void SocketCanController::send(const CanTransport::Frame &frame)
{
    // 未初始化时发送为 no-op，保持调用侧简单。
    if (!initialized_.load() || !interface_) {
        return;
    }

    const can::Frame socketFrame = toSocketCanFrame(frame);
    // frame 非法（如 DLC 超范围）时不上总线。
    if (!socketFrame.isValid()) {
        ROS_WARN_STREAM("[SocketCanController] Skip invalid frame for device " << deviceName_
                        << " id=0x" << std::hex << socketFrame.id << std::dec
                        << " dlc=" << static_cast<unsigned>(socketFrame.dlc)
                        << " ext=" << static_cast<unsigned>(socketFrame.is_extended)
                        << " rtr=" << static_cast<unsigned>(socketFrame.is_rtr)
                        << " err=" << static_cast<unsigned>(socketFrame.is_error));
        return;
    }

    if (!interface_->send(socketFrame)) {
        ROS_WARN_STREAM("[SocketCanController] Failed to enqueue frame on " << deviceName_);
    }
}

std::size_t SocketCanController::addReceiveHandler(ReceiveHandler handler)
{
    if (!handler) {
        return 0;
    }

    std::lock_guard<std::mutex> lock(handlerMutex_);
    const std::size_t id = nextHandlerId_++;
    handlers_.emplace(id, std::move(handler));
    return id;
}

void SocketCanController::removeReceiveHandler(std::size_t handlerId)
{
    if (handlerId == 0) {
        return;
    }

    std::lock_guard<std::mutex> lock(handlerMutex_);
    handlers_.erase(handlerId);
}

void SocketCanController::handleFrame(const can::Frame &frame)
{
    dispatchReceive(fromSocketCanFrame(frame));
}

void SocketCanController::dispatchReceive(const CanTransport::Frame &frame)
{
    std::vector<ReceiveHandler> handlersCopy;
    {
        std::lock_guard<std::mutex> lock(handlerMutex_);
        handlersCopy.reserve(handlers_.size());
        for (const auto &entry : handlers_) {
            handlersCopy.push_back(entry.second);
        }
    }

    // 在锁外回调，避免 handler 内部再次注册/注销造成死锁。
    for (auto &handler : handlersCopy) {
        if (handler) {
            handler(frame);
        }
    }
}

can::Frame SocketCanController::toSocketCanFrame(const CanTransport::Frame &frame) const
{
    can::Frame socketFrame;
    socketFrame.id = frame.id;
    socketFrame.is_extended = frame.isExtended ? 1 : 0;
    socketFrame.is_rtr = frame.isRemoteRequest ? 1 : 0;
    socketFrame.is_error = 0;
    socketFrame.data.fill(0);
    // DLC 始终截断到 8 字节，避免越界复制。
    socketFrame.dlc = std::min<std::uint8_t>(frame.dlc, static_cast<std::uint8_t>(socketFrame.data.size()));
    for (std::size_t i = 0; i < socketFrame.dlc; ++i) {
        socketFrame.data[i] = frame.data[i];
    }
    return socketFrame;
}

CanTransport::Frame SocketCanController::fromSocketCanFrame(const can::Frame &frame) const
{
    CanTransport::Frame userFrame;
    userFrame.id = frame.id;
    userFrame.isExtended = frame.is_extended != 0;
    userFrame.isRemoteRequest = frame.is_rtr != 0;
    userFrame.data.fill(0);
    // 同样按 8 字节上限截断，保持对称转换。
    userFrame.dlc = std::min<std::uint8_t>(frame.dlc, static_cast<std::uint8_t>(userFrame.data.size()));
    for (std::size_t i = 0; i < userFrame.dlc; ++i) {
        userFrame.data[i] = frame.data[i];
    }
    return userFrame;
}
