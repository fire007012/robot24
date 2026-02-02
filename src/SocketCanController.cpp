#include "SocketCanController.h"

#include <ros/console.h>
#include <socketcan_interface/threading.h>
#include <socketcan_interface/settings.h>

#include <algorithm>
#include <cstring>
#include <functional>

namespace {
constexpr std::size_t kFrameBytes = 13;
}

SocketCanController::SocketCanController()
    : interface_(std::make_shared<can::ThreadedSocketCANInterface>())
{
}

SocketCanController::~SocketCanController()
{
    shutdown();
}

bool SocketCanController::initialize(const std::string &device, bool loopback)
{
    shutdown();

    if (!interface_) {
        interface_ = std::make_shared<can::ThreadedSocketCANInterface>();
    }

    if (!interface_->init(device, loopback, can::NoSettings::create())) {
        ROS_ERROR_STREAM("[SocketCanController] Failed to init device " << device);
        return false;
    }

    frameListener_ = interface_->createMsgListener(
        std::bind(&SocketCanController::handleFrame, this, std::placeholders::_1));

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
    initialized_.store(false);
    frameListener_.reset();
    stateListener_.reset();

    if (interface_) {
        interface_->shutdown();
    }

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

void SocketCanController::send(const std::uint8_t *data, std::size_t size)
{
    if (!initialized_.load() || !interface_ || !data || size == 0) {
        return;
    }

    if (size % kFrameBytes != 0) {
        ROS_ERROR_STREAM("[SocketCanController] Payload size " << size
                         << " is not a multiple of " << kFrameBytes);
        return;
    }

    for (std::size_t offset = 0; offset < size; offset += kFrameBytes) {
        can::Frame frame;
        if (!parseBufferToFrame(data + offset, frame)) {
            ROS_WARN("[SocketCanController] Skip malformed frame");
            continue;
        }
        if (!interface_->send(frame)) {
            ROS_WARN_STREAM("[SocketCanController] Failed to enqueue frame on " << deviceName_);
        }
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
    dispatchReceive(encodeFrame(frame));
}

void SocketCanController::dispatchReceive(const Buffer &buffer)
{
    std::unordered_map<std::size_t, ReceiveHandler> handlersCopy;
    {
        std::lock_guard<std::mutex> lock(handlerMutex_);
        handlersCopy = handlers_;
    }

    for (auto &entry : handlersCopy) {
        if (entry.second) {
            entry.second(buffer);
        }
    }
}

bool SocketCanController::parseBufferToFrame(const std::uint8_t *data, can::Frame &frame) const
{
    if (!data) {
        return false;
    }

    const std::uint8_t dlc = data[0];
    if (dlc > 8) {
        return false;
    }

    const std::uint16_t canId =
        static_cast<std::uint16_t>(static_cast<std::uint16_t>(data[3]) << 8) |
        static_cast<std::uint16_t>(data[4]);

    frame = can::Frame();
    frame.id = canId;
    frame.is_extended = 0;
    frame.is_rtr = 0;
    frame.is_error = 0;
    frame.dlc = dlc;

    for (std::size_t i = 0; i < dlc && i < frame.data.size(); ++i) {
        frame.data[i] = data[5 + i];
    }
    return true;
}

SocketCanController::Buffer SocketCanController::encodeFrame(const can::Frame &frame) const
{
    Buffer buffer(kFrameBytes, 0);
    buffer[0] = frame.dlc;
    buffer[3] = static_cast<std::uint8_t>((frame.id >> 8) & 0xFF);
    buffer[4] = static_cast<std::uint8_t>(frame.id & 0xFF);

    const std::size_t bytes = std::min<std::size_t>(frame.dlc, frame.data.size());
    for (std::size_t i = 0; i < bytes; ++i) {
        buffer[5 + i] = frame.data[i];
    }
    return buffer;
}
