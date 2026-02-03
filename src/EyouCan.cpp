#include "can_driver/EyouCan.h"

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <iomanip>
#include <iostream>

// EyouCan.cpp

namespace {
constexpr uint8_t kWriteCommand = 0x01;
constexpr uint8_t kReadCommand = 0x03;
constexpr uint8_t kWriteAck = 0x02;
constexpr uint8_t kReadResponse = 0x04;
constexpr uint16_t kEyouIdFrameBase = 0x0000;

uint32_t readUInt32BE(const CanTransport::Frame &frame, std::size_t index)
{
    if (index + 3 >= frame.dlc) {
        return 0;
    }
    return (static_cast<uint32_t>(frame.data[index]) << 24) |
           (static_cast<uint32_t>(frame.data[index + 1]) << 16) |
           (static_cast<uint32_t>(frame.data[index + 2]) << 8) |
           static_cast<uint32_t>(frame.data[index + 3]);
}

int32_t readInt32BE(const CanTransport::Frame &frame, std::size_t index)
{
    return static_cast<int32_t>(readUInt32BE(frame, index));
}

uint8_t dataByteOrZero(const CanTransport::Frame &frame, std::size_t index)
{
    if (index >= frame.dlc) {
        return 0;
    }
    return frame.data[index];
}
} // namespace

EyouCan::EyouCan(std::shared_ptr<CanTransport> controller)
    : canController(std::move(controller))
{
    if (canController) {
        receiveHandlerId = canController->addReceiveHandler(
            [this](const CanTransport::Frame &frame) { handleResponse(frame); });
    }
}

EyouCan::~EyouCan()
{
    stopRefreshLoop();
    if (canController && receiveHandlerId != 0) {
        canController->removeReceiveHandler(receiveHandlerId);
    }
}

void EyouCan::initializeMotorRefresh(const std::vector<MotorID> &motorIds)
{
    {
        std::lock_guard<std::mutex> lock(refreshMutex);
        refreshMotorIds.clear();
        refreshMotorIds.reserve(motorIds.size());
        for (MotorID id : motorIds) {
            refreshMotorIds.push_back(static_cast<uint8_t>(id));
        }
    }

    if (motorIds.empty()) {
        stopRefreshLoop();
        return;
    }

    if (refreshLoopActive.load()) {
        return;
    }

    bool expected = false;
    if (!refreshLoopActive.compare_exchange_strong(expected, true)) {
        return;
    }

    // 轮询线程专职触发 1ms 定时器，避免阻塞 ROS 回调或其他实时线程。
    refreshThread = std::thread([this]() {
        while (refreshLoopActive.load()) {
            refreshMotorStates();
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    });
}

void EyouCan::setMode(MotorID Id, MotorMode mode)
{
    uint8_t motorId = static_cast<uint8_t>(Id);
    {
        std::lock_guard<std::mutex> stateLock(stateMutex);
        motorStates[motorId].mode = mode;
    }
    const uint32_t modeValue = mode == MotorMode::Velocity ? 0x00000003 : 0x00000001;
    sendWriteCommand(motorId, 0x0F, modeValue, 4);
}

void EyouCan::setVelocity(MotorID Id, int32_t velocity)
{
    uint8_t motorId = static_cast<uint8_t>(Id);
    {
        std::lock_guard<std::mutex> stateLock(stateMutex);
        motorStates[motorId].commandedVelocity = velocity;
    }
    sendWriteCommand(motorId, 0x09, static_cast<uint32_t>(velocity), 4);
}

void EyouCan::setAcceleration(MotorID Id, int32_t acceleration)
{
    uint8_t motorId = static_cast<uint8_t>(Id);
    {
        std::lock_guard<std::mutex> stateLock(stateMutex);
        motorStates[motorId].acceleration = acceleration;
    }
    // 原协议未定义加速度写入，保留数据以备后用
}

void EyouCan::setDeceleration(MotorID Id, int32_t deceleration)
{
    uint8_t motorId = static_cast<uint8_t>(Id);
    {
        std::lock_guard<std::mutex> stateLock(stateMutex);
        motorStates[motorId].deceleration = deceleration;
    }
}

void EyouCan::setPosition(MotorID Id, int32_t position)
{
    uint8_t motorId = static_cast<uint8_t>(Id);
    {
        std::lock_guard<std::mutex> stateLock(stateMutex);
        motorStates[motorId].commandedPosition = position;
    }
    sendWriteCommand(motorId, 0x0A, static_cast<uint32_t>(position), 4);
}

void EyouCan::Enable(MotorID Id)
{
    uint8_t motorId = static_cast<uint8_t>(Id);
    {
        std::lock_guard<std::mutex> stateLock(stateMutex);
        motorStates[motorId].enabled = true;
    }
    sendWriteCommand(motorId, 0x10, 0x00000001, 4);
}

void EyouCan::Disable(MotorID Id)
{
    uint8_t motorId = static_cast<uint8_t>(Id);
    {
        std::lock_guard<std::mutex> stateLock(stateMutex);
        motorStates[motorId].enabled = false;
    }
    sendWriteCommand(motorId, 0x10, 0x00000000, 4);
}

void EyouCan::Stop(MotorID Id)
{
    uint8_t motorId = static_cast<uint8_t>(Id);
    sendWriteCommand(motorId, 0x11, 0x00000000, 4);
}

int32_t EyouCan::getPosition(MotorID Id) const
{
    uint8_t motorId = static_cast<uint8_t>(Id);
    bool hasState = false;
    int32_t position = 0;
    bool needsRefresh = false;
    {
        std::lock_guard<std::mutex> stateLock(stateMutex);
        auto it = motorStates.find(motorId);
        if (it != motorStates.end()) {
            hasState = true;
            position = it->second.position;
            needsRefresh = (position == 0);
        }
    }
    if (!hasState) {
        requestPosition(motorId);
        return 0;
    }
    if (needsRefresh) {
        requestPosition(motorId);
    }
    return position;
}

int16_t EyouCan::getCurrent(MotorID Id) const
{
    uint8_t motorId = static_cast<uint8_t>(Id);
    bool hasState = false;
    {
        std::lock_guard<std::mutex> stateLock(stateMutex);
        auto it = motorStates.find(motorId);
        if (it != motorStates.end()) {
            hasState = true;
        }
    }
    if (!hasState) {
        requestEnable(motorId);
    }
    return 0;
}

int16_t EyouCan::getVelocity(MotorID Id) const
{
    uint8_t motorId = static_cast<uint8_t>(Id);
    bool hasState = false;
    int32_t commandedVelocity = 0;
    {
        std::lock_guard<std::mutex> stateLock(stateMutex);
        auto it = motorStates.find(motorId);
        if (it != motorStates.end()) {
            hasState = true;
            commandedVelocity = it->second.commandedVelocity;
        }
    }
    if (!hasState) {
        return 0;
    }
    return static_cast<int16_t>(commandedVelocity);
}

void EyouCan::sendWriteCommand(uint8_t motorId, uint8_t subCommand, uint32_t value, std::size_t payloadBytes)
{
    if (!canController) {
        std::cerr << "[EyouCan] CAN controller not initialized\n";
        return;
    }

    if (payloadBytes < 1) {
        payloadBytes = 1;
    } else if (payloadBytes > 4) {
        payloadBytes = 4;
    }

    CanTransport::Frame frame;
    frame.id = kEyouIdFrameBase + motorId;
    frame.isExtended = false;
    frame.isRemoteRequest = false;
    const std::size_t maxPayload = std::min<std::size_t>(payloadBytes, frame.data.size() - 2);
    frame.dlc = static_cast<std::uint8_t>(2 + maxPayload);
    frame.data[0] = kWriteCommand;
    frame.data[1] = subCommand;

    for (std::size_t i = 0; i < maxPayload; ++i) {
        const int shift = static_cast<int>((payloadBytes - 1 - i) * 8);
        frame.data[2 + i] = static_cast<uint8_t>((value >> shift) & 0xFF);
    }

    canController->send(frame);
}

void EyouCan::sendReadCommand(uint8_t motorId, uint8_t subCommand) const
{
    if (!canController) {
        return;
    }

    CanTransport::Frame frame;
    frame.id = kEyouIdFrameBase + motorId;
    frame.isExtended = false;
    frame.isRemoteRequest = false;
    frame.dlc = 2;
    frame.data[0] = kReadCommand;
    frame.data[1] = subCommand;
    canController->send(frame);
}

void EyouCan::handleResponse(const CanTransport::Frame &frame)
{
    if (frame.isExtended || frame.dlc < 2) {
        return;
    }

    const uint16_t canId = static_cast<uint16_t>(frame.id & 0x7FF);
    const uint8_t motorId = static_cast<uint8_t>(canId & 0xFF);
    if (canId != kEyouIdFrameBase + motorId) {
        std::cerr << "[EyouCan] Ignore frame with CAN ID 0x"
                  << std::hex << canId << " (expected 0x"
                  << static_cast<int>(motorId) << ')' << std::dec << '\n';
        return;
    }

    const uint8_t responseType = frame.data[0];
    const uint8_t subCommand = frame.data[1];

    {
        std::lock_guard<std::mutex> stateLock(stateMutex);
        MotorState &state = motorStates[motorId];

        if (responseType == kWriteAck) {
            switch (subCommand) {
            case 0x10:
                state.enabled = dataByteOrZero(frame, 2) != 0;
                break;
            case 0x0F:
                state.mode = dataByteOrZero(frame, 5) == 0x03 ? MotorMode::Velocity : MotorMode::Position;
                break;
            default:
                break;
            }
        } else if (responseType == kReadResponse) {
            switch (subCommand) {
            case 0x07:
                state.position = readInt32BE(frame, 2);
                break;
            case 0x0F:
                state.mode = dataByteOrZero(frame, 5) == 0x03 ? MotorMode::Velocity : MotorMode::Position;
                break;
            case 0x10:
                state.enabled = dataByteOrZero(frame, 5) != 0;
                break;
            case 0x15:
                std::cerr << "[EyouCan] Motor " << static_cast<int>(motorId)
                          << " reported error code " << readUInt32BE(frame, 4) << '\n';
                break;
            default:
                break;
            }
        }
    }
}

void EyouCan::requestPosition(uint8_t motorId) const
{
    sendReadCommand(motorId, 0x07);
}

void EyouCan::requestMode(uint8_t motorId) const
{
    sendReadCommand(motorId, 0x0F);
}

void EyouCan::requestEnable(uint8_t motorId) const
{
    sendReadCommand(motorId, 0x10);
}

void EyouCan::refreshMotorStates()
{
    std::vector<uint8_t> motorIds;
    {
        std::lock_guard<std::mutex> lock(refreshMutex);
        motorIds = refreshMotorIds;
    }

    if (!canController) {
        return;
    }

    // 对每个节点轮询三个关键信息：位置、模式、使能状态。
    for (uint8_t motorId : motorIds) {
        if (!refreshLoopActive.load()) {
            break;
        }
        requestPosition(motorId);
        requestMode(motorId);
        requestEnable(motorId);
    }
}

void EyouCan::stopRefreshLoop()
{
    refreshLoopActive.store(false);

    if (refreshThread.joinable()) {
        refreshThread.join();
    }
}
