#include "can_driver/MtCan.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>

namespace {
constexpr uint16_t kResponseBaseId = 0x240;

int16_t readInt16LE(const CanTransport::Frame &frame, std::size_t index)
{
    if (index + 1 >= frame.dlc) {
        return 0;
    }
    const int value = static_cast<int>(frame.data[index]) |
                      (static_cast<int>(frame.data[index + 1]) << 8);
    return static_cast<int16_t>(value);
}

uint16_t readUInt16LE(const CanTransport::Frame &frame, std::size_t index)
{
    if (index + 1 >= frame.dlc) {
        return 0;
    }
    return static_cast<uint16_t>(static_cast<uint16_t>(frame.data[index]) |
                                 (static_cast<uint16_t>(frame.data[index + 1]) << 8));
}
} // namespace

MtCan::MtCan(std::shared_ptr<CanTransport> controller)
    : canController(std::move(controller))
{
    if (canController) {
        receiveHandlerId = canController->addReceiveHandler(
            [this](const CanTransport::Frame &frame) { handleResponse(frame); });
    }
}

MtCan::~MtCan()
{
    stopRefreshLoop();
    if (canController && receiveHandlerId != 0) {
        canController->removeReceiveHandler(receiveHandlerId);
    }
}

void MtCan::initializeMotorRefresh(const std::vector<MotorID> &motorIds)
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

    // 独立线程专门跑 1ms 轮询，避免堵塞 ROS 回调或其他实时线程。
    refreshThread = std::thread([this]() {
        while (refreshLoopActive.load()) {
            refreshMotorStates();
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    });
}

void MtCan::setMode(MotorID Id, MotorMode mode)
{
    uint8_t motorId = static_cast<uint8_t>(Id);
    std::lock_guard<std::mutex> stateLock(stateMutex);
    motorStates[motorId].mode = mode;
}

void MtCan::setVelocity(MotorID Id, int32_t velocity)
{
    uint8_t motorId = static_cast<uint8_t>(Id);
    {
        std::lock_guard<std::mutex> stateLock(stateMutex);
        motorStates[motorId].commandedVelocity = velocity;
    }
    const uint16_t canId = encodeSendCanId(motorId);

    std::array<uint8_t, 4> payload {
        static_cast<uint8_t>(velocity & 0xFF),
        static_cast<uint8_t>((velocity >> 8) & 0xFF),
        static_cast<uint8_t>((velocity >> 16) & 0xFF),
        static_cast<uint8_t>((velocity >> 24) & 0xFF)
    };
    sendFrame(canId, 0xA2, payload);
}

void MtCan::setAcceleration(MotorID Id, int32_t acceleration)
{
    uint8_t motorId = static_cast<uint8_t>(Id);
    (void)motorId;
    (void)acceleration; // 协议当前未支持
}

void MtCan::setDeceleration(MotorID Id, int32_t deceleration)
{
    uint8_t motorId = static_cast<uint8_t>(Id);
    (void)motorId;
    (void)deceleration; // 协议当前未支持
}

void MtCan::setPosition(MotorID Id, int32_t position)
{
    uint8_t motorId = static_cast<uint8_t>(Id);
    int32_t commandedVelocity = 0;
    {
        std::lock_guard<std::mutex> stateLock(stateMutex);
        MotorState &state = motorStates[motorId];
        state.position = position;
        commandedVelocity = state.commandedVelocity;
    }

    const uint16_t canId = encodeSendCanId(motorId);
    const int32_t limitedVelocity = std::max<int32_t>(-32768, std::min<int32_t>(32767, commandedVelocity));
    const int16_t velocity = static_cast<int16_t>(limitedVelocity);

    CanTransport::Frame frame;
    frame.id = canId;
    frame.dlc = 8;
    frame.isExtended = false;
    frame.isRemoteRequest = false;
    frame.data[0] = 0xA4;
    frame.data[1] = 0x00;
    frame.data[2] = static_cast<uint8_t>(velocity & 0xFF);
    frame.data[3] = static_cast<uint8_t>((velocity >> 8) & 0xFF);
    frame.data[4] = static_cast<uint8_t>(position & 0xFF);
    frame.data[5] = static_cast<uint8_t>((position >> 8) & 0xFF);
    frame.data[6] = static_cast<uint8_t>((position >> 16) & 0xFF);
    frame.data[7] = static_cast<uint8_t>((position >> 24) & 0xFF);

    if (canController) {
        canController->send(frame);
    }
}

void MtCan::Enable(MotorID Id)
{
    uint8_t motorId = static_cast<uint8_t>(Id);
    {
        std::lock_guard<std::mutex> stateLock(stateMutex);
        motorStates[motorId].enabled = true;
    }
    setZeroPosition(motorId);
}

void MtCan::Disable(MotorID Id)
{
    uint8_t motorId = static_cast<uint8_t>(Id);
    {
        std::lock_guard<std::mutex> stateLock(stateMutex);
        motorStates[motorId].enabled = false;
    }
    Stop(Id);
}

void MtCan::Stop(MotorID Id)
{
    uint8_t motorId = static_cast<uint8_t>(Id);
    const uint16_t canId = encodeSendCanId(motorId);
    sendFrame(canId, 0x81, {0, 0, 0, 0});
}

int32_t MtCan::getPosition(MotorID Id) const
{
    uint8_t motorId = static_cast<uint8_t>(Id);
    int32_t position = 0;
    bool hasState = false;
    {
        std::lock_guard<std::mutex> stateLock(stateMutex);
        auto it = motorStates.find(motorId);
        if (it != motorStates.end()) {
            hasState = true;
            position = it->second.position;
        }
    }
    if (!hasState) {
        return 0;
    }
    return position;
}

int16_t MtCan::getCurrent(MotorID Id) const
{
    uint8_t motorId = static_cast<uint8_t>(Id);
    double current = 0.0;
    bool hasState = false;
    {
        std::lock_guard<std::mutex> stateLock(stateMutex);
        auto it = motorStates.find(motorId);
        if (it != motorStates.end()) {
            hasState = true;
            current = it->second.current;
        }
    }
    if (!hasState) {
        requestState(motorId);
        return 0;
    }
    return static_cast<int16_t>(std::lround(current * 100));
}

int16_t MtCan::getVelocity(MotorID Id) const
{
    uint8_t motorId = static_cast<uint8_t>(Id);
    int16_t velocity = 0;
    bool hasState = false;
    {
        std::lock_guard<std::mutex> stateLock(stateMutex);
        auto it = motorStates.find(motorId);
        if (it != motorStates.end()) {
            hasState = true;
            velocity = it->second.velocity;
        }
    }
    if (!hasState) {
        requestState(motorId);
        return 0;
    }
    if (velocity == 0) {
        requestState(motorId);
    }
    return velocity;
}

uint16_t MtCan::encodeSendCanId(uint8_t motorId) const
{
    return static_cast<uint16_t>(0x140 | motorId);//mt电机发送数据包时，第一帧为0x140+id
}

void MtCan::sendFrame(uint16_t canId, uint8_t command, const std::array<uint8_t, 4> &payload) const
{
    if (!canController) {
        std::cerr << "[MtCan] CAN controller not initialized\n";
        return;
    }

    CanTransport::Frame frame;
    frame.id = canId;
    frame.dlc = 8;
    frame.isExtended = false;
    frame.isRemoteRequest = false;
    frame.data[0] = command;
    frame.data[1] = 0x00;
    frame.data[2] = 0x00;
    frame.data[3] = 0x00;
    frame.data[4] = payload[0];
    frame.data[5] = payload[1];
    frame.data[6] = payload[2];
    frame.data[7] = payload[3];
    canController->send(frame);
}

void MtCan::requestState(uint8_t motorId) const
{
    const uint16_t canId = encodeSendCanId(motorId);
    if (!canController) {
        return;
    }

    CanTransport::Frame frame;
    frame.id = canId;
    frame.dlc = 1;
    frame.isExtended = false;
    frame.isRemoteRequest = false;
    frame.data[0] = 0x9C;
    canController->send(frame);
}

void MtCan::requestError(uint8_t motorId) const
{
    const uint16_t canId = encodeSendCanId(motorId);
    if (!canController) {
        return;
    }

    CanTransport::Frame frame;
    frame.id = canId;
    frame.dlc = 1;
    frame.isExtended = false;
    frame.isRemoteRequest = false;
    frame.data[0] = 0x9A;
    canController->send(frame);
}

void MtCan::resetSystem(uint8_t motorId) const
{
    const uint16_t canId = encodeSendCanId(motorId);
    sendFrame(canId, 0x76, {0, 0, 0, 0});
}

void MtCan::setZeroPosition(uint8_t motorId) const
{
    const uint16_t canId = encodeSendCanId(motorId);
    sendFrame(canId, 0x64, {0, 0, 0, 0});
}

void MtCan::refreshMotorStates()
{
    std::vector<uint8_t> motorIds;
    {
        std::lock_guard<std::mutex> lock(refreshMutex);
        motorIds = refreshMotorIds;
    }

    if (!canController) {
        return;
    }

    // 统一通过读状态命令触发实际位置/电流的刷新（由 socketcan 回调更新缓存）。
    for (uint8_t motorId : motorIds) {
        if (!refreshLoopActive.load()) {
            break;
        }
        requestState(motorId);
    }
}

void MtCan::stopRefreshLoop()
{
    refreshLoopActive.store(false);

    if (refreshThread.joinable()) {
        refreshThread.join();
    }
}

void MtCan::handleResponse(const CanTransport::Frame &frame)
{
    if (frame.isExtended) {
        return;
    }

    const uint16_t canId = static_cast<uint16_t>(frame.id & 0x7FF);
    if (canId < kResponseBaseId || canId >= kResponseBaseId + 0x100) {
        std::cerr << "[MtCan] Ignore frame with CAN ID 0x"
                  << std::hex << canId << std::dec << '\n';
        return;
    }

    if (frame.dlc == 0) {
        return;
    }

    const uint8_t command = frame.data[0];
    const uint8_t nodeId = static_cast<uint8_t>(canId & 0x00FF);
    bool shouldReset = false;

    {
        std::lock_guard<std::mutex> stateLock(stateMutex);
        MotorState &state = motorStates[nodeId];

        switch (command) {
        case 0x9C: {
            const int16_t rawCurrent = readInt16LE(frame, 2);
            state.current = static_cast<double>(rawCurrent) / 100.0;
            const int16_t rawVelocity = readInt16LE(frame, 4);
            state.velocity = static_cast<int16_t>(rawVelocity / 6);
            break;
        }
        case 0x9A: {
            const uint16_t errorCode = readUInt16LE(frame, 6);
            state.error = errorCode != 0;
            if (state.error) {
                std::cerr << "[MtCan] Motor " << static_cast<int>(nodeId)
                          << " error code " << errorCode << '\n';
            }
            break;
        }
        case 0x64:
            shouldReset = true;
            break;
        default:
            break;
        }
    }

    if (shouldReset) {
        resetSystem(nodeId);
    }
}
