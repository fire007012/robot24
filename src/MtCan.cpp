#include "MtCan.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <iomanip>
#include <iostream>

namespace {
constexpr std::size_t kFrameSize = 13;
constexpr uint16_t kResponseBaseId = 0x240;

int16_t readInt16LE(const CanTransport::Buffer &frame, std::size_t offset)
{
    const int value = static_cast<int>(frame[offset]) |
                      (static_cast<int>(frame[offset + 1]) << 8);
    return static_cast<int16_t>(value);
}

uint16_t readUInt16LE(const CanTransport::Buffer &frame, std::size_t offset)
{
    return static_cast<uint16_t>(static_cast<uint16_t>(frame[offset]) |
                                 (static_cast<uint16_t>(frame[offset + 1]) << 8));
}

uint16_t readCanId(const CanTransport::Buffer &frame, std::size_t offset)
{
    return static_cast<uint16_t>((static_cast<uint16_t>(frame[offset]) << 8) |
                                 static_cast<uint16_t>(frame[offset + 1]));
}
} // namespace

MtCan::MtCan(std::shared_ptr<CanTransport> controller)
    : canController(std::move(controller))
{
    if (canController) {
        receiveHandlerId = canController->addReceiveHandler(
            [this](const CanTransport::Buffer &payload) { handleResponse(payload); });
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

    // 独立线程专门跑 1ms 轮询，避免堵塞 Qt 主线程。
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

    std::array<uint8_t, 4> payload {
        static_cast<uint8_t>(velocity & 0x00FF),
        static_cast<uint8_t>((velocity >> 8) & 0x00FF),
        static_cast<uint8_t>(position & 0x00FF),
        static_cast<uint8_t>((position >> 8) & 0x00FF)
    };

    std::array<uint8_t, kFrameSize> frame {};
    frame[0] = 0x08;
    frame[3] = static_cast<uint8_t>((canId >> 8) & 0xFF);
    frame[4] = static_cast<uint8_t>(canId & 0xFF);
    frame[5] = 0xA4;
    frame[6] = 0x00;
    frame[7] = payload[0];
    frame[8] = payload[1];
    frame[9] = payload[2];
    frame[10] = payload[3];
    frame[11] = static_cast<uint8_t>((position >> 16) & 0xFF);
    frame[12] = static_cast<uint8_t>((position >> 24) & 0xFF);

    if (canController) {
        canController->send(frame.data(), frame.size());
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

    std::array<uint8_t, kFrameSize> frame {};
    frame[0] = 0x08;
    frame[3] = static_cast<uint8_t>((canId >> 8) & 0xFF);
    frame[4] = static_cast<uint8_t>(canId & 0xFF);
    frame[5] = command;
    frame[9] = payload[0];
    frame[10] = payload[1];
    frame[11] = payload[2];
    frame[12] = payload[3];
    canController->send(frame.data(), frame.size());
}

void MtCan::requestState(uint8_t motorId) const
{
    const uint16_t canId = encodeSendCanId(motorId);
    if (!canController) {
        return;
    }

    std::array<uint8_t, kFrameSize> frame {};
    frame[0] = 0x08;
    frame[3] = static_cast<uint8_t>((canId >> 8) & 0xFF);
    frame[4] = static_cast<uint8_t>(canId & 0xFF);
    frame[5] = 0x9C;
    canController->send(frame.data(), frame.size());
}

void MtCan::requestError(uint8_t motorId) const
{
    const uint16_t canId = encodeSendCanId(motorId);
    if (!canController) {
        return;
    }

    std::array<uint8_t, kFrameSize> frame {};
    frame[0] = 0x08;
    frame[3] = static_cast<uint8_t>((canId >> 8) & 0xFF);
    frame[4] = static_cast<uint8_t>(canId & 0xFF);
    frame[5] = 0x9A;
    canController->send(frame.data(), frame.size());
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

    // 统一通过读状态命令触发实际位置/电流的刷新（UDP 回调中更新缓存）。
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

void MtCan::handleResponse(const CanTransport::Buffer &data)
{
    if (data.size() < kFrameSize) {
        return;
    }

    for (std::size_t offset = 0; offset + kFrameSize <= data.size(); offset += kFrameSize) {
        const uint16_t canId = readCanId(data, offset + 3);
        if (canId < kResponseBaseId || canId >= kResponseBaseId + 0x100) {
            std::cerr << "[MtCan] Ignore frame with CAN ID 0x"
                      << std::hex << canId << std::dec << '\n';
            continue;
        }

        const uint8_t command = static_cast<uint8_t>(data[offset + 5]);
        const uint8_t nodeId = static_cast<uint8_t>(canId & 0x00FF);
        bool shouldReset = false;

        {
            std::lock_guard<std::mutex> stateLock(stateMutex);
            MotorState &state = motorStates[nodeId];

            switch (command) {
            case 0x9C: {
                const int16_t rawCurrent = readInt16LE(data, offset + 7);
                state.current = static_cast<double>(rawCurrent) / 100.0;
                const int16_t rawVelocity = readInt16LE(data, offset + 9);
                state.velocity = static_cast<int16_t>(rawVelocity / 6);
                break;
            }
            case 0x9A: {
                const uint16_t errorCode = readUInt16LE(data, offset + 11);
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
}
