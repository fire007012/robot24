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

    refreshThread = std::thread([this]() {
        while (refreshLoopActive.load()) {
            refreshMotorStates();
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    });
}

bool EyouCan::setMode(MotorID Id, MotorMode mode)
{
    if (!canController) {
        return false;
    }
    uint8_t motorId = static_cast<uint8_t>(Id);
    {
        std::lock_guard<std::mutex> stateLock(stateMutex);
        motorStates[motorId].mode = mode;
    }
    const uint32_t modeValue = mode == MotorMode::Velocity ? 0x00000003 : 0x00000001;
    sendWriteCommand(motorId, 0x0F, modeValue, 4);
    return true;
}

bool EyouCan::setVelocity(MotorID Id, int32_t velocity)
{
    if (!canController) {
        return false;
    }
    uint8_t motorId = static_cast<uint8_t>(Id);
    {
        std::lock_guard<std::mutex> stateLock(stateMutex);
        motorStates[motorId].commandedVelocity = velocity;
    }
    sendWriteCommand(motorId, 0x09, static_cast<uint32_t>(velocity), 4);
    return true;
}

// [FIX #5] 协议定义了 0x0B 为目标加速度，补充 CAN 发送
bool EyouCan::setAcceleration(MotorID Id, int32_t acceleration)
{
    if (!canController) {
        return false;
    }
    uint8_t motorId = static_cast<uint8_t>(Id);
    {
        std::lock_guard<std::mutex> stateLock(stateMutex);
        motorStates[motorId].acceleration = acceleration;
    }
    sendWriteCommand(motorId, 0x0B, static_cast<uint32_t>(acceleration), 4);
    return true;
}

// [FIX #6] 协议定义了 0x0C 为目标减速度，补充 CAN 发送
bool EyouCan::setDeceleration(MotorID Id, int32_t deceleration)
{
    if (!canController) {
        return false;
    }
    uint8_t motorId = static_cast<uint8_t>(Id);
    {
        std::lock_guard<std::mutex> stateLock(stateMutex);
        motorStates[motorId].deceleration = deceleration;
    }
    sendWriteCommand(motorId, 0x0C, static_cast<uint32_t>(deceleration), 4);
    return true;
}

bool EyouCan::setPosition(MotorID Id, int32_t position)
{
    if (!canController) {
        return false;
    }
    uint8_t motorId = static_cast<uint8_t>(Id);
    {
        std::lock_guard<std::mutex> stateLock(stateMutex);
        motorStates[motorId].commandedPosition = position;
    }
    sendWriteCommand(motorId, 0x0A, static_cast<uint32_t>(position), 4);
    return true;
}

bool EyouCan::Enable(MotorID Id)
{
    if (!canController) {
        return false;
    }
    uint8_t motorId = static_cast<uint8_t>(Id);
    {
        std::lock_guard<std::mutex> stateLock(stateMutex);
        motorStates[motorId].enabled = true;
    }
    sendWriteCommand(motorId, 0x10, 0x00000001, 4);
    return true;
}

bool EyouCan::Disable(MotorID Id)
{
    if (!canController) {
        return false;
    }
    uint8_t motorId = static_cast<uint8_t>(Id);
    {
        std::lock_guard<std::mutex> stateLock(stateMutex);
        motorStates[motorId].enabled = false;
    }
    sendWriteCommand(motorId, 0x10, 0x00000000, 4);
    return true;
}

// [FIX #1] 协议规定 0x11 写 01 结束当前运行，原代码写了 0x00
bool EyouCan::Stop(MotorID Id)
{
    if (!canController) {
        return false;
    }
    uint8_t motorId = static_cast<uint8_t>(Id);
    sendWriteCommand(motorId, 0x11, 0x00000001, 4);
    return true;
}

// [FIX #9] 移除 position==0 的不可靠判断，改用 hasReceived 标志
int32_t EyouCan::getPosition(MotorID Id) const
{
    uint8_t motorId = static_cast<uint8_t>(Id);
    {
        std::lock_guard<std::mutex> stateLock(stateMutex);
        auto it = motorStates.find(motorId);
        if (it != motorStates.end() && it->second.positionReceived) {
            return it->second.position;
        }
    }
    // 尚未收到过位置数据，主动请求一次
    requestPosition(motorId);
    return 0;
}

// [FIX #7] 读取协议 0x05（当前电流值），返回缓存值
int16_t EyouCan::getCurrent(MotorID Id) const
{
    uint8_t motorId = static_cast<uint8_t>(Id);
    {
        std::lock_guard<std::mutex> stateLock(stateMutex);
        auto it = motorStates.find(motorId);
        if (it != motorStates.end() && it->second.currentReceived) {
            return static_cast<int16_t>(it->second.current);
        }
    }
    requestCurrent(motorId);
    return 0;
}

// [FIX #8] 读取协议 0x06（当前速度值），返回缓存的实际速度
int16_t EyouCan::getVelocity(MotorID Id) const
{
    uint8_t motorId = static_cast<uint8_t>(Id);
    {
        std::lock_guard<std::mutex> stateLock(stateMutex);
        auto it = motorStates.find(motorId);
        if (it != motorStates.end() && it->second.velocityReceived) {
            return static_cast<int16_t>(it->second.actualVelocity);
        }
    }
    requestVelocity(motorId);
    return 0;
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
    // [FIX #10] 协议表头示例为8字节帧，用0填充尾部
    frame.dlc = 8;
    frame.data.fill(0);

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
    // [FIX #10] 统一 DLC 为 8
    frame.dlc = 8;
    frame.data.fill(0);
    frame.data[0] = kReadCommand;
    frame.data[1] = subCommand;
    canController->send(frame);
}

// [FIX #2, #3, #4] 重写 handleResponse，修正写返回解析和读返回偏移
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
            // [FIX #2, #3] 写返回格式: 0x02 ADDR STATE
            // STATE 是写入结果状态码，不是寄存器值
            // 0x01=成功, 0x05=矫正后成功, 其它=失败
            uint8_t writeState = dataByteOrZero(frame, 2);
            if (writeState != 0x01 && writeState != 0x05) {
                std::cerr << "[EyouCan] Write to motor " << static_cast<int>(motorId)
                          << " addr 0x" << std::hex << static_cast<int>(subCommand)
                          << " failed, state=0x" << static_cast<int>(writeState)
                          << std::dec << '\n';
                // 写入失败时回滚本地状态
                switch (subCommand) {
                case 0x10:
                    // Enable/Disable 写入失败，取反之前的乐观更新
                    state.enabled = !state.enabled;
                    break;
                default:
                    break;
                }
            }
            // 写入成功时不需要额外操作，因为 setMode/Enable/Disable 已乐观更新了状态

        } else if (responseType == kReadResponse) {
            // 读返回格式: 0x04 ADDR data0 data1 data2 data3
            // 32位数据从 data[2] 开始（即 DAT2~DAT5）
            switch (subCommand) {
            case 0x05:
                // [FIX #7] 当前电流值，32位有符号数，1=1mA
                state.current = readInt32BE(frame, 2);
                state.currentReceived = true;
                break;
            case 0x06:
                // [FIX #8] 当前速度值，32位有符号数
                state.actualVelocity = readInt32BE(frame, 2);
                state.velocityReceived = true;
                break;
            case 0x07:
                // 当前位置值
                state.position = readInt32BE(frame, 2);
                state.positionReceived = true;  // [FIX #9]
                break;
            case 0x0F:
                // 当前工作模式
                // 读返回: data[2..5] 为32位数据，模式值在最低字节
                state.mode = dataByteOrZero(frame, 5) == 0x03
                                 ? MotorMode::Velocity
                                 : MotorMode::Position;
                break;
            case 0x10:
                // 使能/失能状态
                // 读返回: 32位数据，01=使能 00=失能
                state.enabled = dataByteOrZero(frame, 5) != 0;
                break;
            case 0x15:
                // [FIX #4] 告警指示，数据从 data[2] 开始
                {
                    uint32_t errorCode = readUInt32BE(frame, 2);
                    if (errorCode != 0) {
                        std::cerr << "[EyouCan] Motor " << static_cast<int>(motorId)
                                  << " reported error code 0x" << std::hex
                                  << errorCode << std::dec << '\n';
                    }
                }
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

// [FIX #7] 新增：请求电流值
void EyouCan::requestCurrent(uint8_t motorId) const
{
    sendReadCommand(motorId, 0x05);
}

// [FIX #8] 新增：请求速度值
void EyouCan::requestVelocity(uint8_t motorId) const
{
    sendReadCommand(motorId, 0x06);
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

    for (uint8_t motorId : motorIds) {
        if (!refreshLoopActive.load()) {
            break;
        }
        requestPosition(motorId);
        requestMode(motorId);
        requestEnable(motorId);
        requestCurrent(motorId);   // [FIX #7] 轮询电流
        requestVelocity(motorId);  // [FIX #8] 轮询实际速度
    }
}

void EyouCan::stopRefreshLoop()
{
    refreshLoopActive.store(false);

    if (refreshThread.joinable()) {
        refreshThread.join();
    }
}
