#ifndef CAN_DRIVER_CAN_TRANSPORT_H
#define CAN_DRIVER_CAN_TRANSPORT_H

#include <cstddef>
#include <cstdint>
#include <functional>
#include <vector>

/**
 * @brief Lightweight transport abstraction to decouple protocol logic from the underlying bus.
 *
 * The transport works with the legacy 13-byte frame layout that higher level protocol code already uses:
 * - byte 0 stores DLC (<= 8)
 * - bytes 3-4 store the CAN identifier (standard 11-bit)
 * - bytes 5-12 store up to 8 data bytes
 *
 * Implementations are responsible for translating this layout to their specific backend (UDP tunnel, SocketCAN, etc.).
 */
class CanTransport {
public:
    using Buffer = std::vector<std::uint8_t>;
    using ReceiveHandler = std::function<void(const Buffer &)>;

    virtual ~CanTransport() = default;

    virtual void send(const std::uint8_t *data, std::size_t size) = 0;

    void send(const Buffer &buffer)
    {
        send(buffer.data(), buffer.size());
    }

    virtual std::size_t addReceiveHandler(ReceiveHandler handler) = 0;
    virtual void removeReceiveHandler(std::size_t handlerId) = 0;
};

#endif // CAN_DRIVER_CAN_TRANSPORT_H
