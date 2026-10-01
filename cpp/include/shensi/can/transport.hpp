// L0 transport seam.
//
// Send plus a receive callback: small enough that SocketCAN, a USB-CAN adapter, or the
// controller board's register driver can each be one backend behind it.
//
// OWNERSHIP RULE (docs/l0-interface.md §5). A bus has exactly one master. While our stack owns
// it, no other master may run on it -- the vendor stack included:
//
//   * Two transports in one process remap the same /dev/mem window and the same
//     /dev/misc_shm_can* queue, and become two consumers of one frame queue, so frames split
//     between them nondeterministically.
//   * SDO (0x600 | Dev_ID) is request/response with no source address, so two masters'
//     conversations collide.
//   * The watchdog is fed by periodic control frames (< 500 ms, PR0002 §1). With two partial
//     owners, "the other master is feeding it" is a mid-motion self-lock.
//
// Switching between stacks is therefore whole: one stops, then the other starts.
#ifndef SHENSI_CAN_TRANSPORT_HPP
#define SHENSI_CAN_TRANSPORT_HPP

#include <cstdint>
#include <functional>

#include "shensi/can/frame.hpp"

namespace shensi::can {

class Transport {
public:
    // Called on the transport's own thread, when it has one.
    using Receiver = std::function<void(const Frame&)>;

    Transport() = default;
    Transport(const Transport&) = delete;
    Transport& operator=(const Transport&) = delete;
    virtual ~Transport() = default;

    // Non-blocking. Throws std::runtime_error if the transport is not healthy.
    virtual void send(const Frame& frame) = 0;

    virtual void set_receiver(Receiver receiver) = 0;

    virtual bool healthy() const = 0;

    // Escape hatch for frames with no published layout, so they need not be forced into the
    // typed encoders.
    void send_raw(Bus bus, std::uint32_t id, const std::uint8_t* data, std::uint8_t len,
                  bool brs = true, bool fdf = true);
};

}  // namespace shensi::can

#endif  // SHENSI_CAN_TRANSPORT_HPP
