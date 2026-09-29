// L0 transport seam.
//
// The shape mirrors the vendor's RK3576CanCanfd (can_send_frame + setReadFunction) so the
// vendor register driver can be one backend behind this interface.
//
// OWNERSHIP RULE (docs/l0-interface.md §5). A process that owns the bus must not also
// instantiate Juxie::ControllerJuxie:
//
//   * Two transports in one process remap the same /dev/mem window and the same
//     /dev/misc_shm_can* queue, and become two consumers of one frame queue, so frames split
//     between them nondeterministically.
//   * SDO (0x600 | Dev_ID) is request/response with no source address, so two masters'
//     conversations collide.
//   * The watchdog is fed by periodic control frames (< 500 ms, PR0002 §1). With two partial
//     owners, "the other layer is feeding it" is a mid-motion self-lock.
//   * Once OnRobot() has run, the vendor stack is never quiescent until process exit.
//
// The migration is therefore incremental inside the SDK and atomic at the bus.
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

    // Escape hatch for frames with no published layout -- for example the 7-byte frame that
    // MoveEnd emits on 0x108. Note that 0x108 is 0x100 | 0x08, inside the single-axis command
    // range: the joints use Dev_ID 01..07 (PR0002 §5.3), so 08 is free, and on identifier and
    // length alone that frame is indistinguishable from "a single-axis command to device 8".
    // Whether it is one is unresolved, which is why it gets no type here.
    // See docs/l0-interface.md §3.7.
    void send_raw(Bus bus, std::uint32_t id, const std::uint8_t* data, std::uint8_t len,
                  bool brs = true, bool fdf = true);
};

}  // namespace shensi::can

#endif  // SHENSI_CAN_TRANSPORT_HPP
