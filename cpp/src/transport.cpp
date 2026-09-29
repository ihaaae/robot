#include "shensi/can/transport.hpp"

namespace shensi::can {

void Transport::send_raw(Bus bus, std::uint32_t id, const std::uint8_t* data, std::uint8_t len,
                         bool brs, bool fdf) {
    Frame frame = frame_from_bytes(bus, id, data, len);
    frame.brs = brs;
    frame.fdf = fdf;
    send(frame);
}

}  // namespace shensi::can
