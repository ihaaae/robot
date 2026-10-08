#include "shensi/can/fake_transport.hpp"

#include <stdexcept>
#include <utility>

namespace shensi::can {

void FakeTransport::send(const Frame& frame) {
    if (!healthy_) throw std::runtime_error("FakeTransport: not healthy");
    sent_.push_back(frame);
    // Call a copy: a responder that installs a new responder must not destroy the closure
    // that is running.
    if (responder_) {
        const Responder responder = responder_;
        responder(frame, *this);
    }
}

void FakeTransport::set_receiver(Receiver receiver) { receiver_ = std::move(receiver); }

bool FakeTransport::healthy() const { return healthy_; }

void FakeTransport::set_responder(Responder responder) { responder_ = std::move(responder); }

void FakeTransport::set_healthy(bool healthy) { healthy_ = healthy; }

void FakeTransport::inject(const Frame& frame) {
    if (receiver_) {
        const Receiver receiver = receiver_;  // same reason as in send()
        receiver(frame);
    }
}

std::size_t FakeTransport::sent_count(std::uint32_t id) const {
    std::size_t count = 0;
    for (const Frame& frame : sent_) {
        if (frame.id == id) ++count;
    }
    return count;
}

void FakeTransport::clear() { sent_.clear(); }

}  // namespace shensi::can
