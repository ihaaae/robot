// An in-process transport for tests and for running the stack with no bus.
//
// It records what was sent, lets a test script a response to each sent frame, and lets a test
// inject frames as if they had arrived. Everything above L0 can therefore be exercised
// offline, which is the point: nothing in this repository has run on the robot yet.
#ifndef SHENSI_CAN_FAKE_TRANSPORT_HPP
#define SHENSI_CAN_FAKE_TRANSPORT_HPP

#include <cstddef>
#include <cstdint>
#include <functional>
#include <vector>

#include "shensi/can/frame.hpp"
#include "shensi/can/transport.hpp"

namespace shensi::can {

class FakeTransport final : public Transport {
public:
    // Called after each send, so the responder can reply through inject().
    using Responder = std::function<void(const Frame& sent, FakeTransport& self)>;

    void send(const Frame& frame) override;
    void set_receiver(Receiver receiver) override;
    bool healthy() const override;

    void set_responder(Responder responder);
    void set_healthy(bool healthy);

    // Deliver a frame to the receiver, as the bus would.
    void inject(const Frame& frame);

    const std::vector<Frame>& sent() const { return sent_; }
    std::size_t sent_count() const { return sent_.size(); }
    std::size_t sent_count(std::uint32_t id) const;
    void clear();

private:
    std::vector<Frame> sent_;
    Receiver receiver_;
    Responder responder_;
    bool healthy_ = true;
};

}  // namespace shensi::can

#endif  // SHENSI_CAN_FAKE_TRANSPORT_HPP
