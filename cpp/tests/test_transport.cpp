// Tests for the transport seam and the fake backend.
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "check.hpp"
#include "shensi/can/fake_transport.hpp"
#include "shensi/can/wire.hpp"

using namespace shensi::can;
using shensi::can::from_hex;
using shensi::can::to_hex;

SHENSI_TEST_CASE(fake_transport_records_what_was_sent) {
    FakeTransport bus;
    CHECK(bus.healthy());
    CHECK_EQ(bus.sent_count(), static_cast<std::size_t>(0));

    bus.send(encode_sdo(sdo_write2(0x01, 0x6040, 0, 0x0006)));
    bus.send(encode_sdo(sdo_write2(0x01, 0x6040, 0, 0x0007)));
    bus.send(encode_sdo(sdo_read(0x03, 0x6064, 0)));

    CHECK_EQ(bus.sent_count(), static_cast<std::size_t>(3));
    CHECK_EQ(bus.sent_count(0x601), static_cast<std::size_t>(2));
    CHECK_EQ(bus.sent_count(0x603), static_cast<std::size_t>(1));
    CHECK_EQ(bus.sent_count(0x581), static_cast<std::size_t>(0));

    bus.clear();
    CHECK_EQ(bus.sent_count(), static_cast<std::size_t>(0));
}

SHENSI_TEST_CASE(a_responder_can_script_an_sdo_exchange) {
    FakeTransport bus;

    std::vector<Frame> received;
    bus.set_receiver([&](const Frame& frame) { received.push_back(frame); });

    // Answer a read of 0x6064 with the frame PR0002 §4.12 prints.
    const std::vector<std::uint8_t> answer = from_hex("43 64 60 00 FE 3F 00 00");
    bus.set_responder([&](const Frame& sent, FakeTransport& self) {
        SdoRequest request;
        if (!decode_sdo_request(sent, request)) return;
        if (request.cmd != SdoCmd::Read || request.index != 0x6064) return;
        self.inject(frame_from_bytes(Bus::Can0, kSdoResponseBase | request.dev_id, answer.data(),
                                     kSdoDlc));
    });

    bus.send(encode_sdo(sdo_read(0x01, 0x6064, 0)));

    CHECK_EQ(bus.sent_count(), static_cast<std::size_t>(1));
    CHECK_EQ(received.size(), static_cast<std::size_t>(1));
    CHECK_EQ(received[0].id, static_cast<std::uint32_t>(0x581));

    SdoResponse response;
    CHECK(decode_sdo(received[0], response));
    CHECK_EQ(static_cast<int>(response.index), 0x6064);
    CHECK_EQ(response.value, static_cast<std::uint32_t>(0x3FFE));

    // A read of an object nobody answers must not produce a frame.
    bus.send(encode_sdo(sdo_read(0x01, 0x1234, 0)));
    CHECK_EQ(received.size(), static_cast<std::size_t>(1));
}

SHENSI_TEST_CASE(an_unhealthy_transport_refuses_to_send) {
    FakeTransport bus;
    bus.set_healthy(false);
    CHECK(!bus.healthy());
    const Frame frame = encode_sdo(sdo_read(0x01, 0x6064, 0));
    CHECK_THROWS(bus.send(frame));
    CHECK_EQ(bus.sent_count(), static_cast<std::size_t>(0));

    bus.set_healthy(true);
    bus.send(frame);
    CHECK_EQ(bus.sent_count(), static_cast<std::size_t>(1));
}

SHENSI_TEST_CASE(send_raw_carries_frames_with_no_modelled_layout) {
    FakeTransport bus;

    const std::vector<std::uint8_t> payload = from_hex("01 02 03 04 05 06 07");
    bus.send_raw(Bus::Can1, 0x123, payload.data(), 7);

    CHECK_EQ(bus.sent_count(), static_cast<std::size_t>(1));
    const Frame& sent = bus.sent()[0];
    CHECK_EQ(sent.id, static_cast<std::uint32_t>(0x123));
    CHECK_EQ(static_cast<int>(sent.len), 7);
    CHECK_EQ(static_cast<int>(sent.bus), static_cast<int>(Bus::Can1));
    CHECK(sent.brs);
    CHECK(sent.fdf);
    CHECK_EQ(to_hex(sent.data.data(), sent.len), to_hex(payload.data(), payload.size()));
}

SHENSI_TEST_CASE(vendor_moveend_frame_is_a_single_axis_command_to_dev8) {
    // The vendor's MoveEnd builds C4 HI LO 03 E8 00 00 and sends it on 0x108, DLC 7
    // (research/evidence/disasm/executor.MoveEnd.txt). Here HI LO = 01 2C, i.e. 300.
    FakeTransport bus;
    const std::vector<std::uint8_t> payload = from_hex("C4 01 2C 03 E8 00 00");
    bus.send_raw(Bus::Can0, 0x108, payload.data(), 7);
    const Frame& sent = bus.sent()[0];

    CHECK_EQ(static_cast<int>(classify(sent)), static_cast<int>(FrameClass::SingleAxisCommand));
    CHECK_EQ(static_cast<int>(dev_id_of(sent)), 8);

    ControlSubframe sub;
    CHECK(decode_single_axis(sent, sub));
    CHECK(sub.enable);
    CHECK(sub.brake_release);
    CHECK(!sub.clear_error);
    CHECK_EQ(static_cast<int>(sub.mode), 2);  // profile velocity
    CHECK_EQ(static_cast<int>(sub.target1), 300);
    CHECK_EQ(static_cast<int>(sub.target2), 1000);  // acceleration, RPM/s
    CHECK_EQ(static_cast<int>(sub.feedforward), 0);

    // So the typed encoder reproduces it byte for byte.
    const Frame typed = encode_single_axis(Bus::Can0, 8, sub);
    CHECK_EQ(to_hex(typed.data.data(), typed.len), to_hex(payload.data(), payload.size()));
}

SHENSI_TEST_CASE(send_raw_can_disable_the_fd_flags) {
    FakeTransport bus;
    const std::vector<std::uint8_t> payload = from_hex("AA BB");
    bus.send_raw(Bus::Can0, 0x50, payload.data(), 2, /*brs=*/false, /*fdf=*/false);

    const Frame& sent = bus.sent()[0];
    CHECK(!sent.brs);
    CHECK(!sent.fdf);
    CHECK_EQ(static_cast<int>(classify(sent)), static_cast<int>(FrameClass::Unknown));
    CHECK_EQ(to_hex(sent.data.data(), sent.len), std::string("aabb"));
}

SHENSI_TEST_CASE(a_receiver_only_sees_injected_frames) {
    FakeTransport bus;
    int calls = 0;
    Frame last;
    bus.set_receiver([&](const Frame& frame) {
        ++calls;
        last = frame;
    });

    bus.send(encode_sdo(sdo_read(0x01, 0x6064, 0)));
    CHECK_EQ(calls, 0);  // sending does not echo back

    const std::vector<std::uint8_t> bytes = from_hex("09 E1 FD E5 FF 51 00 00 00 F0 01 C0");
    bus.inject(frame_from_bytes(Bus::Can1, 0x301, bytes.data(), kFeedbackDlc));

    CHECK_EQ(calls, 1);
    CHECK_EQ(last.id, static_cast<std::uint32_t>(0x301));
    CHECK_EQ(static_cast<int>(last.bus), static_cast<int>(Bus::Can1));

    JointFeedback feedback;
    CHECK(decode_feedback(last, feedback));
    CHECK_EQ(feedback.pos_cnt, 2529);
    CHECK_EQ(feedback.vel_rpm, -539);
    CHECK_EQ(feedback.current_ma, -175);
}
