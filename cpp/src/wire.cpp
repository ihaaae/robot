#include "shensi/can/wire.hpp"

#include <cmath>
#include <stdexcept>

namespace shensi::can {
namespace {

// PR0002 §6 / §5.1 write 16-bit fields most-significant byte first in the custom PDO frames.
std::uint16_t read_be16(const std::uint8_t* p) {
    return static_cast<std::uint16_t>((static_cast<std::uint16_t>(p[0]) << 8) | p[1]);
}

void write_be16(std::uint8_t* p, std::int16_t value) {
    const std::uint16_t bits = static_cast<std::uint16_t>(value);
    p[0] = static_cast<std::uint8_t>(bits >> 8);
    p[1] = static_cast<std::uint8_t>(bits & 0xFF);
}

// SDO is little-endian: index in [1..2], value in [4..7] (PR0002 §4.1).
std::uint16_t read_le16(const std::uint8_t* p) {
    return static_cast<std::uint16_t>(p[0] | (static_cast<std::uint16_t>(p[1]) << 8));
}

void write_le16(std::uint8_t* p, std::uint16_t value) {
    p[0] = static_cast<std::uint8_t>(value & 0xFF);
    p[1] = static_cast<std::uint8_t>(value >> 8);
}

}  // namespace

// ------------------------------------------------------------------ units

std::int16_t rad_to_cnt(double rad) {
    // A NaN passes both clamps below and the cast is undefined; it must never become a target.
    if (!std::isfinite(rad)) throw std::invalid_argument("rad_to_cnt: not a finite angle");
    // Nearest count, halves away from zero (wire.hpp).
    const double cnt = std::round(rad / (2.0 * kPi) * kCntPerRev);
    if (cnt >= 32767.0) return 32767;
    if (cnt <= -32768.0) return -32768;
    return static_cast<std::int16_t>(cnt);
}

double cnt_to_rad(std::int16_t cnt) { return static_cast<double>(cnt) * kPi / 32768.0; }

// ------------------------------------------------------------------ SDO

int sdo_value_width(std::uint8_t cmd) {
    switch (cmd) {
        case 0x2F: return 1;
        case 0x2B: return 2;
        case 0x27: return 3;
        case 0x23: return 4;
        case 0x60: return 0;
        case 0x40: return 0;
        case 0x4F: return 1;
        case 0x4B: return 2;
        case 0x47: return 3;
        case 0x43: return 4;
        default: return -1;  // not a command this protocol defines
    }
}

bool is_sdo_write(std::uint8_t cmd) {
    return cmd == 0x2F || cmd == 0x2B || cmd == 0x27 || cmd == 0x23;
}

bool is_sdo_read_response(std::uint8_t cmd) {
    return cmd == 0x4F || cmd == 0x4B || cmd == 0x47 || cmd == 0x43;
}

bool is_sdo_response(std::uint8_t cmd) {
    return cmd == 0x60 || is_sdo_read_response(cmd);
}

std::int32_t SdoResponse::value_signed() const {
    if (value_width <= 0 || value_width >= 4) return static_cast<std::int32_t>(value);
    const std::uint32_t sign_bit = 1u << (value_width * 8 - 1);
    if ((value & sign_bit) == 0) return static_cast<std::int32_t>(value);
    const std::uint32_t mask = (1u << (value_width * 8)) - 1u;
    return static_cast<std::int32_t>(value | ~mask);
}

SdoRequest sdo_write1(std::uint8_t dev_id, std::uint16_t index, std::uint8_t sub, std::uint8_t value) {
    return SdoRequest{dev_id, SdoCmd::Write1, index, sub, value};
}

SdoRequest sdo_write2(std::uint8_t dev_id, std::uint16_t index, std::uint8_t sub, std::uint16_t value) {
    return SdoRequest{dev_id, SdoCmd::Write2, index, sub, value};
}

SdoRequest sdo_write3(std::uint8_t dev_id, std::uint16_t index, std::uint8_t sub, std::uint32_t value) {
    return SdoRequest{dev_id, SdoCmd::Write3, index, sub, value};
}

SdoRequest sdo_write4(std::uint8_t dev_id, std::uint16_t index, std::uint8_t sub, std::uint32_t value) {
    return SdoRequest{dev_id, SdoCmd::Write4, index, sub, value};
}

SdoRequest sdo_read(std::uint8_t dev_id, std::uint16_t index, std::uint8_t sub) {
    return SdoRequest{dev_id, SdoCmd::Read, index, sub, 0};
}

namespace {

// Shared by both directions: the layout is the same, only the identifier base differs.
Frame encode_sdo_frame(std::uint32_t base, std::uint8_t dev_id, std::uint8_t cmd,
                       std::uint16_t index, std::uint8_t sub, std::uint32_t value) {
    const int width = sdo_value_width(cmd);
    Frame frame;
    frame.id = base | dev_id;
    frame.len = kSdoDlc;
    frame.data[0] = cmd;
    write_le16(&frame.data[1], index);
    frame.data[3] = sub;
    // Only the bytes the command width makes meaningful reach the wire; the rest stay zero.
    // PR0002 §4.1 leaves them undefined, and sending a caller's unused high bytes there would
    // put unspecified content in an unspecified field.
    frame.data[4] = 0;
    frame.data[5] = 0;
    frame.data[6] = 0;
    frame.data[7] = 0;
    for (int i = 0; i < width; ++i) {
        frame.data[4 + i] = static_cast<std::uint8_t>((value >> (8 * i)) & 0xFF);
    }
    return frame;
}

}  // namespace

Frame encode_sdo(const SdoRequest& request) {
    const std::uint8_t cmd = static_cast<std::uint8_t>(request.cmd);
    // A response byte on 0x600 + Dev_ID would be a frame decode_sdo_request itself rejects.
    if (!is_sdo_write(cmd) && cmd != 0x40) {
        throw std::invalid_argument("encode_sdo: not a request command byte");
    }
    if (request.dev_id > kMaxDevId) throw std::invalid_argument("encode_sdo: Dev_ID out of range");
    return encode_sdo_frame(kSdoRequestBase, request.dev_id, cmd, request.index, request.sub,
                            request.value);
}

Frame encode_sdo_response(const SdoResponse& response) {
    const std::uint8_t cmd = static_cast<std::uint8_t>(response.cmd);
    if (!is_sdo_response(cmd)) {
        throw std::invalid_argument("encode_sdo_response: not a response command byte");
    }
    if (response.dev_id > kMaxDevId) {
        throw std::invalid_argument("encode_sdo_response: Dev_ID out of range");
    }
    return encode_sdo_frame(kSdoResponseBase, response.dev_id, cmd, response.index, response.sub,
                            response.value);
}

bool decode_sdo(const Frame& frame, SdoResponse& out) {
    if (classify(frame) != FrameClass::SdoResponse) return false;

    const std::uint8_t cmd = frame.data[0];
    const int width = sdo_value_width(cmd);
    if (width < 0) return false;
    // A request command byte on a response identifier is not a response.
    if (!is_sdo_response(cmd)) return false;

    out.dev_id = dev_id_of(frame);
    out.cmd = static_cast<SdoCmd>(cmd);
    out.index = read_le16(&frame.data[1]);
    out.sub = frame.data[3];
    out.value_width = width;
    std::uint32_t value = 0;
    for (int i = 0; i < width; ++i) {
        value |= static_cast<std::uint32_t>(frame.data[4 + i]) << (8 * i);
    }
    out.value = value;
    return true;
}

bool decode_sdo_request(const Frame& frame, SdoRequest& out) {
    if (classify(frame) != FrameClass::SdoRequest) return false;

    const std::uint8_t cmd = frame.data[0];
    const int width = sdo_value_width(cmd);
    if (width < 0) return false;
    // A response command byte on a request identifier is not a request.
    if (is_sdo_response(cmd)) return false;

    out.dev_id = dev_id_of(frame);
    out.cmd = static_cast<SdoCmd>(cmd);
    out.index = read_le16(&frame.data[1]);
    out.sub = frame.data[3];
    std::uint32_t value = 0;
    for (int i = 0; i < width; ++i) {
        value |= static_cast<std::uint32_t>(frame.data[4 + i]) << (8 * i);
    }
    out.value = value;
    return true;
}

// ------------------------------------------------------------------ control subframe

std::array<std::uint8_t, kSingleAxisDlc> encode_subframe(const ControlSubframe& sub) {
    std::array<std::uint8_t, kSingleAxisDlc> out{};
    std::uint8_t byte0 = 0;
    if (sub.enable) byte0 |= 0x80;
    if (sub.brake_release) byte0 |= 0x40;
    if (sub.clear_error) byte0 |= 0x20;
    byte0 |= static_cast<std::uint8_t>((sub.mode & 0x0F) << 1);
    out[0] = byte0;
    write_be16(&out[1], sub.target1);
    write_be16(&out[3], sub.target2);
    write_be16(&out[5], sub.feedforward);
    return out;
}

bool decode_subframe(const std::uint8_t* bytes, std::size_t len, ControlSubframe& out) {
    if (bytes == nullptr || len < kSingleAxisDlc) return false;
    const std::uint8_t byte0 = bytes[0];
    out.enable = (byte0 & 0x80) != 0;
    out.brake_release = (byte0 & 0x40) != 0;
    out.clear_error = (byte0 & 0x20) != 0;
    out.mode = static_cast<std::uint8_t>((byte0 >> 1) & 0x0F);
    out.target1 = static_cast<std::int16_t>(read_be16(&bytes[1]));
    out.target2 = static_cast<std::int16_t>(read_be16(&bytes[3]));
    out.feedforward = static_cast<std::int16_t>(read_be16(&bytes[5]));
    return true;
}

Frame encode_single_axis(Bus bus, std::uint8_t dev_id, const ControlSubframe& sub) {
    if (dev_id > kMaxDevId) throw std::invalid_argument("encode_single_axis: Dev_ID out of range");
    const auto bytes = encode_subframe(sub);
    return frame_from_bytes(bus, kSingleAxisBase | dev_id, bytes.data(), kSingleAxisDlc);
}

bool decode_single_axis(const Frame& frame, ControlSubframe& out) {
    if (classify(frame) != FrameClass::SingleAxisCommand) return false;
    return decode_subframe(frame.data.data(), kSingleAxisDlc, out);
}

// ------------------------------------------------------------------ broadcast

Frame encode_multi_axis(const MultiAxisCommand& command) {
    Frame frame;
    frame.id = kMultiAxisId;
    frame.len = kMultiAxisDlc;
    for (std::size_t slot = 0; slot < 8; ++slot) {
        const auto bytes = encode_subframe(command.subframes[slot]);
        for (std::size_t i = 0; i < kSingleAxisDlc; ++i) {
            frame.data[slot * kSingleAxisDlc + i] = bytes[i];
        }
    }
    for (std::size_t slot = 0; slot < 8; ++slot) {
        frame.data[56 + slot] = command.dev_ids[slot];
    }
    return frame;
}

bool decode_multi_axis(const Frame& frame, MultiAxisCommand& out) {
    if (classify(frame) != FrameClass::MultiAxisCommand) return false;
    for (std::size_t slot = 0; slot < 8; ++slot) {
        if (!decode_subframe(&frame.data[slot * kSingleAxisDlc], kSingleAxisDlc,
                             out.subframes[slot])) {
            return false;
        }
        out.dev_ids[slot] = frame.data[56 + slot];
    }
    return true;
}

// ------------------------------------------------------------------ feedback

bool decode_feedback(const Frame& frame, JointFeedback& out) {
    if (classify(frame) != FrameClass::Feedback) return false;
    const std::uint8_t* p = frame.data.data();
    out.pos_cnt = static_cast<std::int16_t>(read_be16(&p[0]));
    out.vel_rpm = static_cast<std::int16_t>(read_be16(&p[2]));
    out.current_ma = static_cast<std::int16_t>(read_be16(&p[4]));
    out.fault = read_be16(&p[6]);
    out.temp_dc = static_cast<std::int16_t>(read_be16(&p[8]));
    out.mode = p[10];
    out.enabled = (p[11] & 0x80) != 0;
    out.brake_released = (p[11] & 0x40) != 0;
    out.error = (p[11] & 0x20) != 0;
    out.in_position = (p[11] & 0x10) != 0;
    return true;
}

// ------------------------------------------------------------------ NMT / sync

Frame encode_nmt(NmtCommand command, std::uint8_t node) {
    Frame frame;
    frame.id = kNmtId;
    frame.len = kNmtDlc;
    frame.data[0] = static_cast<std::uint8_t>(command);
    frame.data[1] = node;
    return frame;
}

Frame encode_sync(Bus bus) {
    Frame frame;
    frame.bus = bus;
    frame.id = kSyncId;
    frame.len = 0;
    return frame;
}

}  // namespace shensi::can
