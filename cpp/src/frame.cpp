#include "shensi/can/frame.hpp"

#include <stdexcept>

namespace shensi::can {
namespace {

// True when `id` is `base + Dev_ID` for a Dev_ID in the documented range.
bool in_device_range(std::uint32_t id, std::uint32_t base) {
    return id >= base && id <= base + kMaxDevId;
}

}  // namespace

FrameClass classify(const Frame& frame) {
    const std::uint32_t id = frame.id;
    const std::uint8_t len = frame.len;

    // Exact identifiers first: 0x200 and 0x210 are fixed, and 0x210 also falls inside the
    // 0x200..0x27F window, so a range test would be wrong.
    if (id == kNmtId && len == kNmtDlc) return FrameClass::Nmt;
    if (id == kSyncId && len == 0) return FrameClass::Sync;
    if (id == kMultiAxisId && len == kMultiAxisDlc) return FrameClass::MultiAxisCommand;
    if (id == kMitMultiAxisId && len == kMultiAxisDlc) return FrameClass::MitMultiAxisCommand;

    // 0x100 | Dev_ID and 0x110 | Dev_ID overlap above Dev_ID 0x10; the DLC is what separates
    // them. Check the MIT range first only for the shared ids -- the two lengths are disjoint,
    // so the order does not actually matter, but the comment records why both are needed.
    if (in_device_range(id, kMitSingleBase) && len == kMitSingleDlc) return FrameClass::MitSingleCommand;
    if (in_device_range(id, kSingleAxisBase) && len == kSingleAxisDlc) return FrameClass::SingleAxisCommand;

    if (in_device_range(id, kFeedbackBase) && len == kFeedbackDlc) return FrameClass::Feedback;
    if (in_device_range(id, kSdoResponseBase) && len == kSdoDlc) return FrameClass::SdoResponse;
    if (in_device_range(id, kSdoRequestBase) && len == kSdoDlc) return FrameClass::SdoRequest;
    if (in_device_range(id, kHeartbeatBase) && len == kHeartbeatDlc) return FrameClass::Heartbeat;

    return FrameClass::Unknown;
}

const char* to_string(FrameClass kind) {
    switch (kind) {
        case FrameClass::Unknown: return "unknown";
        case FrameClass::Nmt: return "nmt";
        case FrameClass::Sync: return "sync";
        case FrameClass::SdoRequest: return "sdo_request";
        case FrameClass::SdoResponse: return "sdo_response";
        case FrameClass::SingleAxisCommand: return "single_axis_command";
        case FrameClass::MitSingleCommand: return "mit_single_command";
        case FrameClass::MultiAxisCommand: return "multi_axis_command";
        case FrameClass::MitMultiAxisCommand: return "mit_multi_axis_command";
        case FrameClass::Feedback: return "feedback";
        case FrameClass::Heartbeat: return "heartbeat";
    }
    return "unknown";
}

std::uint8_t dev_id_of(const Frame& frame) {
    // Dev_ID is the offset from the frame class's own base, not a bit mask. The two
    // single-axis bases overlap (0x111 is 0x100|0x11 and 0x110|0x01 at the same time), so the
    // class -- which classify() resolves using the DLC -- decides which base applies.
    switch (classify(frame)) {
        case FrameClass::SdoRequest:
            return static_cast<std::uint8_t>(frame.id - kSdoRequestBase);
        case FrameClass::SdoResponse:
            return static_cast<std::uint8_t>(frame.id - kSdoResponseBase);
        case FrameClass::Feedback:
            return static_cast<std::uint8_t>(frame.id - kFeedbackBase);
        case FrameClass::Heartbeat:
            return static_cast<std::uint8_t>(frame.id - kHeartbeatBase);
        case FrameClass::SingleAxisCommand:
            return static_cast<std::uint8_t>(frame.id - kSingleAxisBase);
        case FrameClass::MitSingleCommand:
            return static_cast<std::uint8_t>(frame.id - kMitSingleBase);
        default:
            // NMT carries its node in the payload, not the identifier; the broadcast frames
            // are not addressed to one device.
            return 0;
    }
}

Frame frame_from_bytes(Bus bus, std::uint32_t id, const std::uint8_t* data, std::uint8_t len) {
    if (len > kMaxDlc) throw std::invalid_argument("frame_from_bytes: len exceeds 64");
    Frame frame;
    frame.bus = bus;
    frame.id = id;
    frame.len = len;
    for (std::uint8_t i = 0; i < len; ++i) frame.data[i] = data[i];
    return frame;
}

std::string to_hex(const std::uint8_t* data, std::size_t len) {
    static const char* digits = "0123456789abcdef";
    std::string out;
    out.reserve(len * 2);
    for (std::size_t i = 0; i < len; ++i) {
        out.push_back(digits[data[i] >> 4]);
        out.push_back(digits[data[i] & 0x0F]);
    }
    return out;
}

std::string to_hex(const Frame& frame) { return to_hex(frame.data.data(), frame.len); }

std::vector<std::uint8_t> from_hex(const std::string& text) {
    std::vector<std::uint8_t> out;
    int high = -1;
    for (std::size_t i = 0; i < text.size(); ++i) {
        const char c = text[i];
        int digit = -1;
        if (c >= '0' && c <= '9') {
            digit = c - '0';
        } else if (c >= 'a' && c <= 'f') {
            digit = c - 'a' + 10;
        } else if (c >= 'A' && c <= 'F') {
            digit = c - 'A' + 10;
        } else if (c == ' ' || c == '\t' || c == '_') {
            continue;  // grouping separators are allowed
        } else {
            throw std::invalid_argument(std::string("from_hex: not a hex digit: '") + c + "'");
        }
        if (high < 0) {
            high = digit;
        } else {
            out.push_back(static_cast<std::uint8_t>((high << 4) | digit));
            high = -1;
        }
    }
    if (high >= 0) throw std::invalid_argument("from_hex: an odd number of hex digits");
    return out;
}

}  // namespace shensi::can
