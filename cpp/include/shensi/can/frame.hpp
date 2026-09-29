// L0 wire format: the frame container and the CAN identifier map.
//
// See docs/l0-interface.md. L0 owns bytes <-> wire structs, sending and receiving frames,
// and (next) the trace/diff harness. It owns no policy, no sequencing and no state.
#ifndef SHENSI_CAN_FRAME_HPP
#define SHENSI_CAN_FRAME_HPP

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace shensi::can {

enum class Bus : std::uint8_t { Can0 = 0, Can1 = 1 };

inline constexpr std::size_t kMaxDlc = 64;

// One CAN FD frame, as it goes on the wire.
//
// `id` is the 11-bit standard identifier: PR0002 §5 uses standard frames only, so there is
// no room here for EFF/RTR flags. `len` is the payload length in bytes, not the DLC code.
struct Frame {
    Bus bus = Bus::Can0;
    std::uint32_t id = 0;
    std::uint8_t len = 0;
    bool brs = true;  // data-phase bit rate switch (PR0002 §3.1: 1 Mbps arb / 5 Mbps data, BRS on)
    bool fdf = true;  // CAN FD frame
    std::array<std::uint8_t, kMaxDlc> data{};
};

// Identifier map, PR0002 §4-§7. Everything except the NMT, sync and broadcast identifiers
// carries Dev_ID in the low byte.
inline constexpr std::uint32_t kNmtId = 0x000;  // DLC 2, node-addressed in the payload
inline constexpr std::uint32_t kSyncId = 0x080;  // DLC 0
inline constexpr std::uint32_t kSingleAxisBase = 0x100;  // + Dev_ID, DLC 7
inline constexpr std::uint32_t kMitSingleBase = 0x110;  // + Dev_ID, DLC 9
inline constexpr std::uint32_t kMultiAxisId = 0x200;  // DLC 64: 8 subframes + 8 Dev_IDs
inline constexpr std::uint32_t kMitMultiAxisId = 0x210;  // DLC 64, up to 6 MIT subframes
inline constexpr std::uint32_t kFeedbackBase = 0x300;  // + Dev_ID, DLC 12
inline constexpr std::uint32_t kSdoResponseBase = 0x580;  // + Dev_ID, DLC 8
inline constexpr std::uint32_t kSdoRequestBase = 0x600;  // + Dev_ID, DLC 8
inline constexpr std::uint32_t kHeartbeatBase = 0x700;  // + Dev_ID, DLC 1

inline constexpr std::uint8_t kNmtDlc = 2;
inline constexpr std::uint8_t kSdoDlc = 8;
inline constexpr std::uint8_t kSingleAxisDlc = 7;
inline constexpr std::uint8_t kMitSingleDlc = 9;
inline constexpr std::uint8_t kFeedbackDlc = 12;
inline constexpr std::uint8_t kMultiAxisDlc = 64;
inline constexpr std::uint8_t kHeartbeatDlc = 1;

inline constexpr std::uint8_t kMinDevId = 0x01;  // PR0002 §3.2: 0x01..0x7F
inline constexpr std::uint8_t kMaxDevId = 0x7F;

// What a frame is, decided by identifier *and* length.
//
// The length matters because 0x100 | Dev_ID (DLC 7) and 0x110 | Dev_ID (DLC 9) overlap for
// Dev_ID >= 0x11: 0x111 is both "single-axis control, Dev_ID 0x11" and "MIT single, Dev_ID
// 0x01". The document's Dev_ID range is 0x01..0x7F, so the overlap is real, not theoretical.
// Only the DLC tells them apart, which is why the document's own examples use DLC 7 / DLC 9.
enum class FrameClass {
    Unknown,
    Nmt,
    Sync,
    SdoRequest,
    SdoResponse,
    SingleAxisCommand,
    MitSingleCommand,
    MultiAxisCommand,
    MitMultiAxisCommand,
    Feedback,
    Heartbeat,
};

FrameClass classify(const Frame& frame);
const char* to_string(FrameClass kind);

// The device a frame is addressed to, or 0 when the identifier is not device-scoped.
std::uint8_t dev_id_of(const Frame& frame);

// Build a frame from bytes, the way PR0002 writes them out.
Frame frame_from_bytes(Bus bus, std::uint32_t id, const std::uint8_t* data, std::uint8_t len);

// Lowercase hex with no separators: how the trace format, the golden vectors and the test
// failure messages all write frames.
std::string to_hex(const std::uint8_t* data, std::size_t len);
std::string to_hex(const Frame& frame);

// Parses hex with optional whitespace between bytes ("2B406000" and "2B 40 60 00" both work).
// Throws std::invalid_argument on an odd digit count or a character that is not hex and not
// whitespace -- a corrupt trace should say so rather than decode into something plausible.
std::vector<std::uint8_t> from_hex(const std::string& text);

}  // namespace shensi::can

#endif  // SHENSI_CAN_FRAME_HPP
