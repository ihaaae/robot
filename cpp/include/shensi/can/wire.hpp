// L0 wire format: the typed codecs.
//
// Pure functions: bytes in, structs out, no I/O and no state. This is where the golden
// vectors from PR0002 live. See docs/l0-interface.md §3.
//
// THE ONE THING TO GET RIGHT: this protocol uses two byte orders.
//   * SDO payloads (0x600/0x580) are LITTLE-endian.
//   * Custom PDO payloads (0x100, 0x200, 0x300) are BIG-endian for their 16-bit fields.
// Verified against PR0002's own examples: the SDO index 0x6040 encodes as "40 60", while the
// control frame's target 16384 encodes as "40 00".
#ifndef SHENSI_CAN_WIRE_HPP
#define SHENSI_CAN_WIRE_HPP

#include <array>
#include <cstddef>
#include <cstdint>

#include "shensi/can/frame.hpp"

namespace shensi::can {

// ---------------------------------------------------------------- units (PR0002 §6, §8)

inline constexpr double kPi = 3.14159265358979323846;
inline constexpr double kCntPerRev = 65536.0;  // load-side encoder: 16 bit single-turn

// Rounds to the nearest count, halves away from zero. That halves the worst-case error of
// truncation (the vendor driver's choice) and keeps it symmetric in sign, and it makes
// cnt -> rad -> cnt the identity. Pinned by a unit test: a one-count difference is invisible to
// any property test.
std::int16_t rad_to_cnt(double rad);
double cnt_to_rad(std::int16_t cnt);

// ---------------------------------------------------------------- SDO (PR0002 §4.1)

// The command byte doubles as the value width: 2Fh writes 1 byte, 2Bh 2, 27h 3, 23h 4; 40h is
// a read request; 60h is a write acknowledgement; 4Fh/4Bh/47h/43h are read responses of 1/2/3/4
// bytes.
enum class SdoCmd : std::uint8_t {
    Write1 = 0x2F,
    Write2 = 0x2B,
    Write3 = 0x27,
    Write4 = 0x23,
    Read = 0x40,
    AckWrite = 0x60,
    Read1 = 0x4F,
    Read2 = 0x4B,
    Read3 = 0x47,
    Read4 = 0x43,
};

// Number of value bytes a command carries, or 0 when it carries none.
int sdo_value_width(std::uint8_t cmd);
bool is_sdo_write(std::uint8_t cmd);
bool is_sdo_read_response(std::uint8_t cmd);

struct SdoRequest {
    std::uint8_t dev_id = 0;
    SdoCmd cmd = SdoCmd::Read;
    std::uint16_t index = 0;
    std::uint8_t sub = 0;
    std::uint32_t value = 0;  // only the low sdo_value_width(cmd) bytes reach the wire
};

struct SdoResponse {
    std::uint8_t dev_id = 0;
    SdoCmd cmd = SdoCmd::AckWrite;
    std::uint16_t index = 0;
    std::uint8_t sub = 0;
    std::uint32_t value = 0;  // zero-extended
    int value_width = 0;

    // Sign-extends `value` using `value_width`. Needed for signed objects such as the
    // negative limit 0x607D:01, whose example value is FFFF80C8.
    std::int32_t value_signed() const;
};

// Explicit widths, because the document is explicit: 0x6040 = 6 is sent with 2Bh (2 bytes),
// so choosing "the narrowest width that fits" would not reproduce the documented frames.
SdoRequest sdo_write1(std::uint8_t dev_id, std::uint16_t index, std::uint8_t sub, std::uint8_t value);
SdoRequest sdo_write2(std::uint8_t dev_id, std::uint16_t index, std::uint8_t sub, std::uint16_t value);
SdoRequest sdo_write3(std::uint8_t dev_id, std::uint16_t index, std::uint8_t sub, std::uint32_t value);
SdoRequest sdo_write4(std::uint8_t dev_id, std::uint16_t index, std::uint8_t sub, std::uint32_t value);
SdoRequest sdo_read(std::uint8_t dev_id, std::uint16_t index, std::uint8_t sub);

Frame encode_sdo(const SdoRequest& request);

// False when `frame` is not an SDO response at all (wrong identifier range, wrong DLC, or an
// unknown command byte). The caller decides whether that is an error.
bool decode_sdo(const Frame& frame, SdoResponse& out);

// The request direction. Kept symmetric with decode_sdo so a trace, a test or the diff
// harness can read back what was sent without re-deriving the layout.
bool decode_sdo_request(const Frame& frame, SdoRequest& out);

// ------------------------------------------- single-axis control subframe (PR0002 §5.1)

// The project's own mode values, not the 6060h compatibility numbers (PR0002 §3.4). The custom
// fast-control frames use these directly and do not run the 6060h mapping (§5.2):
//   1 PP / 2 PV / 3 CSP / 4 CSV / 5 current / 6 MIT / 7 torque-sensor closed loop.
//
// `ControlSubframe::mode` and `JointFeedback::mode` stay plain bytes rather than an enum: the
// document's own §5.3 example decodes to 8, which is outside that table, so a strict enum would
// have to lie about it. See docs/l0-interface.md §8.

// 7 bytes. 16-bit fields are BIG-endian here. The same layout is the payload of a single-axis
// frame (0x100 + Dev_ID) and one subframe of the 0x200 broadcast.
struct ControlSubframe {
    bool enable = false;         // byte0 bit7, 1 = 上使能
    bool brake_release = false;  // byte0 bit6, 1 = 抱闸释放
    bool clear_error = false;    // byte0 bit5, 1 = 复位错误
    std::uint8_t mode = 0;       // byte0 bit4..1, a MotionMode value
    std::int16_t target1 = 0;    // bytes1..2; meaning depends on mode
    std::int16_t target2 = 0;    // bytes3..4; profile acceleration in PP/PV modes
    std::int16_t feedforward = 0;  // bytes5..6; output-side profile velocity in position modes
};

std::array<std::uint8_t, kSingleAxisDlc> encode_subframe(const ControlSubframe& sub);

// False when fewer than kSingleAxisDlc bytes are available. This takes a length on purpose:
// the vendor's own getFKpose has a fixed-buffer overflow of exactly this shape (a pointer and
// an assumed size, research/vendor-analysis/sdk-usage.md §6.1), so no decoder here trusts a bare pointer.
bool decode_subframe(const std::uint8_t* bytes, std::size_t len, ControlSubframe& out);

Frame encode_single_axis(Bus bus, std::uint8_t dev_id, const ControlSubframe& sub);
bool decode_single_axis(const Frame& frame, ControlSubframe& out);

// ------------------------------------------------- broadcast frame (PR0002 §5.3)

// 64 bytes: eight 7-byte subframes in [0..55], then the eight Dev_IDs in [56..63]. An unused
// slot is zero-filled in both halves; dev_ids[i] == 0 marks slot i unused.
struct MultiAxisCommand {
    std::array<ControlSubframe, 8> subframes{};
    std::array<std::uint8_t, 8> dev_ids{};
};

Frame encode_multi_axis(const MultiAxisCommand& command);
bool decode_multi_axis(const Frame& frame, MultiAxisCommand& out);

// --------------------------------------------------- actuator feedback (PR0002 §6)

// 12 bytes. Every 16-bit field is BIG-endian, the opposite of SDO.
struct JointFeedback {
    std::int16_t pos_cnt = 0;     // [0..1], -32768..32767 maps to -180..180 deg
    std::int16_t vel_rpm = 0;     // [2..3], motor side
    std::int16_t current_ma = 0;  // [4..5], Iq
    std::uint16_t fault = 0;      // [6..7], see research/vendor-analysis/error-codes.md
    std::int16_t temp_dc = 0;     // [8..9], 0.1 degC
    std::uint8_t mode = 0;        // [10], a MotionMode value
    bool enabled = false;         // [11] bit7
    bool brake_released = false;  // [11] bit6
    bool error = false;           // [11] bit5
    bool in_position = false;     // [11] bit4

    double position_rad() const { return cnt_to_rad(pos_cnt); }
    double temperature_c() const { return temp_dc / 10.0; }
};

bool decode_feedback(const Frame& frame, JointFeedback& out);

// ---------------------------------------------------- NMT and sync (PR0002 §7)

// CiA 301 network management. Note the document's Chinese labels for 0x81 and 0x01 are loose:
// by CiA 301, 0x81 is "reset node" and 0x01 is "start remote node". Node 0 means broadcast.
enum class NmtCommand : std::uint8_t {
    Start = 0x01,
    Stop = 0x02,
    EnterPreOperational = 0x80,
    ResetNode = 0x81,
    ResetCommunication = 0x82,
};

Frame encode_nmt(NmtCommand command, std::uint8_t node);
Frame encode_sync(Bus bus = Bus::Can0);

}  // namespace shensi::can

#endif  // SHENSI_CAN_WIRE_HPP
