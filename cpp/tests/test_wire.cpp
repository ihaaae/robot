// Golden vectors for the L0 wire codec.
//
// Every frame in this file is copied out of PR0002. The rule for this file is: the document is
// the source of truth, not the vendor binary. Where the document contradicts itself or its own
// examples, the contradiction is pinned as a test rather than silently smoothed over -- see
// `pr0002_documented_anomalies`.
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "check.hpp"
#include "shensi/can/wire.hpp"

using namespace shensi::can;
using shensi::can::from_hex;
using shensi::can::to_hex;

namespace {

// ------------------------------------------------------------------ SDO

struct SdoVector {
    const char* bytes;
    SdoCmd cmd;
    std::uint16_t index;
    std::uint8_t sub;
    std::uint32_t value;
};

// Requests, all on 0x601. The write widths are the document's, not "the narrowest that fits":
// 0x6040 = 6 is sent with 2Bh (2 bytes), so an auto-width encoder would not reproduce these.
const SdoVector kSdoRequests[] = {
    // §4.2 read version information
    {"40 08 10 00 00 00 00 00", SdoCmd::Read, 0x1008, 0, 0},
    {"40 00 10 00 00 00 00 00", SdoCmd::Read, 0x1000, 0, 0},
    {"40 0A 10 00 00 00 00 00", SdoCmd::Read, 0x100A, 0, 0},
    {"40 09 10 00 00 00 00 00", SdoCmd::Read, 0x1009, 0, 0},
    // §4.3 set motor id
    {"23 30 25 00 05 00 00 00", SdoCmd::Write4, 0x2530, 0, 5},
    // §4.4 zero calibration
    {"23 31 25 00 01 00 00 00", SdoCmd::Write4, 0x2531, 0, 1},
    // §4.5 read actual position
    {"40 64 60 00 00 00 00 00", SdoCmd::Read, 0x6064, 0, 0},
    // §4.6 watchdog and heartbeat
    {"23 50 26 00 01 00 00 00", SdoCmd::Write4, 0x2650, 0, 1},
    {"2B 17 10 00 D0 07 00 00", SdoCmd::Write2, 0x1017, 0, 2000},
    // §4.7 limits and CAN bit rate
    {"23 7D 60 02 30 7F 00 00", SdoCmd::Write4, 0x607D, 2, 0x00007F30},
    {"40 7D 60 01 00 00 00 00", SdoCmd::Read, 0x607D, 1, 0},
    {"23 7D 60 01 C8 80 FF FF", SdoCmd::Write4, 0x607D, 1, 0xFFFF80C8},
    {"40 7D 60 02 00 00 00 00", SdoCmd::Read, 0x607D, 2, 0},
    {"40 40 25 00 00 00 00 00", SdoCmd::Read, 0x2540, 0, 0},
    {"23 40 25 00 04 00 00 00", SdoCmd::Write4, 0x2540, 0, 4},
    // §4.8 profile position mode
    {"2B 40 60 00 06 00 00 00", SdoCmd::Write2, 0x6040, 0, 0x06},
    {"2B 40 60 00 07 00 00 00", SdoCmd::Write2, 0x6040, 0, 0x07},
    {"2B 40 60 00 0F 00 00 00", SdoCmd::Write2, 0x6040, 0, 0x0F},
    {"2F 60 60 00 01 00 00 00", SdoCmd::Write1, 0x6060, 0, 1},
    {"23 83 60 00 D0 07 00 00", SdoCmd::Write4, 0x6083, 0, 2000},
    {"23 84 60 00 D0 07 00 00", SdoCmd::Write4, 0x6084, 0, 2000},
    {"23 81 60 00 0A 00 00 00", SdoCmd::Write4, 0x6081, 0, 10},
    {"23 7A 60 00 00 40 00 00", SdoCmd::Write4, 0x607A, 0, 16384},
    {"2B 40 60 00 4F 00 00 00", SdoCmd::Write2, 0x6040, 0, 0x4F},
    // §4.9 profile velocity mode
    {"2F 60 60 00 03 00 00 00", SdoCmd::Write1, 0x6060, 0, 3},
    {"23 83 60 00 E8 03 00 00", SdoCmd::Write4, 0x6083, 0, 1000},
    {"23 84 60 00 E8 03 00 00", SdoCmd::Write4, 0x6084, 0, 1000},
    {"23 FF 60 00 F4 01 00 00", SdoCmd::Write4, 0x60FF, 0, 500},
    // §4.10 current mode and torque-sensor closed loop
    {"2F 60 60 00 0A 00 00 00", SdoCmd::Write1, 0x6060, 0, 10},
    {"2B 71 60 00 F4 01 00 00", SdoCmd::Write2, 0x6071, 0, 500},
    {"2F 60 60 00 07 00 00 00", SdoCmd::Write1, 0x6060, 0, 7},
    {"2B 71 60 00 0A 00 00 00", SdoCmd::Write2, 0x6071, 0, 10},
    // §4.11 PI parameters and flash save
    {"23 32 25 00 00 00 00 00", SdoCmd::Write4, 0x2532, 0, 0},
    {"23 33 25 00 00 00 00 00", SdoCmd::Write4, 0x2533, 0, 0},
    {"23 34 25 00 00 00 00 00", SdoCmd::Write4, 0x2534, 0, 0},
    {"23 35 25 00 00 00 00 00", SdoCmd::Write4, 0x2535, 0, 0},
    {"23 36 25 00 00 00 00 00", SdoCmd::Write4, 0x2536, 0, 0},
    {"23 37 25 00 00 00 00 00", SdoCmd::Write4, 0x2537, 0, 0},
    {"23 39 25 00 01 00 00 00", SdoCmd::Write4, 0x2539, 0, 1},
    {"40 32 25 00 00 00 00 00", SdoCmd::Read, 0x2532, 0, 0},
    {"40 33 25 00 00 00 00 00", SdoCmd::Read, 0x2533, 0, 0},
    {"40 34 25 00 00 00 00 00", SdoCmd::Read, 0x2534, 0, 0},
    {"40 35 25 00 00 00 00 00", SdoCmd::Read, 0x2535, 0, 0},
    {"40 36 25 00 00 00 00 00", SdoCmd::Read, 0x2536, 0, 0},
    {"40 37 25 00 00 00 00 00", SdoCmd::Read, 0x2537, 0, 0},
    {"40 38 25 00 00 00 00 00", SdoCmd::Read, 0x2538, 0, 0},
    // §4.12 feedback reads
    {"40 78 60 00 00 00 00 00", SdoCmd::Read, 0x6078, 0, 0},
    {"40 6C 60 00 00 00 00 00", SdoCmd::Read, 0x606C, 0, 0},
    {"40 3F 60 00 00 00 00 00", SdoCmd::Read, 0x603F, 0, 0},
    {"40 41 60 00 00 00 00 00", SdoCmd::Read, 0x6041, 0, 0},
    {"40 62 26 00 00 00 00 00", SdoCmd::Read, 0x2662, 0, 0},
    {"40 63 26 00 00 00 00 00", SdoCmd::Read, 0x2663, 0, 0},
    // §7 PDO configuration
    {"23 00 18 01 81 01 00 80", SdoCmd::Write4, 0x1800, 1, 0x80000181},
    {"23 01 18 01 81 02 00 80", SdoCmd::Write4, 0x1801, 1, 0x80000281},
    {"23 02 18 01 81 03 00 80", SdoCmd::Write4, 0x1802, 1, 0x80000381},
    {"23 03 18 01 81 04 00 80", SdoCmd::Write4, 0x1803, 1, 0x80000481},
    {"23 00 14 01 01 02 00 80", SdoCmd::Write4, 0x1400, 1, 0x80000201},
    {"23 01 14 01 01 03 00 80", SdoCmd::Write4, 0x1401, 1, 0x80000301},
    {"23 02 14 01 01 04 00 80", SdoCmd::Write4, 0x1402, 1, 0x80000401},
    {"23 03 14 01 01 05 00 80", SdoCmd::Write4, 0x1403, 1, 0x80000501},
    {"2F 00 18 02 01 00 00 00", SdoCmd::Write1, 0x1800, 2, 1},
    {"2F 00 1A 00 00 00 00 00", SdoCmd::Write1, 0x1A00, 0, 0},
    {"23 00 1A 01 10 00 41 60", SdoCmd::Write4, 0x1A00, 1, 0x60410010},
    {"23 00 1A 02 10 00 78 60", SdoCmd::Write4, 0x1A00, 2, 0x60780010},
    {"23 00 1A 03 20 00 64 60", SdoCmd::Write4, 0x1A00, 3, 0x60640020},
    {"2F 00 1A 00 03 00 00 00", SdoCmd::Write1, 0x1A00, 0, 3},
    {"23 00 18 01 81 01 00 00", SdoCmd::Write4, 0x1800, 1, 0x00000181},
    {"2F 00 14 02 01 00 00 00", SdoCmd::Write1, 0x1400, 2, 1},
    {"2F 00 16 00 00 00 00 00", SdoCmd::Write1, 0x1600, 0, 0},
    {"23 00 16 01 10 00 40 60", SdoCmd::Write4, 0x1600, 1, 0x60400010},
    {"23 00 16 02 10 00 71 60", SdoCmd::Write4, 0x1600, 2, 0x60710010},
    {"23 00 16 03 20 00 7A 60", SdoCmd::Write4, 0x1600, 3, 0x607A0020},
    {"2F 00 16 00 03 00 00 00", SdoCmd::Write1, 0x1600, 0, 3},
    {"23 00 14 01 01 02 00 00", SdoCmd::Write4, 0x1400, 1, 0x00000201},
    {"2F 60 60 00 08 00 00 00", SdoCmd::Write1, 0x6060, 0, 8},
};

// Responses, all on 0x581.
const SdoVector kSdoResponses[] = {
    {"60 17 10 00 00 00 00 00", SdoCmd::AckWrite, 0x1017, 0, 0},
    {"60 00 18 01 00 00 00 00", SdoCmd::AckWrite, 0x1800, 1, 0},
    {"60 01 18 01 00 00 00 00", SdoCmd::AckWrite, 0x1801, 1, 0},
    {"60 02 18 01 00 00 00 00", SdoCmd::AckWrite, 0x1802, 1, 0},
    {"60 03 18 01 00 00 00 00", SdoCmd::AckWrite, 0x1803, 1, 0},
    {"60 00 14 01 00 00 00 00", SdoCmd::AckWrite, 0x1400, 1, 0},
    {"60 00 18 02 00 00 00 00", SdoCmd::AckWrite, 0x1800, 2, 0},
    {"60 00 1A 00 00 00 00 00", SdoCmd::AckWrite, 0x1A00, 0, 0},
    {"60 00 1A 01 00 00 00 00", SdoCmd::AckWrite, 0x1A00, 1, 0},
    {"60 00 1A 02 00 00 00 00", SdoCmd::AckWrite, 0x1A00, 2, 0},
    {"60 00 1A 03 00 00 00 00", SdoCmd::AckWrite, 0x1A00, 3, 0},
    {"60 00 14 02 00 00 00 00", SdoCmd::AckWrite, 0x1400, 2, 0},
    {"60 60 60 00 00 00 00 00", SdoCmd::AckWrite, 0x6060, 0, 0},
    // §7: the document's response to `2F 00 16 00 ...` (clear rxPDO1) carries sub 01, not 00.
    // Recorded here as written; see pr0002_documented_anomalies.
    {"60 00 16 01 00 00 00 00", SdoCmd::AckWrite, 0x1600, 1, 0},
    // §4.12 read responses
    {"43 64 60 00 FE 3F 00 00", SdoCmd::Read4, 0x6064, 0, 0x3FFE},
    {"4B 78 60 00 45 01 00 00", SdoCmd::Read2, 0x6078, 0, 0x0145},
    {"43 6C 60 00 F9 01 00 00", SdoCmd::Read4, 0x606C, 0, 0x01F9},
    {"4B 3F 60 00 00 00 00 00", SdoCmd::Read2, 0x603F, 0, 0},
    {"4B 41 60 00 00 00 00 00", SdoCmd::Read2, 0x6041, 0, 0},
    {"43 62 26 00 18 01 00 00", SdoCmd::Read4, 0x2662, 0, 0x0118},
    {"43 63 26 00 18 01 00 00", SdoCmd::Read4, 0x2663, 0, 0x0118},
};

// Build through the public builders rather than an aggregate, so the golden-vector test
// exercises the path L1 will actually call.
SdoRequest build_request(const SdoVector& vector) {
    switch (vector.cmd) {
        case SdoCmd::Write1:
            return sdo_write1(0x01, vector.index, vector.sub,
                              static_cast<std::uint8_t>(vector.value));
        case SdoCmd::Write2:
            return sdo_write2(0x01, vector.index, vector.sub,
                              static_cast<std::uint16_t>(vector.value));
        case SdoCmd::Write3:
            return sdo_write3(0x01, vector.index, vector.sub, vector.value);
        case SdoCmd::Write4:
            return sdo_write4(0x01, vector.index, vector.sub, vector.value);
        case SdoCmd::Read:
            return sdo_read(0x01, vector.index, vector.sub);
        default:
            return SdoRequest{0x01, vector.cmd, vector.index, vector.sub, vector.value};
    }
}

// ------------------------------------------------------------------ control subframes (§5.2)

struct SubframeVector {
    const char* bytes;
    std::uint8_t mode;
    std::int16_t target1;
    std::int16_t target2;
    std::int16_t feedforward;
};

const SubframeVector kSubframeVectors[] = {
    {"C2 00 00 07 D0 00 05", 1, 0, 2000, 5},
    {"C2 40 00 03 E8 00 08", 1, 16384, 1000, 8},
    {"C2 C0 00 03 E8 00 08", 1, -16384, 1000, 8},
    {"C4 01 F4 03 E8 00 00", 2, 500, 1000, 0},
    {"C4 FE 0C 03 E8 00 00", 2, -500, 1000, 0},
    {"CA 03 E8 00 00 00 00", 5, 1000, 0, 0},
    {"CA FC 18 00 00 00 00", 5, -1000, 0, 0},
    {"C6 00 32 00 00 00 00", 3, 50, 0, 0},
    {"C6 00 64 00 00 00 00", 3, 100, 0, 0},
    {"CE 00 0A 00 00 00 00", 7, 10, 0, 0},
};

// ------------------------------------------------------------------ feedback (§6, §5.2)

struct FeedbackVector {
    const char* bytes;
    std::int16_t pos_cnt;
    std::int16_t vel_rpm;
    std::int16_t current_ma;
    std::uint16_t fault;
    std::int16_t temp_dc;
    std::uint8_t mode;
    bool enabled;
    bool brake_released;
    bool error;
    bool in_position;
};

const FeedbackVector kFeedbackVectors[] = {
    // §6, the worked decode
    {"09 E1 FD E5 FF 51 00 00 00 F0 01 C0", 2529, -539, -175, 0x0000, 240, 1, true, true, false, false},
    // §5.2, the frame that answers the CSP/PP example
    {"FF FE 00 00 FF E2 00 00 01 04 01 D0", -2, 0, -30, 0x0000, 260, 1, true, true, false, true},
};

// ------------------------------------------------------------------ broadcast (§5.3)

const char* const kSevenAxisBroadcast =
    "d016e500000000"
    "d02d1500000000"
    "d017ea00000000"
    "d0fd6500000000"
    "d0e10700000000"
    "d0f4e900000000"
    "d008c800000000"
    "00000000000000"
    "0102030405060700";

// decode_subframe reports failure through its return value; the golden-vector loop wants the
// value, so it fails the case rather than silently decoding a default-constructed struct.
ControlSubframe decode_subframe_or_fail(const std::vector<std::uint8_t>& bytes) {
    ControlSubframe out;
    if (!decode_subframe(bytes.data(), bytes.size(), out)) {
        shensi::test::report_failure(__FILE__, __LINE__, "decode_subframe rejected a golden vector");
    }
    return out;
}

}  // namespace

SHENSI_TEST_CASE(sdo_requests_encode_to_the_documented_bytes) {
    for (const SdoVector& vector : kSdoRequests) {
        const std::vector<std::uint8_t> want = from_hex(vector.bytes);
        CHECK_EQ(want.size(), static_cast<std::size_t>(kSdoDlc));

        SdoRequest request = build_request(vector);
        request.dev_id = 0x01;

        const Frame frame = encode_sdo(request);
        CHECK_EQ(frame.id, static_cast<std::uint32_t>(0x601));
        CHECK_EQ(static_cast<int>(frame.len), static_cast<int>(kSdoDlc));
        CHECK_EQ(static_cast<int>(classify(frame)), static_cast<int>(FrameClass::SdoRequest));
        const std::string got = to_hex(frame.data.data(), frame.len);
        const std::string expected = to_hex(want.data(), want.size());
        CHECK_MSG(got == expected, std::string("vector \"") + vector.bytes + "\"\n      got  " +
                                       got + "\n      want " + expected);
    }
}

SHENSI_TEST_CASE(sdo_three_byte_writes_round_trip) {
    // PR0002's command table defines 27h (write 3 bytes) but prints no example frame, so this is
    // a round trip rather than a golden vector: the document fixes the layout, not this payload.
    const SdoRequest request = sdo_write3(0x02, 0x1234, 0x05, 0x00ABCDEFu);
    const Frame frame = encode_sdo(request);
    CHECK_EQ(frame.id, static_cast<std::uint32_t>(0x602));
    // Only the three meaningful bytes reach the wire; byte 7 stays zero.
    CHECK_EQ(to_hex(frame.data.data(), frame.len), std::string("27341205efcdab00"));

    SdoRequest decoded;
    CHECK(decode_sdo_request(frame, decoded));
    CHECK_EQ(decoded.value, static_cast<std::uint32_t>(0x00ABCDEF));
    CHECK_EQ(static_cast<int>(decoded.index), 0x1234);
    CHECK_EQ(static_cast<int>(decoded.sub), 0x05);
}

SHENSI_TEST_CASE(sdo_responses_decode_from_the_documented_bytes) {
    for (const SdoVector& vector : kSdoResponses) {
        const std::vector<std::uint8_t> bytes = from_hex(vector.bytes);
        CHECK_EQ(bytes.size(), static_cast<std::size_t>(kSdoDlc));

        const Frame frame =
            frame_from_bytes(Bus::Can0, kSdoResponseBase | 0x01, bytes.data(), kSdoDlc);

        SdoResponse response;
        CHECK(decode_sdo(frame, response));
        CHECK_EQ(static_cast<int>(response.cmd), static_cast<int>(vector.cmd));
        CHECK_EQ(static_cast<int>(response.index), static_cast<int>(vector.index));
        CHECK_EQ(static_cast<int>(response.sub), static_cast<int>(vector.sub));
        CHECK_EQ(response.value, vector.value);
        CHECK_EQ(response.dev_id, static_cast<std::uint8_t>(0x01));
    }
}

SHENSI_TEST_CASE(sdo_round_trips_through_encode_and_decode) {
    const std::vector<SdoVector> everything = [] {
        std::vector<SdoVector> all;
        for (const SdoVector& v : kSdoRequests) all.push_back(v);
        for (const SdoVector& v : kSdoResponses) all.push_back(v);
        return all;
    }();

    for (const SdoVector& vector : everything) {
        const std::vector<std::uint8_t> bytes = from_hex(vector.bytes);

        // Decode with the command byte swapped onto the matching identifier, so both
        // directions are exercised on the same payload.
        const std::uint8_t cmd = bytes[0];
        const bool is_write = is_sdo_write(cmd) || cmd == 0x40;
        const std::uint32_t id = is_write ? kSdoRequestBase : kSdoResponseBase;
        const Frame frame = frame_from_bytes(Bus::Can0, id | 0x01, bytes.data(), kSdoDlc);

        if (is_write) {
            SdoResponse response;
            // A write command byte on a response identifier is not a response.
            CHECK(!decode_sdo(frame, response));
        } else {
            SdoResponse response;
            CHECK(decode_sdo(frame, response));
            const SdoRequest request{response.dev_id, response.cmd, response.index, response.sub,
                                     response.value};
            const Frame reencoded = encode_sdo(request);
            CHECK_EQ(to_hex(reencoded.data.data(), reencoded.len),
                     to_hex(bytes.data(), bytes.size()));
        }
    }
}

SHENSI_TEST_CASE(sdo_rejects_frames_it_should_not_accept) {
    const std::vector<std::uint8_t> read = from_hex("40 64 60 00 00 00 00 00");
    SdoResponse response;

    // Right payload, wrong identifier range.
    CHECK(!decode_sdo(frame_from_bytes(Bus::Can0, 0x501, read.data(), kSdoDlc), response));
    // Right identifier, wrong length.
    CHECK(!decode_sdo(frame_from_bytes(Bus::Can0, 0x581, read.data(), 7), response));
    // An unknown command byte.
    const std::vector<std::uint8_t> bogus = from_hex("99 64 60 00 00 00 00 00");
    CHECK(!decode_sdo(frame_from_bytes(Bus::Can0, 0x581, bogus.data(), kSdoDlc), response));
    // A write command byte on a response identifier.
    const std::vector<std::uint8_t> write = from_hex("2B 40 60 00 06 00 00 00");
    CHECK(!decode_sdo(frame_from_bytes(Bus::Can0, 0x581, write.data(), kSdoDlc), response));
}

SHENSI_TEST_CASE(sdo_value_widths_and_sign_extension) {
    CHECK_EQ(sdo_value_width(0x2F), 1);
    CHECK_EQ(sdo_value_width(0x2B), 2);
    CHECK_EQ(sdo_value_width(0x27), 3);
    CHECK_EQ(sdo_value_width(0x23), 4);
    CHECK_EQ(sdo_value_width(0x60), 0);
    CHECK_EQ(sdo_value_width(0x40), 0);
    CHECK_EQ(sdo_value_width(0x4F), 1);
    CHECK_EQ(sdo_value_width(0x4B), 2);
    CHECK_EQ(sdo_value_width(0x47), 3);
    CHECK_EQ(sdo_value_width(0x43), 4);
    CHECK_EQ(sdo_value_width(0x11), -1);

    CHECK(is_sdo_write(0x23));
    CHECK(!is_sdo_write(0x43));
    CHECK(is_sdo_read_response(0x43));
    CHECK(!is_sdo_read_response(0x23));

    // §4.7's negative limit is FFFF80C8, which is -32568 as a signed 32-bit value.
    SdoResponse negative;
    negative.value = 0xFFFF80C8u;
    negative.value_width = 4;
    CHECK_EQ(negative.value_signed(), -32568);

    SdoResponse small_negative;
    small_negative.value = 0xFFC8u;
    small_negative.value_width = 2;
    CHECK_EQ(small_negative.value_signed(), -56);

    SdoResponse positive;
    positive.value = 0x0145u;
    positive.value_width = 2;
    CHECK_EQ(positive.value_signed(), 325);
}

SHENSI_TEST_CASE(control_subframes_match_the_documented_bytes) {
    for (const SubframeVector& vector : kSubframeVectors) {
        const std::vector<std::uint8_t> bytes = from_hex(vector.bytes);
        CHECK_EQ(bytes.size(), static_cast<std::size_t>(kSingleAxisDlc));

        const ControlSubframe decoded = decode_subframe_or_fail(bytes);
        CHECK_MSG(static_cast<int>(decoded.mode) == static_cast<int>(vector.mode),
                  std::string("vector \"") + vector.bytes + "\" mode");
        CHECK_MSG(decoded.target1 == vector.target1,
                  std::string("vector \"") + vector.bytes + "\" target1");
        CHECK_MSG(decoded.target2 == vector.target2,
                  std::string("vector \"") + vector.bytes + "\" target2");
        CHECK_MSG(decoded.feedforward == vector.feedforward,
                  std::string("vector \"") + vector.bytes + "\" feedforward");
        // Every example in §5.2 has the device enabled with the brake released.
        CHECK(decoded.enable);
        CHECK(decoded.brake_release);
        CHECK(!decoded.clear_error);

        const auto reencoded = encode_subframe(decoded);
        CHECK_EQ(to_hex(reencoded.data(), reencoded.size()), to_hex(bytes.data(), bytes.size()));
    }
}

SHENSI_TEST_CASE(single_axis_frames_use_the_0x100_identifier) {
    const std::vector<std::uint8_t> bytes = from_hex("C2 00 00 07 D0 00 05");
    const Frame frame = frame_from_bytes(Bus::Can1, 0x101, bytes.data(), kSingleAxisDlc);
    CHECK_EQ(static_cast<int>(classify(frame)), static_cast<int>(FrameClass::SingleAxisCommand));
    CHECK_EQ(static_cast<int>(dev_id_of(frame)), 1);

    ControlSubframe sub;
    CHECK(decode_single_axis(frame, sub));
    CHECK_EQ(static_cast<int>(sub.mode), 1);
    CHECK_EQ(sub.target2, 2000);

    const Frame rebuilt = encode_single_axis(Bus::Can1, 0x01, sub);
    CHECK_EQ(rebuilt.id, static_cast<std::uint32_t>(0x101));
    CHECK_EQ(to_hex(rebuilt.data.data(), rebuilt.len), to_hex(bytes.data(), bytes.size()));
}

SHENSI_TEST_CASE(the_identifier_overlap_is_resolved_by_length) {
    // 0x111 is ambiguous on identifier alone: it is 0x100|0x11 and 0x110|0x01 at the same time.
    const std::vector<std::uint8_t> seven = from_hex("C2 00 00 07 D0 00 05");
    const Frame as_single = frame_from_bytes(Bus::Can0, 0x111, seven.data(), kSingleAxisDlc);
    CHECK_EQ(static_cast<int>(classify(as_single)), static_cast<int>(FrameClass::SingleAxisCommand));
    CHECK_EQ(static_cast<int>(dev_id_of(as_single)), 0x11);

    std::vector<std::uint8_t> nine = from_hex("C2 00 00 07 D0 00 05 00 00");
    const Frame as_mit = frame_from_bytes(Bus::Can0, 0x111, nine.data(), kMitSingleDlc);
    CHECK_EQ(static_cast<int>(classify(as_mit)), static_cast<int>(FrameClass::MitSingleCommand));
    CHECK_EQ(static_cast<int>(dev_id_of(as_mit)), 0x01);

    // Neither length: not a frame this protocol defines.
    const Frame neither = frame_from_bytes(Bus::Can0, 0x111, seven.data(), 8);
    CHECK_EQ(static_cast<int>(classify(neither)), static_cast<int>(FrameClass::Unknown));
}

SHENSI_TEST_CASE(feedback_frames_decode_to_the_documented_values) {
    for (const FeedbackVector& vector : kFeedbackVectors) {
        const std::vector<std::uint8_t> bytes = from_hex(vector.bytes);
        CHECK_EQ(bytes.size(), static_cast<std::size_t>(kFeedbackDlc));

        const Frame frame =
            frame_from_bytes(Bus::Can0, kFeedbackBase | 0x01, bytes.data(), kFeedbackDlc);
        CHECK_EQ(static_cast<int>(classify(frame)), static_cast<int>(FrameClass::Feedback));

        JointFeedback feedback;
        CHECK(decode_feedback(frame, feedback));
        CHECK_MSG(feedback.pos_cnt == vector.pos_cnt,
                  std::string("vector \"") + vector.bytes + "\" pos_cnt");
        CHECK_MSG(feedback.vel_rpm == vector.vel_rpm,
                  std::string("vector \"") + vector.bytes + "\" vel_rpm");
        CHECK_MSG(feedback.current_ma == vector.current_ma,
                  std::string("vector \"") + vector.bytes + "\" current_ma");
        CHECK_MSG(feedback.fault == vector.fault,
                  std::string("vector \"") + vector.bytes + "\" fault");
        CHECK_MSG(feedback.temp_dc == vector.temp_dc,
                  std::string("vector \"") + vector.bytes + "\" temp_dc");
        CHECK_MSG(feedback.mode == vector.mode,
                  std::string("vector \"") + vector.bytes + "\" mode");
        CHECK_MSG(feedback.enabled == vector.enabled, std::string("vector \"") + vector.bytes + "\" enabled");
        CHECK_MSG(feedback.brake_released == vector.brake_released,
                  std::string("vector \"") + vector.bytes + "\" brake_released");
        CHECK_MSG(feedback.error == vector.error, std::string("vector \"") + vector.bytes + "\" error");
        CHECK_MSG(feedback.in_position == vector.in_position,
                  std::string("vector \"") + vector.bytes + "\" in_position");
    }
}

SHENSI_TEST_CASE(feedback_units_match_the_documented_conversions) {
    JointFeedback feedback;
    feedback.pos_cnt = 16382;  // §4.5 / §4.12: "about 90 deg"
    feedback.temp_dc = 280;    // §4.12: "28.0 degC"
    CHECK_MSG(std::abs(feedback.position_rad() * 180.0 / kPi - 90.0) < 0.02,
              "16382 counts should be about 90 deg");
    CHECK_EQ(feedback.temperature_c(), 28.0);
}

SHENSI_TEST_CASE(the_seven_axis_broadcast_matches_the_documented_bytes) {
    const std::vector<std::uint8_t> bytes = from_hex(kSevenAxisBroadcast);
    CHECK_EQ(bytes.size(), static_cast<std::size_t>(kMultiAxisDlc));

    const Frame frame = frame_from_bytes(Bus::Can0, kMultiAxisId, bytes.data(), kMultiAxisDlc);
    CHECK_EQ(static_cast<int>(classify(frame)), static_cast<int>(FrameClass::MultiAxisCommand));

    MultiAxisCommand command;
    CHECK(decode_multi_axis(frame, command));
    for (std::size_t slot = 0; slot < 7; ++slot) {
        CHECK_EQ(static_cast<int>(command.dev_ids[slot]), static_cast<int>(slot + 1));
    }
    CHECK_EQ(static_cast<int>(command.dev_ids[7]), 0);

    CHECK_EQ(command.subframes[0].target1, 5861);  // 0x16E5
    CHECK_EQ(command.subframes[6].target1, 2248);  // 0x08C8
    // The unused eighth subframe is zero-filled, so its mode and target are zero too.
    CHECK_EQ(static_cast<int>(command.subframes[7].mode), 0);
    CHECK_EQ(command.subframes[7].target1, 0);
    CHECK(!command.subframes[7].enable);

    const Frame reencoded = encode_multi_axis(command);
    CHECK_EQ(to_hex(reencoded.data.data(), reencoded.len), to_hex(bytes.data(), bytes.size()));
}

SHENSI_TEST_CASE(nmt_and_sync_frames) {
    // §7: 81 01 then 01 00.
    const Frame reset = encode_nmt(NmtCommand::ResetNode, 0x01);
    CHECK_EQ(reset.id, static_cast<std::uint32_t>(0x000));
    CHECK_EQ(static_cast<int>(reset.len), static_cast<int>(kNmtDlc));
    CHECK_EQ(to_hex(reset.data.data(), reset.len), std::string("8101"));

    const Frame start = encode_nmt(NmtCommand::Start, 0x00);
    CHECK_EQ(to_hex(start.data.data(), start.len), std::string("0100"));

    const Frame sync = encode_sync();
    CHECK_EQ(sync.id, static_cast<std::uint32_t>(0x080));
    CHECK_EQ(static_cast<int>(sync.len), 0);
    CHECK_EQ(static_cast<int>(classify(sync)), static_cast<int>(FrameClass::Sync));
}

SHENSI_TEST_CASE(the_two_byte_orders_stay_apart) {
    // SDO: the index 0x6040 goes on the wire low byte first.
    const Frame sdo = encode_sdo(sdo_write2(0x01, 0x6040, 0, 0x0006));
    CHECK_EQ(to_hex(&sdo.data[1], 2), std::string("4060"));

    // Custom PDO: the target 16384 goes on the wire high byte first.
    ControlSubframe sub;
    sub.enable = true;
    sub.brake_release = true;
    sub.mode = 1;
    sub.target1 = 16384;
    const Frame pdo = encode_single_axis(Bus::Can0, 0x01, sub);
    CHECK_EQ(to_hex(&pdo.data[1], 2), std::string("4000"));

    // And the feedback frame reads big-endian too: 0x3FFE is "3F FE", not "FE 3F".
    const std::vector<std::uint8_t> feedback_bytes = from_hex("3F FE 00 00 00 00 00 00 00 00 00 00");
    JointFeedback feedback;
    CHECK(decode_feedback(frame_from_bytes(Bus::Can0, 0x301, feedback_bytes.data(), kFeedbackDlc),
                          feedback));
    CHECK_EQ(static_cast<int>(feedback.pos_cnt), 16382);
}

SHENSI_TEST_CASE(rad_to_cnt_rounds_to_nearest) {
    // Truncation would give 1234 here, and std::floor would give -1235 for the negative.
    const auto rad_of = [](double counts) { return counts * 2.0 * kPi / kCntPerRev; };
    CHECK_EQ(static_cast<int>(rad_to_cnt(rad_of(1234.9))), 1235);
    CHECK_EQ(static_cast<int>(rad_to_cnt(rad_of(-1234.9))), -1235);
    CHECK_EQ(static_cast<int>(rad_to_cnt(rad_of(1234.4))), 1234);
    CHECK_EQ(static_cast<int>(rad_to_cnt(rad_of(-1234.4))), -1234);

    // cnt -> rad -> cnt is the identity over the whole range.
    bool round_trips = true;
    for (int cnt = -32768; cnt <= 32767; ++cnt) {
        const auto value = static_cast<std::int16_t>(cnt);
        if (rad_to_cnt(cnt_to_rad(value)) != value) round_trips = false;
    }
    CHECK_MSG(round_trips, "rad_to_cnt(cnt_to_rad(c)) == c for every 16-bit count");

    CHECK_EQ(static_cast<int>(rad_to_cnt(0.0)), 0);
    // The extreme counts clamp instead of wrapping.
    CHECK_EQ(static_cast<int>(rad_to_cnt(kPi)), 32767);
    CHECK_EQ(static_cast<int>(rad_to_cnt(-kPi)), -32768);
    CHECK_EQ(static_cast<int>(rad_to_cnt(100.0)), 32767);
    CHECK_EQ(static_cast<int>(rad_to_cnt(-100.0)), -32768);
}

SHENSI_TEST_CASE(cnt_to_rad_matches_the_documented_scale) {
    CHECK_MSG(std::abs(cnt_to_rad(32767) - kPi) < 1e-4, "32767 counts should be about pi");
    CHECK_MSG(std::abs(cnt_to_rad(16384) - kPi / 2.0) < 1e-6, "16384 counts should be pi/2");
    CHECK_EQ(cnt_to_rad(0), 0.0);
    // One count is 360/65536 degrees.
    CHECK_MSG(std::abs(cnt_to_rad(1) * 180.0 / kPi - 360.0 / 65536.0) < 1e-9,
              "one count should be 360/65536 deg");
}

SHENSI_TEST_CASE(frame_classification_and_device_ids) {
    const std::vector<std::uint8_t> payload(64, 0x00);
    struct Expectation {
        std::uint32_t id;
        std::uint8_t len;
        FrameClass kind;
        std::uint8_t dev_id;
    };
    const Expectation expectations[] = {
        {0x000, 2, FrameClass::Nmt, 0},
        {0x080, 0, FrameClass::Sync, 0},
        {0x101, 7, FrameClass::SingleAxisCommand, 1},
        {0x111, 9, FrameClass::MitSingleCommand, 1},
        {0x200, 64, FrameClass::MultiAxisCommand, 0},
        {0x210, 64, FrameClass::MitMultiAxisCommand, 0},
        {0x301, 12, FrameClass::Feedback, 1},
        {0x581, 8, FrameClass::SdoResponse, 1},
        {0x601, 8, FrameClass::SdoRequest, 1},
        {0x701, 1, FrameClass::Heartbeat, 1},
        {0x77F, 1, FrameClass::Heartbeat, 0x7F},
        // §5.2 prints a "simple feedback" frame on 0x081 with DLC 8. Nothing in the document
        // defines that identifier, so it must come out as unknown rather than be guessed at.
        {0x081, 8, FrameClass::Unknown, 0},
        {0x201, 64, FrameClass::Unknown, 0},
    };

    for (const Expectation& expectation : expectations) {
        const Frame frame =
            frame_from_bytes(Bus::Can0, expectation.id, payload.data(), expectation.len);
        CHECK_MSG(classify(frame) == expectation.kind,
                  std::string("id ") + std::to_string(expectation.id) + " len " +
                      std::to_string(expectation.len) + " classified as " +
                      to_string(classify(frame)) + ", expected " +
                      to_string(expectation.kind));
        CHECK_EQ(static_cast<int>(dev_id_of(frame)), static_cast<int>(expectation.dev_id));
    }
}

SHENSI_TEST_CASE(decode_entry_points_reject_the_wrong_frame_class) {
    const std::vector<std::uint8_t> payload(64, 0x00);
    const Frame feedback = frame_from_bytes(Bus::Can0, 0x301, payload.data(), kFeedbackDlc);
    const Frame single = frame_from_bytes(Bus::Can0, 0x101, payload.data(), kSingleAxisDlc);
    const Frame multi = frame_from_bytes(Bus::Can0, 0x200, payload.data(), kMultiAxisDlc);

    ControlSubframe sub;
    JointFeedback telemetry;
    MultiAxisCommand command;
    CHECK(!decode_single_axis(feedback, sub));
    CHECK(!decode_single_axis(multi, sub));
    CHECK(!decode_feedback(single, telemetry));
    CHECK(!decode_feedback(multi, telemetry));
    CHECK(!decode_multi_axis(single, command));
    CHECK(!decode_multi_axis(feedback, command));
    CHECK(decode_multi_axis(multi, command));
}

SHENSI_TEST_CASE(decode_subframe_refuses_a_short_buffer) {
    // The defect this pins: a pointer plus an assumed size is what overflowed the vendor's own
    // getFKpose. Every decoder here takes a length instead.
    ControlSubframe sub;
    const std::vector<std::uint8_t> full = from_hex("C2 00 00 07 D0 00 05");
    for (std::size_t len = 0; len < kSingleAxisDlc; ++len) {
        CHECK_MSG(!decode_subframe(full.data(), len, sub),
                  "a " + std::to_string(len) + "-byte buffer must be rejected");
    }
    CHECK(decode_subframe(full.data(), full.size(), sub));
    CHECK(!decode_subframe(nullptr, kSingleAxisDlc, sub));
}

SHENSI_TEST_CASE(pr0002_documented_anomalies) {
    // 1. §5.3's seven-axis example carries control byte D0 in every subframe. Under §5.1's bit
    //    layout that is mode 8, and the mode table in §3.4/§5.2 only defines 1..7. The example's
    //    acceleration and velocity fields are also zero, which no profile-position command
    //    would be. Pinned here so it is not quietly "fixed" by whoever writes L1.
    const std::vector<std::uint8_t> first_subframe = from_hex("D0 16 E5 00 00 00 00");
    ControlSubframe sub;
    CHECK(decode_subframe(first_subframe.data(), first_subframe.size(), sub));
    CHECK_EQ(static_cast<int>(sub.mode), 8);
    CHECK(static_cast<int>(sub.mode) < 1 || static_cast<int>(sub.mode) > 7);

    // 2. §4.7 sets the positive limit on sub-index 02 but reads it back from 01, and sets the
    //    negative limit on 01 while reading it from 02. CiA 402 defines 607Dh:01 = min and
    //    :02 = max, so the two "read" rows look swapped.
    const std::vector<std::uint8_t> set_positive = from_hex("23 7D 60 02 30 7F 00 00");
    const std::vector<std::uint8_t> read_positive = from_hex("40 7D 60 01 00 00 00 00");
    const std::vector<std::uint8_t> set_negative = from_hex("23 7D 60 01 C8 80 FF FF");
    const std::vector<std::uint8_t> read_negative = from_hex("40 7D 60 02 00 00 00 00");
    CHECK_EQ(static_cast<int>(set_positive[3]), 2);
    CHECK_EQ(static_cast<int>(read_positive[3]), 1);
    CHECK_EQ(static_cast<int>(set_negative[3]), 1);
    CHECK_EQ(static_cast<int>(read_negative[3]), 2);

    // 3. §7's response to "clear rxPDO1" (2F 00 16 00) comes back with sub-index 01.
    const std::vector<std::uint8_t> clear_request = from_hex("2F 00 16 00 00 00 00 00");
    const std::vector<std::uint8_t> clear_response = from_hex("60 00 16 01 00 00 00 00");
    CHECK_EQ(static_cast<int>(clear_request[3]), 0);
    CHECK_EQ(static_cast<int>(clear_response[3]), 1);
}

SHENSI_TEST_CASE(the_vendor_dlc8_sync_classifies_as_sync) {
    // hardware-facts.md 2.7: the vendor stack sends 0x080 with eight zero bytes.
    const std::uint8_t zeros[8] = {};
    const Frame vendor = frame_from_bytes(Bus::Can0, kSyncId, zeros, kSyncVendorDlc);
    CHECK_EQ(static_cast<int>(classify(vendor)), static_cast<int>(FrameClass::Sync));
    // Any other length on 0x080 is not a sync.
    const Frame odd = frame_from_bytes(Bus::Can0, kSyncId, zeros, 4);
    CHECK_EQ(static_cast<int>(classify(odd)), static_cast<int>(FrameClass::Unknown));
}

SHENSI_TEST_CASE(rad_to_cnt_rejects_non_finite_angles) {
    CHECK_THROWS(rad_to_cnt(std::nan("")));
    CHECK_THROWS(rad_to_cnt(INFINITY));
    CHECK_THROWS(rad_to_cnt(-INFINITY));
}

SHENSI_TEST_CASE(encoders_reject_a_dev_id_past_the_11_bit_range) {
    CHECK_THROWS(encode_sdo(sdo_read(kMaxDevId + 1, 0x6064, 0)));
    CHECK_THROWS(encode_single_axis(Bus::Can0, kMaxDevId + 1, ControlSubframe{}));
    // The top valid id still encodes.
    CHECK_EQ(encode_sdo(sdo_read(kMaxDevId, 0x6064, 0)).id, static_cast<std::uint32_t>(0x67F));
}
