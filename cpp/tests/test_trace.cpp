// Tests for the trace format, the differential comparison, and the recording/replay transports.
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <string>
#include <vector>

#include "check.hpp"
#include "shensi/can/fake_transport.hpp"
#include "shensi/can/trace.hpp"
#include "shensi/can/wire.hpp"

using namespace shensi::can;
using shensi::can::from_hex;
using shensi::can::to_hex;

namespace {

Frame frame(Bus bus, std::uint32_t id, const std::string& data_hex) {
    const std::vector<std::uint8_t> bytes = from_hex(data_hex);
    return frame_from_bytes(bus, id, bytes.data(), static_cast<std::uint8_t>(bytes.size()));
}

// The three SDO writes a §4.8 enable sequence opens with, and the feedback frame from §6.
const char* const kWrite6 = "2b40600006000000";
const char* const kWrite7 = "2b40600007000000";
const char* const kWriteF = "2b4060000f000000";
const char* const kFeedback = "09e1fde5ff51000000f001c0";

// A base trace: three commands on bus 0, then one feedback frame. What most diff tests vary.
Trace sample_trace() {
    Trace trace;
    trace.add(frame(Bus::Can0, 0x601, kWrite6), 1000, Direction::Tx);
    trace.add(frame(Bus::Can0, 0x601, kWrite7), 2000, Direction::Tx);
    trace.add(frame(Bus::Can0, 0x601, kWriteF), 3000, Direction::Tx);
    trace.add(frame(Bus::Can0, 0x301, kFeedback), 4000, Direction::Rx);
    return trace;
}

void expect_trace_error(const char* file, int line, const std::string& text,
                        const std::string& needle) {
    ++shensi::test::checks();
    try {
        Trace::from_text(text);
    } catch (const std::exception& error) {
        const std::string message = error.what();
        if (message.find(needle) == std::string::npos) {
            shensi::test::report_failure(file, line, "message \"" + message +
                                                         "\" does not contain \"" + needle + "\"");
        }
        return;
    }
    shensi::test::report_failure(file, line, "expected a parse error containing \"" + needle + "\"");
}

}  // namespace

#define EXPECT_TRACE_ERROR(text, needle) \
    expect_trace_error(__FILE__, __LINE__, (text), (needle))

SHENSI_TEST_CASE(trace_text_round_trips) {
    Trace original;
    original.add(frame(Bus::Can0, 0x601, kWrite6), 1000, Direction::Tx, "enable");
    original.add(frame(Bus::Can0, 0x601, kWrite7), 2000, Direction::Tx);
    original.add(frame(Bus::Can0, 0x601, kWriteF), 3000, Direction::Tx);
    original.add(frame(Bus::Can0, 0x301, kFeedback), 4000, Direction::Rx, "feedback");
    original.add(frame(Bus::Can1, 0x080, ""), 5000, Direction::Tx, "sync");  // zero-length frame

    const std::string text = original.to_text();
    const Trace reloaded = Trace::from_text(text);

    CHECK_EQ(reloaded.size(), original.size());
    for (std::size_t i = 0; i < original.size(); ++i) {
        const TraceEntry& a = original.entries()[i];
        const TraceEntry& b = reloaded.entries()[i];
        CHECK_MSG(a.t_ns == b.t_ns, "t_ns at entry " + std::to_string(i));
        CHECK_MSG(a.direction == b.direction, "direction at entry " + std::to_string(i));
        CHECK_MSG(a.frame.bus == b.frame.bus, "bus at entry " + std::to_string(i));
        CHECK_MSG(a.frame.id == b.frame.id, "id at entry " + std::to_string(i));
        CHECK_MSG(a.frame.len == b.frame.len, "len at entry " + std::to_string(i));
        CHECK_MSG(a.frame.brs == b.frame.brs, "brs at entry " + std::to_string(i));
        CHECK_MSG(a.frame.fdf == b.frame.fdf, "fdf at entry " + std::to_string(i));
        CHECK_MSG(to_hex(a.frame) == to_hex(b.frame), "data at entry " + std::to_string(i));
        CHECK_MSG(a.tag == b.tag, "tag at entry " + std::to_string(i));
    }
    // The text is stable: a second round trip is byte-identical.
    CHECK_EQ(reloaded.to_text(), text);
}

SHENSI_TEST_CASE(trace_text_records_the_bus_semantics) {
    // The bus column is only useful if a reader knows which arm each bus is. The header says so,
    // because getting it backwards is a left/right swap.
    const std::string text = sample_trace().to_text();
    CHECK_MSG(text.find("bus 0 = controller CAN1 (left arm)") != std::string::npos,
              "the header must name the bus semantics");
    CHECK_MSG(text.find("# columns:") != std::string::npos, "the header must name the columns");
    CHECK_MSG(text.find("\n1000 tx 0 601 8 1 1 2b40600006000000 -") != std::string::npos,
              "the first entry line is not as expected:\n" + text);
}

SHENSI_TEST_CASE(trace_ignores_comments_and_blank_lines) {
    const Trace trace = Trace::from_text(
        "# a comment\n"
        "\n"
        "   \n"
        "# another\n"
        "5 tx 0 601 8 1 1 2b40600006000000 -\n");
    CHECK_EQ(trace.size(), static_cast<std::size_t>(1));
    CHECK_EQ(trace.entries()[0].t_ns, static_cast<std::uint64_t>(5));
}

SHENSI_TEST_CASE(trace_rejects_malformed_lines) {
    EXPECT_TRACE_ERROR("1 tx 0 601 8 1 1 2b40600006000000\n", "expected 9 fields, got 8");
    EXPECT_TRACE_ERROR("1 xx 0 601 8 1 1 2b40600006000000 -\n", "direction must be tx or rx");
    EXPECT_TRACE_ERROR("1 tx 2 601 8 1 1 2b40600006000000 -\n", "bus must be 0 or 1");
    EXPECT_TRACE_ERROR("1 tx 0 801 8 1 1 2b40600006000000 -\n", "11 bits");
    EXPECT_TRACE_ERROR("1 tx 0 601 65 1 1 2b40600006000000 -\n", "len must be 0..64");
    EXPECT_TRACE_ERROR("1 tx 0 601 8 2 1 2b40600006000000 -\n", "brs must be 0 or 1");
    EXPECT_TRACE_ERROR("1 tx 0 601 8 1 1 2b406000060000 -\n", "holds 7 bytes but len is 8");
    EXPECT_TRACE_ERROR("1 tx 0 601 8 1 1 - -\n", "data_hex is '-' but len is not 0");
    EXPECT_TRACE_ERROR("1 tx 0 601 8 1 1 zz40600006000000 -\n", "not a hex digit");
    EXPECT_TRACE_ERROR("1 tx 0 601 8 1 1 2b4060000600000 -\n", "odd number of hex digits");
    EXPECT_TRACE_ERROR("1 tx 0 nothex 8 1 1 2b40600006000000 -\n",
                       "identifier or length is not a number");

    // The message names the offending line, so a long trace is diagnosable.
    EXPECT_TRACE_ERROR(
        "# header\n1 tx 0 601 8 1 1 2b40600006000000 -\n1 xx 0 601 8 1 1 2b40600006000000 -\n",
        "line 3");
}

SHENSI_TEST_CASE(trace_round_trips_through_a_file) {
    const Trace original = sample_trace();
    const std::string path = "trace-round-trip.tmp";
    original.save(path);
    const Trace reloaded = Trace::load(path);
    CHECK_EQ(reloaded.size(), original.size());
    CHECK_EQ(reloaded.to_text(), original.to_text());
    std::remove(path.c_str());
}

SHENSI_TEST_CASE(diff_identical_traces_are_ok) {
    const DiffResult result = diff(sample_trace(), sample_trace());
    CHECK(result.ok());
    CHECK_EQ(result.items.size(), static_cast<std::size_t>(0));
    CHECK_MSG(result.summary().rfind("ok:", 0) == 0, result.summary());
}

SHENSI_TEST_CASE(diff_reports_a_missing_command) {
    Trace actual;
    actual.add(frame(Bus::Can0, 0x601, kWrite6), 1000, Direction::Tx);
    actual.add(frame(Bus::Can0, 0x601, kWrite7), 2000, Direction::Tx);

    const DiffResult result = diff(sample_trace(), actual);
    CHECK(!result.ok());
    CHECK_EQ(result.count(DiffKind::MissingCommand), static_cast<std::size_t>(1));
    CHECK_EQ(result.count(DiffKind::ExtraCommand), static_cast<std::size_t>(0));
    CHECK_EQ(result.items[0].kind, DiffKind::MissingCommand);
    CHECK_MSG(result.items[0].detail.find(kWriteF) != std::string::npos, result.items[0].detail);
}

SHENSI_TEST_CASE(diff_reports_an_extra_command) {
    Trace actual = sample_trace();
    actual.add(frame(Bus::Can0, 0x601, "2b40600080000000"), 5000, Direction::Tx);

    const DiffResult result = diff(sample_trace(), actual);
    CHECK(!result.ok());
    CHECK_EQ(result.count(DiffKind::ExtraCommand), static_cast<std::size_t>(1));
    CHECK_EQ(result.count(DiffKind::MissingCommand), static_cast<std::size_t>(0));
}

SHENSI_TEST_CASE(diff_reports_a_payload_mismatch) {
    // Same identifier, same length, one different byte: the golden 0F becomes 0E.
    Trace actual;
    actual.add(frame(Bus::Can0, 0x601, kWrite6), 1000, Direction::Tx);
    actual.add(frame(Bus::Can0, 0x601, kWrite7), 2000, Direction::Tx);
    actual.add(frame(Bus::Can0, 0x601, "2b4060000e000000"), 3000, Direction::Tx);
    actual.add(frame(Bus::Can0, 0x301, kFeedback), 4000, Direction::Rx);

    const DiffResult result = diff(sample_trace(), actual);
    CHECK(!result.ok());
    CHECK_EQ(result.count(DiffKind::PayloadMismatch), static_cast<std::size_t>(1));
    CHECK_EQ(result.count(DiffKind::MissingCommand), static_cast<std::size_t>(0));
    CHECK_EQ(result.count(DiffKind::ExtraCommand), static_cast<std::size_t>(0));
}

SHENSI_TEST_CASE(diff_ignores_feedback_unless_asked) {
    Trace actual;
    actual.add(frame(Bus::Can0, 0x601, kWrite6), 1000, Direction::Tx);
    actual.add(frame(Bus::Can0, 0x601, kWrite7), 2000, Direction::Tx);
    actual.add(frame(Bus::Can0, 0x601, kWriteF), 3000, Direction::Tx);
    // Feedback payloads carry position, current and temperature: never reproducible.
    actual.add(frame(Bus::Can0, 0x301, "12e1fde5ff51000000f001c0"), 4000, Direction::Rx);

    const DiffResult quiet = diff(sample_trace(), actual);
    CHECK(quiet.ok());
    CHECK_EQ(quiet.items.size(), static_cast<std::size_t>(0));

    NormalizeOptions options;
    options.include_rx = true;
    const DiffResult loud = diff(sample_trace(), actual, options);
    CHECK_EQ(loud.count(DiffKind::RxMismatch), static_cast<std::size_t>(1));
    // Advisory: a differing feedback payload is not a defect, so this still counts as ok.
    CHECK(loud.ok());
    CHECK_MSG(loud.summary().find("1 advisory") != std::string::npos, loud.summary());
}

SHENSI_TEST_CASE(diff_reports_a_left_right_swap_as_a_bus_mismatch) {
    // The same command sent on the other bus. Per the controller manual CAN1 is the left arm and
    // CAN2 the right, so this is the most likely way to get this wrong -- and it must not read as
    // "one frame missing, one frame extra".
    Trace golden;
    golden.add(frame(Bus::Can0, 0x101, "c2000007d00005"), 0, Direction::Tx);
    Trace swapped;
    swapped.add(frame(Bus::Can1, 0x101, "c2000007d00005"), 0, Direction::Tx);

    const DiffResult result = diff(golden, swapped);
    CHECK(!result.ok());
    CHECK_EQ(result.count(DiffKind::BusMismatch), static_cast<std::size_t>(1));
    CHECK_EQ(result.count(DiffKind::MissingCommand), static_cast<std::size_t>(0));
    CHECK_EQ(result.count(DiffKind::ExtraCommand), static_cast<std::size_t>(0));
    CHECK_EQ(result.items[0].kind, DiffKind::BusMismatch);
    CHECK_MSG(result.items[0].detail.find("other bus") != std::string::npos,
              result.items[0].detail);

    // A genuinely different identifier is still missing/extra, not a bus mismatch.
    Trace other;
    other.add(frame(Bus::Can1, 0x102, "c2000007d00005"), 0, Direction::Tx);
    const DiffResult plain = diff(golden, other);
    CHECK_EQ(plain.count(DiffKind::BusMismatch), static_cast<std::size_t>(0));
    CHECK_EQ(plain.count(DiffKind::MissingCommand), static_cast<std::size_t>(1));
    CHECK_EQ(plain.count(DiffKind::ExtraCommand), static_cast<std::size_t>(1));
}

SHENSI_TEST_CASE(diff_keeps_repeated_stop_frames_by_default) {
    // The controller manual says Stop needs several clicks before a cyclic motion actually
    // stops, so a repeated stop frame can be real protocol behaviour rather than a retry.
    // Collapsing by default would hide that.
    Trace golden;
    for (int i = 0; i < 3; ++i) golden.add(frame(Bus::Can0, 0x601, kWriteF), i, Direction::Tx);
    Trace actual;
    actual.add(frame(Bus::Can0, 0x601, kWriteF), 0, Direction::Tx);

    const DiffResult strict = diff(golden, actual);
    CHECK(!strict.ok());
    CHECK_EQ(strict.count(DiffKind::MissingCommand), static_cast<std::size_t>(2));

    NormalizeOptions collapsing;
    collapsing.collapse_consecutive_duplicates = true;
    CHECK(diff(golden, actual, collapsing).ok());
}

SHENSI_TEST_CASE(diff_checks_timing_only_when_a_tolerance_is_given) {
    Trace golden;
    golden.add(frame(Bus::Can0, 0x601, kWrite6), 0, Direction::Tx);
    golden.add(frame(Bus::Can0, 0x601, kWrite7), 1000000, Direction::Tx);

    Trace actual;
    actual.add(frame(Bus::Can0, 0x601, kWrite6), 500000, Direction::Tx);   // absolute times differ
    actual.add(frame(Bus::Can0, 0x601, kWrite7), 6500000, Direction::Tx);  // gap is 6 ms not 1 ms

    const DiffResult ignored = diff(golden, actual);
    CHECK(ignored.ok());
    CHECK_EQ(ignored.count(DiffKind::TimingOutOfTolerance), static_cast<std::size_t>(0));

    NormalizeOptions tight;
    tight.timing_tolerance_ns = 100000;  // 0.1 ms
    const DiffResult checked = diff(golden, actual, tight);
    CHECK_EQ(checked.count(DiffKind::TimingOutOfTolerance), static_cast<std::size_t>(1));
    // Timing is advisory: it does not make the comparison fail.
    CHECK(checked.ok());

    NormalizeOptions generous;
    generous.timing_tolerance_ns = 10000000;  // 10 ms
    const DiffResult relaxed = diff(golden, actual, generous);
    CHECK(relaxed.ok());
    CHECK_EQ(relaxed.count(DiffKind::TimingOutOfTolerance), static_cast<std::size_t>(0));
}

SHENSI_TEST_CASE(diff_resynchronises_after_an_insertion) {
    // One extra frame early in the actual trace must not cascade into a tail of differences.
    Trace actual;
    actual.add(frame(Bus::Can0, 0x601, kWrite6), 1000, Direction::Tx);
    actual.add(frame(Bus::Can0, 0x601, kWrite7), 1500, Direction::Tx);
    actual.add(frame(Bus::Can0, 0x601, kWrite7), 2000, Direction::Tx);
    actual.add(frame(Bus::Can0, 0x601, kWriteF), 3000, Direction::Tx);
    actual.add(frame(Bus::Can0, 0x301, kFeedback), 4000, Direction::Rx);

    const DiffResult result = diff(sample_trace(), actual);
    CHECK(!result.ok());
    CHECK_EQ(result.count(DiffKind::ExtraCommand), static_cast<std::size_t>(1));
    CHECK_EQ(result.count(DiffKind::MissingCommand), static_cast<std::size_t>(0));
    CHECK_EQ(result.count(DiffKind::PayloadMismatch), static_cast<std::size_t>(0));
}

SHENSI_TEST_CASE(recording_transport_records_both_directions) {
    FakeTransport inner;
    RecordingTransport recorder(inner);

    std::uint64_t now = 100;
    recorder.set_clock([&now] { return now += 10; });

    recorder.send(encode_sdo(sdo_write2(0x01, 0x6040, 0, 0x0006)));
    inner.inject(frame(Bus::Can0, 0x301, kFeedback));
    recorder.send(encode_sdo(sdo_write2(0x01, 0x6040, 0, 0x0007)));

    CHECK_EQ(recorder.trace().size(), static_cast<std::size_t>(3));
    CHECK_EQ(recorder.trace().entries()[0].direction, Direction::Tx);
    CHECK_EQ(recorder.trace().entries()[1].direction, Direction::Rx);
    CHECK_EQ(recorder.trace().entries()[2].direction, Direction::Tx);
    CHECK_EQ(recorder.trace().entries()[0].t_ns, static_cast<std::uint64_t>(110));
    CHECK_EQ(recorder.trace().entries()[2].t_ns, static_cast<std::uint64_t>(130));
    CHECK_EQ(recorder.trace().entries()[1].frame.id, static_cast<std::uint32_t>(0x301));

    // Forwarding still happened: the inner transport saw both sends.
    CHECK_EQ(inner.sent_count(), static_cast<std::size_t>(2));
    CHECK(recorder.healthy());

    const Trace taken = recorder.take_trace();
    CHECK_EQ(taken.size(), static_cast<std::size_t>(3));
    CHECK_EQ(recorder.trace().size(), static_cast<std::size_t>(0));
}

SHENSI_TEST_CASE(recording_transport_captures_a_scripted_exchange_as_a_trace) {
    FakeTransport inner;
    const std::vector<std::uint8_t> answer = from_hex("43 64 60 00 FE 3F 00 00");
    inner.set_responder([&](const Frame& sent, FakeTransport& self) {
        SdoRequest request;
        if (!decode_sdo_request(sent, request)) return;
        self.inject(frame_from_bytes(Bus::Can0, kSdoResponseBase | request.dev_id, answer.data(),
                                     kSdoDlc));
    });

    RecordingTransport recorder(inner);
    std::vector<Frame> seen;
    recorder.set_receiver([&](const Frame& f) { seen.push_back(f); });
    recorder.send(encode_sdo(sdo_read(0x01, 0x6064, 0)));

    CHECK_EQ(seen.size(), static_cast<std::size_t>(1));
    CHECK_EQ(recorder.trace().size(), static_cast<std::size_t>(2));
    CHECK_EQ(recorder.trace().entries()[0].direction, Direction::Tx);
    CHECK_EQ(recorder.trace().entries()[1].direction, Direction::Rx);
    CHECK_EQ(recorder.trace().entries()[1].frame.id, static_cast<std::uint32_t>(0x581));

    // The captured exchange survives the text format, which is what makes it a usable golden
    // vector: it can be committed, diffed, and replayed later.
    const Trace captured = recorder.trace();
    CHECK_EQ(Trace::from_text(captured.to_text()).size(), captured.size());
}

SHENSI_TEST_CASE(replay_transport_drives_the_differential_loop) {
    const Trace golden = sample_trace();

    ReplayTransport bus(golden);
    std::vector<Frame> received;
    bus.set_receiver([&](const Frame& f) { received.push_back(f); });

    // The stack under test sends the enable sequence it believes in.
    bus.send(encode_sdo(sdo_write2(0x01, 0x6040, 0, 0x0006)));
    bus.send(encode_sdo(sdo_write2(0x01, 0x6040, 0, 0x0007)));
    bus.send(encode_sdo(sdo_write2(0x01, 0x6040, 0, 0x000F)));
    bus.replay_rx();

    CHECK_EQ(received.size(), static_cast<std::size_t>(1));
    CHECK_EQ(received[0].id, static_cast<std::uint32_t>(0x301));

    const DiffResult result = diff(golden, bus.sent_trace());
    CHECK_MSG(result.ok(), result.summary());
    CHECK_EQ(bus.sent_count(), static_cast<std::size_t>(3));

    // A wrong sequence is caught.
    bus.clear();
    bus.send(encode_sdo(sdo_write2(0x01, 0x6040, 0, 0x0006)));
    const DiffResult wrong = diff(golden, bus.sent_trace());
    CHECK(!wrong.ok());
    CHECK_EQ(wrong.count(DiffKind::MissingCommand), static_cast<std::size_t>(2));

    bus.set_healthy(false);
    CHECK(!bus.healthy());
    CHECK_THROWS(bus.send(encode_sdo(sdo_read(0x01, 0x6064, 0))));
}

SHENSI_TEST_CASE(diff_kind_labels_are_stable) {
    // The labels appear in reports, so they are part of the interface.
    CHECK_EQ(std::string(to_string(DiffKind::MissingCommand)), std::string("missing_command"));
    CHECK_EQ(std::string(to_string(DiffKind::ExtraCommand)), std::string("extra_command"));
    CHECK_EQ(std::string(to_string(DiffKind::PayloadMismatch)), std::string("payload_mismatch"));
    CHECK_EQ(std::string(to_string(DiffKind::BusMismatch)), std::string("bus_mismatch"));
    CHECK_EQ(std::string(to_string(Direction::Tx)), std::string("tx"));
    CHECK_EQ(std::string(to_string(Direction::Rx)), std::string("rx"));
    CHECK(is_defect(DiffKind::MissingCommand));
    CHECK(is_defect(DiffKind::BusMismatch));
    CHECK(!is_defect(DiffKind::RxMismatch));
    CHECK(!is_defect(DiffKind::TimingOutOfTolerance));
}
