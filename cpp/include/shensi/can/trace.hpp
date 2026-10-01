// L0 trace and differential comparison.
//
// The oracle problem: nothing in this repository has run on the robot, so "did I implement this
// correctly?" has no answer yet. The plan is to run the vendor stack, record the bus traffic per
// operation, freeze it as golden vectors, and compare a reimplementation against it. This is the
// comparison half of that. See docs/l0-interface.md §6.
//
// The comparison operator is NOT byte equality on a raw capture. It is: filter to the channel of
// interest, keep relative order rather than absolute time, align on (bus, identifier, length),
// and compare payload sequences. See diff() for what each outcome means.
#ifndef SHENSI_CAN_TRACE_HPP
#define SHENSI_CAN_TRACE_HPP

#include <cstddef>
#include <cstdint>
#include <functional>
#include <string>
#include <vector>

#include "shensi/can/frame.hpp"
#include "shensi/can/transport.hpp"

namespace shensi::can {

enum class Direction : std::uint8_t { Tx, Rx };
const char* to_string(Direction direction);

struct TraceEntry {
    std::uint64_t t_ns = 0;
    Direction direction = Direction::Tx;
    Frame frame;
    std::string tag;  // free-form label; must not contain whitespace
};

// A recorded exchange, stored as line-based text. Line-based and dependency-free on purpose:
// this repository has no JSON library and does not want one for a format only this harness reads.
//
// The bus column carries the semantics that matter most here. Per the controller manual,
// CAN1 drives the left arm and CAN2 the right; the vendor's own header names the same two buses
// CAN0 and CAN1 (zero-based). So bus 0 below is the left arm -- an inference to confirm on the
// robot, and exactly the kind of thing that silently becomes a left/right swap. That is why
// diff() reports a bus difference as its own outcome rather than as a missing frame.
class Trace {
public:
    const std::vector<TraceEntry>& entries() const { return entries_; }
    std::size_t size() const { return entries_.size(); }
    bool empty() const { return entries_.empty(); }
    void clear() { entries_.clear(); }

    void add(const TraceEntry& entry) { entries_.push_back(entry); }
    void add(const Frame& frame, std::uint64_t t_ns, Direction direction, std::string tag = {});

    std::string to_text() const;

    // Throws std::runtime_error naming the offending line number. A trace that does not parse
    // is a broken oracle, not something to guess at.
    static Trace from_text(const std::string& text);

    void save(const std::string& path) const;
    static Trace load(const std::string& path);

private:
    std::vector<TraceEntry> entries_;
};

// ---------------------------------------------------------------- comparison

struct NormalizeOptions {
    // Transmitted frames are what a reimplementation must reproduce.
    bool include_tx = true;

    // Received frames carry load-side position, current and temperature, so their payloads are
    // never byte-stable. Off by default: an rx payload difference is not a defect. Turn it on to
    // check that the same *shape* of feedback arrived, which is how the §8 questions about
    // byte[10] / byte[11] get answered.
    bool include_rx = false;

    // Consecutive byte-identical frames on the same bus are collapsed. Off by default, and that
    // matters: the vendor's controller manual says Stop needs several clicks before the cyclic
    // motion actually stops, so a repeated stop frame can be real protocol behaviour rather than
    // a retry. Collapsing by default would hide a genuine difference.
    bool collapse_consecutive_duplicates = false;

    // Compare the gaps between aligned frames. Negative disables it, which is the default: the
    // real cycle time has never been measured (research/vendor-analysis/can-protocol-comparison.md §8.4), so a
    // timing bound here would be invented.
    std::int64_t timing_tolerance_ns = -1;
};

enum class DiffKind {
    MissingCommand,  // in the golden trace, absent from the actual one -- a defect
    ExtraCommand,    // in the actual trace, absent from the golden one -- a defect
    PayloadMismatch,  // same bus, identifier and length, different bytes -- a defect
    BusMismatch,     // same identifier and length on the other bus -- a defect
    TimingOutOfTolerance,  // advisory
    RxMismatch,            // advisory: feedback payloads are not reproducible
};

const char* to_string(DiffKind kind);
bool is_defect(DiffKind kind);

struct DiffItem {
    DiffKind kind = DiffKind::MissingCommand;
    std::size_t golden_index = 0;  // kNoIndex when the item has no counterpart
    std::size_t actual_index = 0;
    std::string detail;

    static constexpr std::size_t kNoIndex = static_cast<std::size_t>(-1);
};

struct DiffResult {
    std::vector<DiffItem> items;

    bool ok() const;  // no defects; advisory items do not count
    std::size_t count(DiffKind kind) const;
    std::string summary() const;
};

// Aligns on (bus, identifier, length) within a bounded lookahead window, then compares payloads
// for the aligned pairs. The lookahead is what lets a small insertion or deletion resynchronise
// instead of cascading. Traces being compared are expected to be near-identical -- that is the
// premise of a differential test -- so the bounded window is sufficient in practice; two wildly
// divergent traces will produce a suboptimal but still correctly-classified alignment.
DiffResult diff(const Trace& golden, const Trace& actual, const NormalizeOptions& options = {});

// ---------------------------------------------------------------- transports

// Decorator: forwards to an inner transport and records both directions. The recording receiver
// is installed on the inner transport at construction, so received frames are captured whether
// or not anyone asked for them; set_receiver only adds a handler on top of the recording.
class RecordingTransport final : public Transport {
public:
    explicit RecordingTransport(Transport& inner);

    void send(const Frame& frame) override;
    void set_receiver(Receiver receiver) override;
    bool healthy() const override;

    const Trace& trace() const { return trace_; }
    Trace take_trace();
    void clear();

    // Injectable so a test does not depend on wall-clock values.
    void set_clock(std::function<std::uint64_t()> clock);

private:
    Transport& inner_;
    Trace trace_;
    Receiver user_receiver_;
    std::function<std::uint64_t()> clock_;
};

// Replays a golden trace. It injects that trace's received frames on demand and records what the
// stack under test sends, so the two can be diffed:
//
//     ReplayTransport bus(golden);
//     drive_your_implementation(bus);     // its sends land in bus.sent()
//     bus.replay_rx();                    // feed it the recorded feedback
//     DiffResult result = diff(golden, bus.sent_trace());
class ReplayTransport final : public Transport {
public:
    explicit ReplayTransport(Trace golden);

    // Records; a replay has nothing to forward to. Throws if the transport is unhealthy.
    void send(const Frame& frame) override;
    void set_receiver(Receiver receiver) override;
    bool healthy() const override;
    void set_healthy(bool healthy);

    // Deliver every rx entry of the golden trace, in order.
    void replay_rx();

    const std::vector<Frame>& sent() const { return sent_; }
    std::size_t sent_count() const { return sent_.size(); }
    Trace sent_trace() const;  // the recorded sends, as a Trace, for diff()
    void clear();

private:
    Trace golden_;
    std::vector<Frame> sent_;
    Receiver receiver_;
    bool healthy_ = true;
};

}  // namespace shensi::can

#endif  // SHENSI_CAN_TRACE_HPP
