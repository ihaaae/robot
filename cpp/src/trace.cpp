#include "shensi/can/trace.hpp"

#include <algorithm>
#include <chrono>
#include <fstream>
#include <sstream>
#include <stdexcept>

namespace shensi::can {
namespace {

constexpr std::size_t kLookahead = 64;
constexpr const char* kNoValue = "-";

std::uint64_t steady_now_ns() {
    return static_cast<std::uint64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::steady_clock::now().time_since_epoch())
            .count());
}

// All identifiers in this protocol are 11-bit, so three hex digits always suffice.
std::string hex_id(std::uint32_t id) {
    static const char* digits = "0123456789abcdef";
    std::string out(3, '0');
    out[0] = digits[(id >> 8) & 0xF];
    out[1] = digits[(id >> 4) & 0xF];
    out[2] = digits[id & 0xF];
    return out;
}

std::vector<std::string> split_whitespace(const std::string& line) {
    std::vector<std::string> fields;
    std::istringstream stream(line);
    std::string field;
    while (stream >> field) fields.push_back(field);
    return fields;
}

std::string describe_frame(const Frame& frame) {
    return "bus " + std::to_string(static_cast<int>(frame.bus)) + " id " + hex_id(frame.id) +
           " len " + std::to_string(frame.len) + " data " + to_hex(frame);
}

bool same_key(const TraceEntry& a, const TraceEntry& b) {
    return a.frame.bus == b.frame.bus && a.frame.id == b.frame.id && a.frame.len == b.frame.len;
}

bool same_payload(const Frame& a, const Frame& b) {
    if (a.len != b.len) return false;
    return std::equal(a.data.begin(), a.data.begin() + a.len, b.data.begin());
}

std::vector<TraceEntry> filtered(const Trace& trace, const NormalizeOptions& options) {
    std::vector<TraceEntry> out;
    for (const TraceEntry& entry : trace.entries()) {
        const bool wanted =
            entry.direction == Direction::Tx ? options.include_tx : options.include_rx;
        if (!wanted) continue;
        if (options.collapse_consecutive_duplicates && !out.empty()) {
            const TraceEntry& previous = out.back();
            if (same_key(previous, entry) && same_payload(previous.frame, entry.frame)) continue;
        }
        out.push_back(entry);
    }
    return out;
}

}  // namespace

const char* to_string(Direction direction) {
    return direction == Direction::Tx ? "tx" : "rx";
}

const char* to_string(DiffKind kind) {
    switch (kind) {
        case DiffKind::MissingCommand: return "missing_command";
        case DiffKind::ExtraCommand: return "extra_command";
        case DiffKind::PayloadMismatch: return "payload_mismatch";
        case DiffKind::BusMismatch: return "bus_mismatch";
        case DiffKind::TimingOutOfTolerance: return "timing_out_of_tolerance";
        case DiffKind::RxMismatch: return "rx_mismatch";
    }
    return "unknown";
}

bool is_defect(DiffKind kind) {
    switch (kind) {
        case DiffKind::MissingCommand:
        case DiffKind::ExtraCommand:
        case DiffKind::PayloadMismatch:
        case DiffKind::BusMismatch:
            return true;
        case DiffKind::TimingOutOfTolerance:
        case DiffKind::RxMismatch:
            return false;
    }
    return false;
}

// ------------------------------------------------------------------ Trace

void Trace::add(const Frame& frame, std::uint64_t t_ns, Direction direction, std::string tag) {
    entries_.push_back(TraceEntry{t_ns, direction, frame, std::move(tag)});
}

std::string Trace::to_text() const {
    std::string out;
    out += "# shensi can trace v1\n";
    out += "# bus 0 = controller CAN1 (left arm), bus 1 = controller CAN2 (right arm)\n";
    out += "# columns: t_ns direction bus id len brs fdf data_hex tag\n";
    for (const TraceEntry& entry : entries_) {
        out += std::to_string(entry.t_ns);
        out += ' ';
        out += to_string(entry.direction);
        out += ' ';
        out += std::to_string(static_cast<int>(entry.frame.bus));
        out += ' ';
        out += hex_id(entry.frame.id);
        out += ' ';
        out += std::to_string(entry.frame.len);
        out += ' ';
        out += entry.frame.brs ? '1' : '0';
        out += ' ';
        out += entry.frame.fdf ? '1' : '0';
        out += ' ';
        out += entry.frame.len == 0 ? std::string(kNoValue) : to_hex(entry.frame);
        out += ' ';
        out += entry.tag.empty() ? std::string(kNoValue) : entry.tag;
        out += '\n';
    }
    return out;
}

Trace Trace::from_text(const std::string& text) {
    Trace trace;
    std::istringstream stream(text);
    std::string line;
    std::size_t line_number = 0;
    while (std::getline(stream, line)) {
        ++line_number;
        if (line.empty() || line[0] == '#') continue;

        const std::vector<std::string> fields = split_whitespace(line);
        // A line of only spaces is blank, not malformed. Trailing whitespace and CRLF line
        // endings are both normal in a hand-edited trace.
        if (fields.empty()) continue;

        const auto bad = [&](const std::string& why) {
            throw std::runtime_error("trace line " + std::to_string(line_number) + ": " + why +
                                     " -- \"" + line + "\"");
        };
        if (fields.size() != 9) bad("expected 9 fields, got " + std::to_string(fields.size()));

        TraceEntry entry;
        try {
            entry.t_ns = std::stoull(fields[0]);
        } catch (const std::exception&) {
            bad("t_ns is not a number");
        }
        if (fields[1] == "tx") {
            entry.direction = Direction::Tx;
        } else if (fields[1] == "rx") {
            entry.direction = Direction::Rx;
        } else {
            bad("direction must be tx or rx");
        }
        try {
            const unsigned long bus = std::stoul(fields[2]);
            if (bus > 1) bad("bus must be 0 or 1");
            entry.frame.bus = static_cast<Bus>(bus);
            const unsigned long id = std::stoul(fields[3], nullptr, 16);
            if (id > 0x7FF) bad("identifier must fit in 11 bits");
            entry.frame.id = static_cast<std::uint32_t>(id);
            const unsigned long len = std::stoul(fields[4]);
            if (len > kMaxDlc) bad("len must be 0..64");
            entry.frame.len = static_cast<std::uint8_t>(len);
        } catch (const std::runtime_error&) {
            throw;
        } catch (const std::exception&) {
            bad("bus, identifier or length is not a number");
        }
        if (fields[5] != "0" && fields[5] != "1") bad("brs must be 0 or 1");
        if (fields[6] != "0" && fields[6] != "1") bad("fdf must be 0 or 1");
        entry.frame.brs = fields[5] == "1";
        entry.frame.fdf = fields[6] == "1";

        if (fields[7] != kNoValue) {
            std::vector<std::uint8_t> data;
            try {
                data = from_hex(fields[7]);
            } catch (const std::invalid_argument& error) {
                bad(error.what());
            }
            if (data.size() != entry.frame.len) {
                bad("data_hex holds " + std::to_string(data.size()) + " bytes but len is " +
                    std::to_string(entry.frame.len));
            }
            std::copy(data.begin(), data.end(), entry.frame.data.begin());
        } else if (entry.frame.len != 0) {
            bad("data_hex is '-' but len is not 0");
        }

        entry.tag = fields[8] == kNoValue ? std::string() : fields[8];
        trace.add(entry);
    }
    return trace;
}

void Trace::save(const std::string& path) const {
    std::ofstream out(path, std::ios::binary | std::ios::trunc);
    if (!out) throw std::runtime_error("cannot open trace for writing: " + path);
    out << to_text();
    if (!out) throw std::runtime_error("cannot write trace: " + path);
}

Trace Trace::load(const std::string& path) {
    std::ifstream in(path, std::ios::binary);
    if (!in) throw std::runtime_error("cannot open trace for reading: " + path);
    std::ostringstream buffer;
    buffer << in.rdbuf();
    return from_text(buffer.str());
}

// ------------------------------------------------------------------ diff

bool DiffResult::ok() const {
    for (const DiffItem& item : items) {
        if (is_defect(item.kind)) return false;
    }
    return true;
}

std::size_t DiffResult::count(DiffKind kind) const {
    std::size_t total = 0;
    for (const DiffItem& item : items) {
        if (item.kind == kind) ++total;
    }
    return total;
}

std::string DiffResult::summary() const {
    std::size_t advisory = 0;
    for (const DiffItem& item : items) {
        if (!is_defect(item.kind)) ++advisory;
    }
    std::string out = ok() ? "ok" : "DIFFERENT";
    out += ": " + std::to_string(count(DiffKind::MissingCommand)) + " missing, " +
           std::to_string(count(DiffKind::ExtraCommand)) + " extra, " +
           std::to_string(count(DiffKind::PayloadMismatch)) + " payload, " +
           std::to_string(count(DiffKind::BusMismatch)) + " bus, " +
           std::to_string(advisory) + " advisory";
    return out;
}

DiffResult diff(const Trace& golden, const Trace& actual, const NormalizeOptions& options) {
    const std::vector<TraceEntry> a = filtered(golden, options);
    const std::vector<TraceEntry> b = filtered(actual, options);

    DiffResult result;

    bool have_previous = false;
    std::uint64_t a_previous = 0;
    std::uint64_t b_previous = 0;

    const auto compare_aligned = [&](std::size_t ai, std::size_t bi) {
        const TraceEntry& ga = a[ai];
        const TraceEntry& ac = b[bi];

        if (!same_payload(ga.frame, ac.frame)) {
            // A differing feedback payload is not a defect: those bytes carry position, current
            // and temperature. A differing command payload is.
            const bool feedback = ga.direction == Direction::Rx;
            result.items.push_back(DiffItem{feedback ? DiffKind::RxMismatch
                                                     : DiffKind::PayloadMismatch,
                                            ai, bi,
                                            "golden " + describe_frame(ga.frame) + " vs actual " +
                                                describe_frame(ac.frame)});
        }

        if (options.timing_tolerance_ns >= 0 && have_previous) {
            const std::uint64_t golden_gap = ga.t_ns - a_previous;
            const std::uint64_t actual_gap = ac.t_ns - b_previous;
            const std::uint64_t delta = golden_gap > actual_gap ? golden_gap - actual_gap
                                                                : actual_gap - golden_gap;
            if (delta > static_cast<std::uint64_t>(options.timing_tolerance_ns)) {
                result.items.push_back(DiffItem{DiffKind::TimingOutOfTolerance, ai, bi,
                                                "gap " + std::to_string(actual_gap) +
                                                    " ns vs golden " +
                                                    std::to_string(golden_gap) + " ns"});
            }
        }
        a_previous = ga.t_ns;
        b_previous = ac.t_ns;
        have_previous = true;
    };

    const auto mark_missing = [&](std::size_t ai) {
        result.items.push_back(DiffItem{DiffKind::MissingCommand, ai, DiffItem::kNoIndex,
                                        "golden only: " + describe_frame(a[ai].frame)});
    };
    const auto mark_extra = [&](std::size_t bi) {
        result.items.push_back(DiffItem{DiffKind::ExtraCommand, DiffItem::kNoIndex, bi,
                                        "actual only: " + describe_frame(b[bi].frame)});
    };

    std::size_t i = 0;
    std::size_t j = 0;
    std::size_t skip_a = 0;
    std::size_t skip_b = 0;

    // Closest position within the window where both sides line up, measured by how many frames
    // each has to skip. `require_payload` distinguishes "the same frame" from "the same
    // address".
    const auto find_match = [&](bool require_payload) {
        for (std::size_t total = 1; total <= 2 * kLookahead; ++total) {
            for (std::size_t da = 0; da <= total; ++da) {
                const std::size_t db = total - da;
                if (i + da >= a.size() || j + db >= b.size()) continue;
                if (!same_key(a[i + da], b[j + db])) continue;
                if (require_payload && !same_payload(a[i + da].frame, b[j + db].frame)) continue;
                skip_a = da;
                skip_b = db;
                return true;
            }
        }
        return false;
    };
    const auto skip_forward = [&] {
        for (std::size_t k = 0; k < skip_a; ++k) mark_missing(i + k);
        for (std::size_t k = 0; k < skip_b; ++k) mark_extra(j + k);
        i += skip_a;
        j += skip_b;
    };

    while (i < a.size() && j < b.size()) {
        if (same_key(a[i], b[j]) && same_payload(a[i].frame, b[j].frame)) {
            compare_aligned(i, j);
            ++i;
            ++j;
            continue;
        }

        // An exact match further along means a frame was inserted or dropped. Resynchronise on it
        // instead of reporting a payload difference for every frame in the tail. Without this,
        // one inserted frame cascades -- and the addresses here repeat, so an insertion and a
        // changed payload look identical at the current position.
        if (find_match(/*require_payload=*/true)) {
            skip_forward();
            continue;
        }

        // Nothing nearby matches exactly. The same address at the current position is one frame
        // whose bytes differ.
        if (same_key(a[i], b[j])) {
            compare_aligned(i, j);
            ++i;
            ++j;
            continue;
        }

        // The same address further along: resynchronise there.
        if (find_match(/*require_payload=*/false)) {
            skip_forward();
            continue;
        }

        mark_missing(i);
        mark_extra(j);
        ++i;
        ++j;
    }
    while (i < a.size()) mark_missing(i++);
    while (j < b.size()) mark_extra(j++);

    // A frame on one bus and absent on the other, with the same identifier and length, is a
    // left/right swap. The controller manual's CAN1/CAN2 wiring makes that the most likely way
    // to get this wrong, so it gets its own outcome instead of reading as "some frames missing".
    std::vector<bool> extra_consumed(result.items.size(), false);
    for (std::size_t m = 0; m < result.items.size(); ++m) {
        DiffItem& missing = result.items[m];
        if (missing.kind != DiffKind::MissingCommand) continue;
        const Frame& missing_frame = a[missing.golden_index].frame;
        for (std::size_t e = 0; e < result.items.size(); ++e) {
            if (e == m || extra_consumed[e]) continue;
            DiffItem& extra = result.items[e];
            if (extra.kind != DiffKind::ExtraCommand) continue;
            const Frame& extra_frame = b[extra.actual_index].frame;
            if (extra_frame.id != missing_frame.id || extra_frame.len != missing_frame.len) continue;
            if (extra_frame.bus == missing_frame.bus) continue;

            missing.kind = DiffKind::BusMismatch;
            missing.actual_index = extra.actual_index;
            missing.detail = "same identifier and length on the other bus: golden " +
                             describe_frame(missing_frame) + " vs actual " +
                             describe_frame(extra_frame);
            extra_consumed[e] = true;
            break;
        }
    }
    {
        std::vector<DiffItem> kept;
        kept.reserve(result.items.size());
        for (std::size_t k = 0; k < result.items.size(); ++k) {
            if (!extra_consumed[k]) kept.push_back(result.items[k]);
        }
        result.items = std::move(kept);
    }

    const auto order = [](std::size_t index) {
        return index == DiffItem::kNoIndex ? static_cast<std::size_t>(-1) : index;
    };
    std::stable_sort(result.items.begin(), result.items.end(),
                     [&](const DiffItem& left, const DiffItem& right) {
                         if (order(left.golden_index) != order(right.golden_index)) {
                             return order(left.golden_index) < order(right.golden_index);
                         }
                         return order(left.actual_index) < order(right.actual_index);
                     });
    return result;
}

// ------------------------------------------------------------------ transports

RecordingTransport::RecordingTransport(Transport& inner) : inner_(inner), clock_(steady_now_ns) {
    // The recording receiver is installed here, not in set_receiver: recording is this
    // decorator's job and must not depend on the user happening to ask for frames. set_receiver
    // only adds a handler on top.
    inner_.set_receiver([this](const Frame& frame) {
        trace_.add(frame, clock_(), Direction::Rx);
        if (user_receiver_) user_receiver_(frame);
    });
}

void RecordingTransport::send(const Frame& frame) {
    trace_.add(frame, clock_(), Direction::Tx);
    inner_.send(frame);
}

void RecordingTransport::set_receiver(Receiver receiver) { user_receiver_ = std::move(receiver); }

bool RecordingTransport::healthy() const { return inner_.healthy(); }

Trace RecordingTransport::take_trace() {
    Trace out = std::move(trace_);
    trace_.clear();
    return out;
}

void RecordingTransport::clear() { trace_.clear(); }

void RecordingTransport::set_clock(std::function<std::uint64_t()> clock) {
    clock_ = clock ? std::move(clock) : std::function<std::uint64_t()>(steady_now_ns);
}

ReplayTransport::ReplayTransport(Trace golden) : golden_(std::move(golden)) {}

void ReplayTransport::send(const Frame& frame) {
    if (!healthy_) throw std::runtime_error("ReplayTransport: not healthy");
    sent_.push_back(frame);
}

void ReplayTransport::set_receiver(Receiver receiver) { receiver_ = std::move(receiver); }

bool ReplayTransport::healthy() const { return healthy_; }

void ReplayTransport::set_healthy(bool healthy) { healthy_ = healthy; }

void ReplayTransport::replay_rx() {
    for (const TraceEntry& entry : golden_.entries()) {
        if (entry.direction != Direction::Rx) continue;
        if (receiver_) receiver_(entry.frame);
    }
}

Trace ReplayTransport::sent_trace() const {
    Trace out;
    for (const Frame& frame : sent_) out.add(frame, 0, Direction::Tx);
    return out;
}

void ReplayTransport::clear() { sent_.clear(); }

}  // namespace shensi::can
