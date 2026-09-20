// Demo 7 — replay the vendor's recorded trajectory through the streaming interface.
//
// This is the counterpart to the vendor's own `test_controller --gtest_filter=*MoveJCanfdTest*`,
// which is the only place in the package that reads these CSVs. That test builds the path as
//
//     std::string(getenv("DUAL_ARM_SDK_CONFIG")) + "/data/array0_all/array04_2.csv"
//
// parses each row, and feeds it to MoveJ_Canfd(..., 50) — 50 Hz, the top of the 10-50 Hz range
// the header documents. This demo does the same thing, with a dry run in front of it.
//
// Why the streaming interface and not MoveJ: the CSV is a dense sequence of setpoints (727
// rows here, largest step between rows about 0.007 rad). You are the one doing the
// interpolation, so you must hand the controller a new point every 20 ms. MoveJ would instead
// take a single target and plan the whole move itself.
//
// NOTE: this program calls OnRobot() at startup, which switches the robot to "ready" and
// powers the low-level board. That happens even in a dry run; --yes is what commands motion.
// READ THIS BEFORE RUNNING IT ON THE ROBOT
// ----------------------------------------
// MoveJ_Canfd / MoveP_Canfd are the least verified part of this SDK: nothing in this
// repository has ever exercised them against hardware, and under emulation they return -1
// (RobotConnectFailed). See docs/hardware-acceptance.md P1-2. Start with --limit 50 --rate 10.
//
// Column mapping, taken from the vendor's test rather than guessed: the CSV has 14 columns
// (left arm 7, then right arm 7) and it fills a 17-slot JointSpaceData as
//
//     joints[0]      = -100.0          waist: DEFAULT_INVALID_VALUE, "keep current"
//     joints[1..7]   = csv[0..6]       left arm
//     joints[8..14]  = csv[7..13]      right arm
//     joints[15..16] = (not written by the vendor's test; this demo sends -100.0)
//
// The vendor's test leaves the head slots unwritten, which is undefined. This demo sends
// -100.0 there, the documented "leave alone" value, so it is at least deliberate.
//
// Build: ./examples/cpp/build.sh
// Run:   DUAL_ARM_SDK_CONFIG=<sdk>/usr/etc LD_LIBRARY_PATH=<sdk>/usr/lib \
//            qemu-aarch64-static -L <sysroot> <build>/07_replay_trajectory          (dry run)
#include <juxie_controller/juxie_controller.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <string>
#include <thread>
#include <vector>

namespace {

constexpr int kJointCount = 17;
constexpr double kInvalid = Juxie::DEFAULT_INVALID_VALUE;  // -100.0

const char *state_name(int state) {
    switch (state) {
        case 0: return "power_off";
        case 1: return "ready";
        case 2: return "idle";
        case 3: return "running";
        case 4: return "fault";
        default: return "unknown";
    }
}

// The vendor's own default path, built the way its test builds it.
//
// DUAL_ARM_SDK_CONFIG is not optional: libjuxie_controller reads it during OnRobot() and
// aborts with "basic_string::_M_construct null not valid" if it is missing. That happens
// inside the vendor library, not here, so check it before doing anything else.
const char *config_root() {
    const char *root = std::getenv("DUAL_ARM_SDK_CONFIG");
    if (!root || !*root) {
        std::printf("error: DUAL_ARM_SDK_CONFIG is not set.\n\n");
        std::printf("  libjuxie_controller reads it during OnRobot() and aborts without it, so\n");
        std::printf("  this is a requirement of the vendor library, not of this demo. For the\n");
        std::printf("  committed SDK tree:\n\n");
        std::printf("      export DUAL_ARM_SDK_CONFIG=$PWD/vendor/sdk/dual-arm-app/0.6.4/usr/etc\n\n");
        std::printf("  See docs/sdk.md section 8, item 4.\n");
        std::exit(2);
    }
    return root;
}

std::string default_csv() {
    return std::string(config_root()) + "/data/array0_all/array04_2.csv";
}

std::vector<std::vector<double>> read_csv(const std::string &path, int columns) {
    std::ifstream file(path);
    if (!file) {
        std::printf("error: cannot open %s\n", path.c_str());
        std::printf("  Set DUAL_ARM_SDK_CONFIG, or pass --csv. The file ships inside the .deb\n");
        std::printf("  and is committed under vendor/sdk/dual-arm-app/0.6.4/usr/etc/data/.\n");
        std::exit(2);
    }

    std::vector<std::vector<double>> rows;
    std::string line;
    while (std::getline(file, line)) {
        if (!line.empty() && line.back() == '\r') line.pop_back();  // the files are CRLF
        if (line.empty()) continue;

        std::vector<double> row;
        const char *cursor = line.c_str();
        char *end = nullptr;
        while (*cursor) {
            const double value = std::strtod(cursor, &end);
            if (end == cursor) break;
            row.push_back(value);
            cursor = end;
            if (*cursor == ',') ++cursor;
        }
        if (row.empty()) continue;
        if (static_cast<int>(row.size()) != columns) {
            std::printf("error: %s has a row with %zu columns, expected %d\n",
                        path.c_str(), row.size(), columns);
            std::exit(2);
        }
        rows.push_back(std::move(row));
    }
    return rows;
}

// CSV row -> 17-slot JointSpaceData, exactly as the vendor's test does it.
Juxie::JointSpaceData to_joint_space(const std::vector<double> &row) {
    Juxie::JointSpaceData joints{};
    joints.fill(kInvalid);                      // waist and head: "keep current"
    for (int i = 0; i < 14; ++i) joints[i + 1] = row[i];
    return joints;
}

void usage(const char *argv0) {
    std::printf(
        "usage: %s [--csv PATH] [--rate HZ] [--limit N] [--yes]\n"
        "\n"
        "  --csv PATH   trajectory to replay (default: $DUAL_ARM_SDK_CONFIG/data/array0_all/array04_2.csv)\n"
        "  --rate HZ    streaming rate, 10-50 (default 50, the value the vendor's test uses)\n"
        "  --limit N    stream only the first N rows (default: all)\n"
        "  --max-start-jump RAD  largest acceptable first step (default 0.35, about 20 deg)\n"
        "  --yes        actually move. Without it this is a dry run.\n",
        argv0);
}

}  // namespace

int main(int argc, char **argv) {
    std::string csv;
    int rate = 50;
    long limit = -1;
    double max_start_jump = 0.35;  // rad, about 20 deg
    bool confirmed = false;

    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--yes") {
            confirmed = true;
        } else if (arg == "--help") {
            usage(argv[0]);
            return 0;
        } else if (arg == "--csv" && i + 1 < argc) {
            csv = argv[++i];
        } else if (arg == "--rate" && i + 1 < argc) {
            rate = std::atoi(argv[++i]);
        } else if (arg == "--limit" && i + 1 < argc) {
            limit = std::atol(argv[++i]);
        } else if (arg == "--max-start-jump" && i + 1 < argc) {
            max_start_jump = std::atof(argv[++i]);
        } else {
            usage(argv[0]);
            return 2;
        }
    }
    config_root();  // fail early and clearly, before the vendor library can abort
    if (csv.empty()) csv = default_csv();

    if (rate < 10 || rate > 50) {
        std::printf("error: --rate must be 10-50 Hz; the SDK documents that range.\n");
        return 2;
    }
    if (limit == 0 || limit < -1) {
        std::printf("error: --limit must be positive (omit it to stream every row)\n");
        return 2;
    }

    const auto rows = read_csv(csv, 14);
    const long count = limit > 0 ? std::min<long>(limit, static_cast<long>(rows.size()))
                                 : static_cast<long>(rows.size());
    if (count == 0) {
        std::printf("error: %s has no rows\n", csv.c_str());
        return 2;
    }

    // ---------------------------------------------------------------- what we are about to send
    std::printf("=== trajectory ===\n");
    std::printf("  file      %s\n", csv.c_str());
    std::printf("  rows      %zu total, streaming %ld\n", rows.size(), count);
    std::printf("  rate      %d Hz  (%.1f ms per point, %.2f s for %ld rows)\n",
                rate, 1000.0 / rate, static_cast<double>(count) / rate, count);

    double worst_step = 0.0;
    for (long i = 1; i < count; ++i) {
        for (int c = 0; c < 14; ++c) {
            worst_step = std::max(worst_step, std::fabs(rows[i][c] - rows[i - 1][c]));
        }
    }
    std::printf("  largest per-step joint change  %.5f rad (%.2f deg)\n",
                worst_step, worst_step * 180.0 / M_PI);

    std::printf("  per-joint travel (rad):\n");
    for (int arm = 0; arm < 2; ++arm) {
        std::printf("    %-5s", arm == 0 ? "left" : "right");
        for (int j = 0; j < 7; ++j) {
            double lo = rows[0][arm * 7 + j], hi = lo;
            for (long i = 0; i < count; ++i) {
                lo = std::min(lo, rows[i][arm * 7 + j]);
                hi = std::max(hi, rows[i][arm * 7 + j]);
            }
            std::printf(" %6.3f", hi - lo);
        }
        std::printf("\n");
    }
    std::printf("  first row -> joints[1..14], with joints[0] = joints[15..16] = %.1f (keep current)\n",
                kInvalid);

    // ---------------------------------------------------------------- preflight
    Juxie::ControllerJuxie controller;
    controller.OnRobot();
    const int state = controller.GetRobotState();
    const auto current = controller.GetJointPositions();

    std::printf("\n=== current state ===\n");
    std::printf("  state     %d (%s)\n", state, state_name(state));

    std::vector<std::string> problems;
    if (state == 4) {
        problems.push_back("robot is in 'fault'. Motion is refused. Clear the fault first.");
    } else if (state != 1 && state != 2) {
        problems.push_back(std::string("robot state is ") + state_name(state) +
                           ", expected one of [ready, idle]");
    }

    for (const auto &row : rows) {
        for (double value : row) {
            if (!std::isfinite(value)) {
                problems.push_back("the trajectory contains a non-finite value");
                break;
            }
        }
    }

    // The file starts well away from zero (1.074 rad on joint 3). Nothing plans that move in
    // streaming mode, so refuse unless the operator accepts a first step this large.
    double jump = 0.0;
    bool have_telemetry = false;
    for (int i = 0; i < 14; ++i) {
        if (current[i + 1] <= kInvalid + 1e-9) continue;
        have_telemetry = true;
        jump = std::max(jump, std::fabs(rows[0][i] - current[i + 1]));
    }
    std::printf("  first commanded step  ");
    if (!have_telemetry) {
        std::printf("no joint telemetry, cannot check\n");
        problems.push_back("no usable joint telemetry, so the first commanded step cannot be "
                           "checked. Refusing to stream blind.");
    } else {
        std::printf("%.4f rad (%.1f deg), limit %.4f rad\n",
                    jump, jump * 180.0 / M_PI, max_start_jump);
        if (jump > max_start_jump) {
            char message[256];
            std::snprintf(message, sizeof(message),
                          "the trajectory starts %.4f rad (%.1f deg) from where the robot is "
                          "now, and streaming does not plan that move. Move closer first, or "
                          "raise --max-start-jump if you mean it.",
                          jump, jump * 180.0 / M_PI);
            problems.push_back(message);
        }
    }

    if (!problems.empty()) {
        std::printf("\n=== PREFLIGHT FAILED -- refusing to move ===\n");
        for (const auto &problem : problems) std::printf("  - %s\n", problem.c_str());
        std::printf("\n  Note: without a CAN bus the state is 'fault', so this is expected in\n");
        std::printf("  emulation. The dry run below still shows the plan.\n");
    }

    if (!confirmed) {
        std::printf("\n=== DRY RUN -- nothing was sent. Re-run with --yes to execute. ===\n");
        std::printf("  The sequence would be: enable_robot, %ld x MoveJ_Canfd(..., %d), then\n",
                    count, rate);
        std::printf("  disable_robot. On hardware, watch the first --limit 50 rows before\n");
        std::printf("  letting it run the whole file.\n");
        return problems.empty() ? 0 : 1;
    }
    if (!problems.empty()) return 1;

    // ---------------------------------------------------------------- stream
    std::printf("\n=== streaming ===\n");
    std::printf("  EnableRobot() ... ");
    std::fflush(stdout);
    if (!controller.EnableRobot()) {
        std::printf("failed; refusing to stream.\n");
        return 1;
    }
    std::printf("ok\n");

    const auto period = std::chrono::microseconds(1000000 / rate);
    int failures = 0;
    long sent = 0;
    double worst_track = 0.0;

    for (long i = 0; i < count; ++i) {
        const auto deadline = std::chrono::steady_clock::now() + period;
        const auto joints = to_joint_space(rows[i]);

        // Fail closed: the first rejected setpoint ends the stream. Continuing would keep
        // commanding a robot that is not following, and returning success would hide it.
        const int ret = controller.MoveJ_Canfd(joints, rate);
        ++sent;
        if (ret != 0) {
            failures = 1;
            std::printf("  row %ld: MoveJ_Canfd returned %d -- aborting the stream\n", i, ret);
            break;
        }

        // Compare what we asked for with what the robot reports, so a lagging or ignoring
        // controller shows up as a growing number rather than as a silent no-op.
        const auto actual = controller.GetJointPositions();
        double worst = 0.0;
        for (int j = 1; j <= 14; ++j) {
            if (joints[j] <= kInvalid + 1e-9) continue;
            if (actual[j] <= kInvalid + 1e-9) continue;
            worst = std::max(worst, std::fabs(actual[j] - joints[j]));
        }
        worst_track = std::max(worst_track, worst);

        // Abort rather than burst: if the loop fell behind, the remaining points would go
        // out faster than the requested rate.
        const auto now = std::chrono::steady_clock::now();
        if (now - deadline > 5 * period) {
            std::printf("  fell more than 5 periods behind at row %ld -- aborting instead of "
                        "bursting\n", i);
            failures = 1;
            break;
        }
        if ((i + 1) % 100 == 0 || i + 1 == count) {
            std::printf("  %ld/%ld  worst tracking error so far %.4f rad\n",
                        i + 1, count, worst_track);
        }
        std::this_thread::sleep_until(deadline);
    }

    std::printf("  Stop() ... ");
    std::fflush(stdout);
    std::printf("%d\n", static_cast<int>(controller.Stop()));
    std::printf("  DisableRobot() ... ");
    std::fflush(stdout);
    std::printf("%d\n", static_cast<int>(controller.DisableRobot()));

    std::printf("\n  %ld of %ld points sent, worst tracking error %.4f rad\n",
                sent, count, worst_track);
    if (failures) {
        std::printf("  the stream did not complete cleanly (-1 is what you get with no CAN\n");
        std::printf("  bus; see docs/error-codes.md)\n");
    }
    return failures ? 1 : 0;
}
