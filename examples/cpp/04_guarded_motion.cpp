// Demo 4 — a guarded motion sequence. DRY RUN BY DEFAULT.
//
// One of the three C++ demos that move the robot (with 06 and 07), so it is deliberately
// paranoid:
//
//   * it refuses to move while the robot reports a fault;
//   * it checks that the robot is in a state where motion is accepted;
//   * it validates the target against the limits the controller reports;
//   * it prints the whole plan, and only executes with --yes.
//
// Run it with no --yes first. It will tell you exactly what it would do.
//
//   # dry run -- prints the plan, moves nothing
//   ./04_guarded_motion --joints "0 0 0 0 0 0 0 -100 -100 -100 -100 -100 -100 -100"
//
//   # execute (only after reading the plan)
//   ./04_guarded_motion --joints "..." --yes
//
// NOTE: this program calls OnRobot() at startup, which switches the robot to "ready" and
// powers the low-level board. That happens even in a dry run; --yes is what commands motion.
// READ docs/hardware-acceptance.md BEFORE RUNNING THIS ON THE ROBOT. Several conclusions in
// this repository are unverified on real hardware, including whether the joint angles that
// IK returns reach the pose you asked for (docs/kinematics.md records a 180 mm disagreement
// between the vendor's own IK and FK).
//
// Build: ./examples/cpp/build.sh
#include <juxie_controller/juxie_controller.h>

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

namespace {

constexpr double kInvalid = Juxie::DEFAULT_INVALID_VALUE;  // -100.0 = "leave this joint"

// JointSpaceData is 17 slots: waist(1) | left arm(7) | right arm(7) | head(2).
constexpr int kJointCount = 17;
constexpr int kArmCount = 14;  // the two arms, which is all this machine has

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

bool movable(int state) { return state == 1 /*ready*/ || state == 2 /*idle*/; }

void usage(const char *argv0) {
    std::printf(
        "usage: %s --joints \"<17 values in radians, -100 to leave a joint alone>\" [--v N] [--yes]\n"
        "\n"
        "  --v N     velocity factor passed to MoveJ (default 10)\n"
        "  --yes     actually move. Without it this is a dry run.\n",
        argv0);
}

}  // namespace

int main(int argc, char **argv) {
    std::vector<double> target;
    int velocity = 10;
    bool confirmed = false;

    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--yes") {
            confirmed = true;
        } else if (arg == "--v" && i + 1 < argc) {
            velocity = std::atoi(argv[++i]);
        } else if (arg == "--joints" && i + 1 < argc) {
            std::string text = argv[++i];
            for (char &c : text) {
                if (c == ',') c = ' ';
            }
            const char *cursor = text.c_str();
            char *end = nullptr;
            while (*cursor) {
                const double value = std::strtod(cursor, &end);
                if (end == cursor) break;
                target.push_back(value);
                cursor = end;
            }
        } else {
            usage(argv[0]);
            return 2;
        }
    }

    if (target.size() != kJointCount) {
        std::printf("error: --joints needs %d values, got %zu\n", kJointCount, target.size());
        usage(argv[0]);
        return 2;
    }
    if (velocity <= 0) {
        std::printf("error: --v must be positive\n");
        return 2;
    }

    Juxie::ControllerJuxie controller;
    controller.OnRobot();

    const int state = controller.GetRobotState();
    const auto config = controller.getConfig();
    const auto joints = controller.GetJointPositions();
    const auto codes = controller.getJointerrcode();

    // ---------------------------------------------------------------- what we see
    std::printf("=== current state ===\n");
    std::printf("  state      %d (%s)\n", state, state_name(state));
    std::printf("  joints     ");
    for (int i = 0; i < kJointCount; ++i) {
        std::printf("%+8.4f", joints[i]);
        if (i == 0 || i == 7) std::printf(" |");
    }
    std::printf("\n  errcodes   ");
    bool any_code = false;
    for (int i = 0; i < codes.size(); ++i) {
        std::printf(" %u", static_cast<unsigned>(codes[i]));
        if (codes[i] != 0) any_code = true;
    }
    std::printf("%s\n", any_code ? "" : "   (all zero)");

    // ---------------------------------------------------------------- the plan
    std::printf("\n=== planned sequence ===\n");
    std::printf("  1. verify the robot state is one of [ready, idle]\n");
    std::printf("  2. EnableRobot()\n");
    std::printf("  3. MoveJ(target, v=%d)  -- blocks until the motion finishes\n", velocity);
    std::printf("  4. print the resulting state\n");
    std::printf("  5. DisableRobot()\n");
    std::printf("\n  target     ");
    for (int i = 0; i < kJointCount; ++i) {
        if (target[i] <= kInvalid + 1e-9) std::printf("   (hold)");
        else std::printf("%+8.4f", target[i]);
        if (i == 0 || i == 7) std::printf(" |");
    }
    std::printf("\n");

    // ---------------------------------------------------------------- preflight
    // Return a list of reasons not to move. Empty means it is safe to proceed.
    std::vector<std::string> problems;
    if (state == 4) {
        problems.push_back(
            "robot is in 'fault'. Motion is refused. Clear the fault first and find out why "
            "it happened -- see docs/error-codes.md and the vendor fault table in "
            "docs/can-protocol-comparison.md.");
    } else if (!movable(state)) {
        problems.push_back(std::string("robot state is ") + state_name(state) +
                           ", expected one of [ready, idle]");
    }

    // This machine has no waist or head, so those slots must say "leave alone" rather than
    // carry a target nobody will act on.
    for (int slot : {0, 15, 16}) {
        if (target[slot] != kInvalid) {
            char message[256];
            std::snprintf(message, sizeof(message),
                          "slot %d is %+.4f; this machine has no waist or head, so those "
                          "slots must be %.1f (the \"leave alone\" sentinel)",
                          slot, target[slot], kInvalid);
            problems.push_back(message);
        }
    }

    // Only the two arms exist on this machine, so validate slots 1..14 against their own
    // limits. Both comparisons with NaN are false, so NaN has to be rejected explicitly or
    // it slips through as "in range".
    for (int i = 1; i < 1 + kArmCount; ++i) {
        const double value = target[i];
        if (!std::isfinite(value)) {
            char message[256];
            std::snprintf(message, sizeof(message), "J%d target is not a finite number", i);
            problems.push_back(message);
            continue;
        }
        if (value == kInvalid) continue;  // exactly -100 means "leave this joint alone"
        if (value < config.lower[i - 1] - 1e-9 || value > config.upper[i - 1] + 1e-9) {
            char message[256];
            std::snprintf(message, sizeof(message),
                          "J%d target %+.4f rad is outside [%+.4f, %+.4f]",
                          i, value, config.lower[i - 1], config.upper[i - 1]);
            problems.push_back(message);
        }
    }
    if (any_code) {
        problems.push_back("the controller reports non-zero joint error codes");
    }

    if (!problems.empty()) {
        std::printf("\n=== PREFLIGHT FAILED -- refusing to move ===\n");
        for (const auto &problem : problems) {
            std::printf("  - %s\n", problem.c_str());
        }
        return 1;
    }
    std::printf("\n  preflight: ok\n");

    if (!confirmed) {
        std::printf("\n=== DRY RUN -- nothing was sent. Re-run with --yes to execute. ===\n");
        return 0;
    }

    // ---------------------------------------------------------------- execute
    std::printf("\n=== executing ===\n");
    std::printf("  EnableRobot() ... ");
    std::fflush(stdout);
    const bool enabled = controller.EnableRobot();
    std::printf("%d\n", static_cast<int>(enabled));
    if (!enabled) {
        std::printf("  refusing to move: the robot did not reach 'idle'.\n");
        return 1;
    }

    Juxie::JointSpaceData command{};
    for (int i = 0; i < kJointCount; ++i) command[i] = target[i];

    std::printf("  MoveJ(v=%d) ... ", velocity);
    std::fflush(stdout);
    const int move = controller.MoveJ(command, velocity);
    std::printf("%d%s\n", move, move == 0 ? "" : "  <- non-zero: see state/error_code.h");

    std::printf("  state after = %d (%s)\n", controller.GetRobotState(),
                state_name(controller.GetRobotState()));

    std::printf("  DisableRobot() ... ");
    std::fflush(stdout);
    std::printf("%d\n", static_cast<int>(controller.DisableRobot()));

    // Stop() is the panic button: it halts motion and returns the robot to idle. It is not
    // called here, but wire it to your emergency path before you run this on real hardware.
    return move == 0 ? 0 : 1;
}
