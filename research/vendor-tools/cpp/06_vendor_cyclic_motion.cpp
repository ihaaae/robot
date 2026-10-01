// Demo 6 — the vendor's built-in cyclic motion, done directly through the C++ SDK.
//
// The Python demo 06 does the same thing by sending the node's `fixed_action` command. This
// one does not use the node at all: it reproduces the motion itself, which is what you need
// if you are writing your own controller rather than driving the vendor's.
//
// What the vendor's `fixed_action` does, recovered from the handler in
// `dual_arm_app_interface_node`: it calls `getDof()`, copies one of two 17-double constants
// out of .rodata depending on the result, then loops
//
//     MoveJ(all zeros, v)        # home
//     MoveJ(hardcoded pose, v)   # the constant
//     ... count times
//
// On this machine `getDof()` returns 7, so the pose is the constant below: about 90 deg on
// J1 and up to 140 deg on J2 of both arms. The web UI defaults `count` to 1000; this demo
// defaults it to 1.
//
// NOTE: this program calls OnRobot() at startup, which switches the robot to "ready" and
// powers the low-level board. That happens even in a dry run; --yes is what commands motion.
// DRY RUN BY DEFAULT. READ docs/hardware-acceptance.md BEFORE RUNNING THIS ON THE ROBOT.
//
// Build: ./research/vendor-tools/cpp/build.sh
// Run:   DUAL_ARM_SDK_CONFIG=<sdk>/usr/etc LD_LIBRARY_PATH=<sdk>/usr/lib \
//            qemu-aarch64-static -L <sysroot> <build>/06_vendor_cyclic_motion        (dry run)
#include <juxie_controller/juxie_controller.h>

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <string>

namespace {

constexpr int kJointCount = 17;

// The dof == 7 constant, byte for byte out of the vendor binary at .rodata + 0x42a10.
// Same numbers as `vendor_model.VENDOR_CYCLIC_POSE`.
const Juxie::JointSpaceData kVendorCyclicPose = {
    0.0,
    1.57079, 2.443459, 1.80526, 1.58166, 1.530725, 1.258, 1.5,
    1.57079, 2.443459, 1.80526, 1.58166, 1.530725, 1.258, 1.5,
    0.0, 0.0,
};

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

void usage(const char *argv0) {
    std::printf(
        "usage: %s [--count N] [--v V] [--yes]\n"
        "\n"
        "  --count N   cycles of home -> pose (default 1; the vendor UI uses 1000)\n"
        "  --v V       velocity factor passed to MoveJ (default 10)\n"
        "  --yes       actually move. Without it this is a dry run.\n",
        argv0);
}

}  // namespace

int main(int argc, char **argv) {
    int count = 1;
    int velocity = 10;
    bool confirmed = false;

    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--yes") {
            confirmed = true;
        } else if (arg == "--help") {
            usage(argv[0]);
            return 0;
        } else if (arg == "--count" && i + 1 < argc) {
            count = std::atoi(argv[++i]);
        } else if (arg == "--v" && i + 1 < argc) {
            velocity = std::atoi(argv[++i]);
        } else {
            usage(argv[0]);
            return 2;
        }
    }
    if (count < 1 || velocity < 1) {
        std::printf("error: --count and --v must be >= 1\n");
        return 2;
    }

    std::printf("=== what this moves to ===\n");
    const char *names[3] = {"waist", "left arm", "right arm"};
    const int offsets[3] = {0, 1, 8};
    const int lengths[3] = {1, 7, 7};
    for (int g = 0; g < 3; ++g) {
        std::printf("  %-10s", names[g]);
        for (int i = 0; i < lengths[g]; ++i) {
            const double v = kVendorCyclicPose[offsets[g] + i];
            std::printf("  %+8.4f", v);
        }
        std::printf("   rad\n");
    }
    std::printf("  %-10s  %+8.4f  %+8.4f   rad   (head, not driven here)\n", "head",
                kVendorCyclicPose[15], kVendorCyclicPose[16]);

    std::printf("\n=== planned sequence ===\n");
    std::printf("  1. verify the robot state is one of [ready, idle]\n");
    std::printf("  2. EnableRobot()\n");
    std::printf("  3. %d x { MoveJ(home, v=%d); MoveJ(pose, v=%d) }\n", count, velocity, velocity);
    std::printf("  4. DisableRobot()\n");
    std::printf("  The vendor's node also reports cycle_index / cycle_total; this loop prints its\n");
    std::printf("  own progress instead.\n");

    // ---------------------------------------------------------------- preflight
    Juxie::ControllerJuxie controller;
    controller.OnRobot();
    const int state = controller.GetRobotState();

    std::printf("\n=== current state ===\n");
    std::printf("  state     %d (%s)\n", state, state_name(state));

    if (state == 4) {
        std::printf("\n=== PREFLIGHT FAILED -- refusing to move ===\n");
        std::printf("  - robot is in 'fault'. Clear the fault first and find out why it happened.\n");
        std::printf("\n  Without a CAN bus the state is 'fault', so this is expected in emulation.\n");
        return 1;
    }
    if (state != 1 && state != 2) {
        std::printf("\n=== PREFLIGHT FAILED -- refusing to move ===\n");
        std::printf("  - robot state is %s, expected one of [ready, idle]\n", state_name(state));
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
    if (!controller.EnableRobot()) {
        std::printf("failed; refusing to move.\n");
        return 1;
    }
    std::printf("ok\n");

    Juxie::JointSpaceData home{};
    home.fill(0.0);

    // Fail closed: a rejected move stops the sequence and makes the program exit non-zero,
    // rather than reporting success after commanding nothing.
    int failures = 0;
    for (int cycle = 0; cycle < count; ++cycle) {
        const int to_home = controller.MoveJ(home, velocity);
        const int to_pose = to_home == 0 ? controller.MoveJ(kVendorCyclicPose, velocity) : to_home;
        std::printf("  cycle %d/%d  MoveJ(home)=%d  MoveJ(pose)=%d\n",
                    cycle + 1, count, to_home, to_pose);
        std::fflush(stdout);
        if (to_home != 0 || to_pose != 0) {
            failures = 1;
            std::printf("  a move was rejected -- stopping here. -1 is what you get with no CAN\n");
            std::printf("  bus; see research/vendor-analysis/error-codes.md\n");
            break;
        }
    }

    std::printf("  Stop() ... ");
    std::fflush(stdout);
    std::printf("%d\n", static_cast<int>(controller.Stop()));
    std::printf("  DisableRobot() ... ");
    std::fflush(stdout);
    std::printf("%d\n", static_cast<int>(controller.DisableRobot()));

    // Stop() is the panic button; on hardware, wire it to your emergency path as well.
    return failures ? 1 : 0;
}
