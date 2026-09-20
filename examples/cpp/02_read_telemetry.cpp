// Demo 2 — read telemetry from the robot through the vendor SDK.
//
// NOTE: this program calls OnRobot() at startup, which switches the robot to "ready" and
// powers the low-level board. That happens even in a dry run; --yes is what commands motion.
// Needs a robot (or the emulated control node) on the CAN bus. Without one it still runs
// and still shows you the important thing: what "no data" looks like, so your code checks
// for it instead of feeding -100 into a planner.
//
// What it shows:
//   * the 17-slot joint layout decoded into waist / left arm / right arm / head;
//   * the DEFAULT_INVALID_VALUE sentinel as a *return* value, not just a command value;
//   * TCP pose in quaternion form (GetTCPPose) and the joint torques;
//   * the three separate error encodings the SDK exposes, which do not share a scale.
//
// The vendor library prints its own debug output to stdout before your own. Normal.
//
// Build: ./examples/cpp/build.sh
// Run:   DUAL_ARM_SDK_CONFIG=<sdk>/usr/etc LD_LIBRARY_PATH=<sdk>/usr/lib \
//            qemu-aarch64-static -L <sysroot> <build>/02_read_telemetry   (or natively on arm64)
#include <juxie_controller/juxie_controller.h>

#include <cstdio>
#include <vector>

namespace {

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

// The joint vector is 17 slots: waist(1) | left arm(7) | right arm(7) | head(2).
// This machine has 14 motors on two CAN buses, so the waist and head slots are part of the
// product family's superset API and are not populated here.
void print_group(const char *label, const Juxie::JointSpaceData &j, int offset, int count) {
    std::printf("  %-11s", label);
    bool any = false;
    for (int i = 0; i < count; ++i) {
        const double v = j[offset + i];
        if (v <= kInvalid + 1e-9) {
            std::printf("  %8s", "--");
        } else {
            std::printf("  %+8.4f", v);
            any = true;
        }
    }
    std::printf("%s\n", any ? "" : "   (no data)");
}

}  // namespace

int main() {
    Juxie::ControllerJuxie controller;

    // Required before anything else, even for reads. See docs/sdk-usage.md.
    const bool ready = controller.OnRobot();

    std::printf("=== state ===\n");
    std::printf("  OnRobot()  = %d %s\n", static_cast<int>(ready),
                ready ? "" : "(no CAN: internal state is initialised, but nothing is attached)");
    std::printf("  state      = %d (%s)\n", controller.GetRobotState(),
                state_name(controller.GetRobotState()));

    // ---------------------------------------------------------------- joints
    const auto joints = controller.GetJointPositions();
    std::printf("\n=== joints (17 slots, radians) ===\n");
    print_group("waist", joints, 0, 1);
    print_group("left arm", joints, 1, 7);
    print_group("right arm", joints, 8, 7);
    print_group("head", joints, 15, 2);
    std::printf("  -- means the SDK returned DEFAULT_INVALID_VALUE (-100.0). The header\n");
    std::printf("  documents -100 as a command sentinel (\"keep the current position\"), but it is\n");
    std::printf("  also what GetJointPositions / GetTCPPose return *before* OnRobot() runs --\n");
    std::printf("  every slot is -100 then. After OnRobot() with no CAN attached they come back\n");
    std::printf("  as zeros instead, which is why this run shows zeros. Check for both.\n");

    // ---------------------------------------------------------------- TCP
    // 14 values: per arm x, y, z then a quaternion (qw, qx, qy, qz).
    const auto tcp = controller.GetTCPPose();
    std::printf("\n=== TCP pose (per arm, relative to that arm's base -- NOT the world) ===\n");
    for (int arm = 0; arm < 2; ++arm) {
        const int o = arm * 7;
        std::printf("  %-5s xyz = %8.5f %8.5f %8.5f   quat = %.5f %.5f %.5f %.5f\n",
                    arm == 0 ? "left" : "right",
                    tcp[o + 0], tcp[o + 1], tcp[o + 2],
                    tcp[o + 3], tcp[o + 4], tcp[o + 5], tcp[o + 6]);
    }

    // ---------------------------------------------------------------- torques
    const auto torques = controller.GetSingleTorques();
    std::printf("\n=== torques (14 values: left 7, then right 7) ===\n");
    std::printf("  ");
    for (int i = 0; i < 14; ++i) {
        std::printf("%+9.4f", torques[i]);
        if (i == 6) std::printf("   |");
    }
    std::printf("\n");

    // ---------------------------------------------------------------- errors
    // Three encodings that do not share a scale:
    //   GetRobotState()          -> RobotState (0..4)
    //   getJointerrcode()        -> per-joint status words (unsigned short)
    //   GetFaultType(code)       -> the controller's own reason strings
    //   GetAxisFault(code)       -> the vendor's CANopen fault table
    std::printf("\n=== error encodings ===\n");
    const auto codes = controller.getJointerrcode();
    std::printf("  getJointerrcode():");
    bool any_code = false;
    for (int i = 0; i < codes.size(); ++i) {
        std::printf(" %u", static_cast<unsigned>(codes[i]));
        if (codes[i] != 0) any_code = true;
    }
    std::printf("%s\n", any_code ? "" : "   (all zero)");
    for (int i = 0; i < codes.size(); ++i) {
        if (codes[i] == 0) continue;
        std::printf("    J%d  code=%u  axis fault: %s\n", i + 1, static_cast<unsigned>(codes[i]),
                    controller.GetAxisFault(codes[i]).c_str());
    }
    std::printf("  GetFaultType(%d) = %s\n", codes[0],
                controller.GetFaultType(codes[0]).c_str());
    std::printf("  ^ GetFaultType takes an *error code*, not a robot state. Passing a state\n");
    std::printf("    value (0..4) silently returns an unrelated string.\n");

    // ---------------------------------------------------------------- limits
    const auto config = controller.getConfig();
    std::printf("\n=== limits as the controller reports them ===\n");
    std::printf("  lower[0]=%+.4f upper[0]=%+.4f   (all 14 are the same +-pi defaults;\n",
                config.lower[0], config.upper[0]);
    std::printf("  params.yml carries placeholders and sets UseLimit: false)\n");
    return 0;
}
