// Demo 1 — offline kinematics through the vendor SDK. No robot, no CAN bus.
//
// This is the only C++ demo that runs with nothing attached, so it is the right starting
// point while the robot is still in transit.
//
// SAFE ON HARDWARE BY DEFAULT. This is the one C++ program here that does not power the
// robot: everything it does by default (FK, getConfig, GetRobotState, GetTCPPose) works
// without OnRobot(), so you can run it on the robot's own computer without energising
// anything. --power-on adds the parts that need OnRobot() (getDof and IK), and OnRobot()
// switches the robot to "ready", which powers the low-level board.
//
// What it shows:
//   * which SDK calls need OnRobot() and which do not;
//   * getFKpose at the zero configuration, next to get_tcp_pose;
//   * the joint limits from getConfig(), and the fact that nothing enforces them;
//   * IK, and the two things that surprise people about it:
//       - the request must carry a valid pose for BOTH arms. DEFAULT_INVALID_VALUE is
//         rejected here, even though it means "leave this joint alone" in MoveJ;
//       - IK's forward model is NOT getFKpose. The vendor's own two commands disagree,
//         by about 180 mm for the target below. See docs/kinematics.md.
//
//   * the getFKpose buffer overflow, which is the root cause of the "get_FK_pose crashes
//     the node" bug recorded in docs/sdk.md. Pass at most 14 values. The overflow is
//     reproduced by fk_overflow_repro.cpp, not here.
//
// The vendor library prints its own debug output ("Link_XYZ", matrices) to stdout before
// your own output. That is its normal behaviour, not a sign that something went wrong.
//
// Build: ./examples/cpp/build.sh
// Run:   DUAL_ARM_SDK_CONFIG=<sdk>/usr/etc LD_LIBRARY_PATH=<sdk>/usr/lib \
//            qemu-aarch64-static -L <sysroot> <build>/01_offline_kinematics   (or natively on arm64)
//            ... 01_offline_kinematics --power-on     # also getDof and IK; powers the board
#include <juxie_controller/juxie_controller.h>

#include <cmath>
#include <cstdio>
#include <cstring>
#include <vector>

namespace {

// From juxie_controller/juxie_state.hpp. That header cannot be included directly: it pulls
// in bot_executor/ExecutorJuxie.hpp, which needs headers the vendor did not ship.
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

// getFKpose returns 12 values: 6 per arm (x, y, z, rx, ry, rz), left arm first.
void print_pose(const char *label, const Juxie::CartesianSpaceDataRPY &pose, int arm) {
    const int o = arm * 6;
    std::printf("  %-22s xyz = %8.5f %8.5f %8.5f    rpy = %8.5f %8.5f %8.5f\n",
                label, pose[o + 0], pose[o + 1], pose[o + 2],
                pose[o + 3], pose[o + 4], pose[o + 5]);
}

// Defined after main; declared here so both the early-exit and the normal path can call it.
void print_overflow_note();

double xyz_distance(const double *a, const double *b) {
    const double dx = a[0] - b[0], dy = a[1] - b[1], dz = a[2] - b[2];
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

}  // namespace

int main(int argc, char **argv) {
    bool power_on = false;
    for (int i = 1; i < argc; ++i) {
        if (std::strcmp(argv[i], "--power-on") == 0) {
            power_on = true;
        } else if (std::strcmp(argv[i], "--help") == 0) {
            std::printf("usage: %s [--power-on]\n\n", argv[0]);
            std::printf("Offline kinematics through the vendor SDK.\n"
                        "  --power-on  also call OnRobot() so getDof() and IK() can run.\n"
                        "              OnRobot() switches the robot to 'ready', which powers\n"
                        "              the low-level board. Without this flag nothing is\n"
                        "              powered and no motion command is ever sent.\n");
            return 0;
        } else {
            std::printf("usage: %s [--power-on]\n", argv[0]);
            return 2;
        }
    }

    Juxie::ControllerJuxie controller;

    // OnRobot() is the only call here that touches the hardware's power state: it switches
    // the robot to "ready". getFKpose, getConfig, GetRobotState and GetTCPPose all work
    // without it. getDof, IK, getJointerrcode and setJointZeroPosition do not -- they
    // dereference an uninitialised state pointer and segfault -- which is why they are
    // behind --power-on.
    std::printf("=== controller ===\n");
    if (power_on) {
        const bool ready = controller.OnRobot();
        std::printf("  OnRobot()   = %d        (0 = no CAN attached; state is still initialised)\n",
                    static_cast<int>(ready));
        std::printf("  state       = %d (%s)\n", controller.GetRobotState(),
                    state_name(controller.GetRobotState()));
        std::printf("  getDof()    = %d        (joints per arm; the joint API is 17 slots)\n",
                    controller.getDof());
    } else {
        std::printf("  OnRobot()   not called  (--power-on to enable; it powers the board)\n");
        std::printf("  state       = %d (%s)\n", controller.GetRobotState(),
                    state_name(controller.GetRobotState()));
        std::printf("  getDof()    skipped     (needs OnRobot(); without it this call segfaults)\n");
        std::printf("  Everything below here works without OnRobot().\n");
    }

    // ---------------------------------------------------------------- forward kinematics
    // 14 values, not 17. See the overflow section at the end of this file.
    std::printf("\n=== forward kinematics ===\n");
    std::vector<double> zero(14, 0.0);
    auto fk = controller.getFKpose(zero, 7, 7);
    print_pose("left  arm at zeros", fk, 0);
    print_pose("right arm at zeros", fk, 1);

    auto tcp = controller.GetTCPPose();
    if (tcp[2] <= Juxie::DEFAULT_INVALID_VALUE + 1e-9) {
        // Without OnRobot() the telemetry reads come back as the sentinel rather than as
        // values, so there is nothing to compare FK against.
        std::printf("  get_tcp_pose left z  = %.5f  (the -100 sentinel: no telemetry until\n", tcp[2]);
        std::printf("                         OnRobot() has run, so FK cannot be compared here)\n");
    } else {
        std::printf("  get_tcp_pose left z  = %.5f   (quaternion form, from the same zeros)\n", tcp[2]);
        std::printf("  ^ FK and get_tcp_pose differ by about 0.3 mm here. That gap is part of the\n");
        std::printf("    three-model inconsistency, not a typo -- see docs/kinematics.md.\n");
    }

    // ---------------------------------------------------------------- joint limits
    std::printf("\n=== joint limits from getConfig() ===\n");
    const auto config = controller.getConfig();
    for (int i = 0; i < 14; ++i) {
        std::printf("  J%-2d  [%+.4f, %+.4f] rad = [%+7.2f, %+7.2f] deg\n",
                    i + 1, config.lower[i], config.upper[i],
                    config.lower[i] * 180.0 / M_PI, config.upper[i] * 180.0 / M_PI);
    }
    std::printf("  NOTE: these are the library defaults and look like placeholders. The\n");
    std::printf("        [1.5, 6.5] pairs in params.yml are per-joint max velocity /\n");
    std::printf("        acceleration, not position limits (UseLimit: false turns that limiter\n");
    std::printf("        off). Whether anything enforces these position limits is unverified.\n");
    std::printf("        Your own planner has to decide.\n");

    // ---------------------------------------------------------------- inverse kinematics
    std::printf("\n=== inverse kinematics ===\n");
    if (!power_on) {
        std::printf("  skipped: IK() needs OnRobot(), which this run did not call.\n");
        std::printf("  Re-run with --power-on to include it (that powers the board).\n");
        std::printf("\n=== DEFAULT_INVALID_VALUE is not accepted by IK ===\n");
        std::printf("  skipped for the same reason.\n");
        print_overflow_note();
        return 0;
    }
    // CartesianSpaceData is 14 values: left (x, y, z, qw, qx, qy, qz) then right.
    // Both arms must carry a real pose. See the sentinel case below.
    Juxie::CartesianSpaceData target{};
    for (auto &v : target) v = 0.0;
    target[0] = 0.20; target[1] = 0.0; target[2] = 0.50; target[3] = 1.0;  // left, identity quat
    target[7] = 0.20; target[8] = 0.0; target[9] = 0.50; target[10] = 1.0;  // right, same target

    Eigen::VectorXd joints;
    const int ret = controller.IK(target, joints);
    std::printf("  IK(both arms) = %d, %ld joints\n", ret, static_cast<long>(joints.size()));
    if (ret == 0 && joints.size() >= 14) {
        std::printf("  left  joints =");
        for (int i = 0; i < 7; ++i) std::printf(" %+.6f", joints[i]);
        std::printf("\n");

        // Feed the vendor's own solution back into the vendor's own FK.
        std::vector<double> q(14, 0.0);
        for (int i = 0; i < 14; ++i) q[i] = joints[i];
        auto solved = controller.getFKpose(q, 7, 7);
        print_pose("IK solution, via FK", solved, 0);
        std::printf("  residual = %.2f mm  <- the vendor's IK does not invert its own FK\n",
                    xyz_distance(&solved[0], &target[0]) * 1000.0);
        std::printf("  (rotation matches exactly; the offset is along the tool z axis, and the\n");
        std::printf("   84.721 mm constant is what our offline model compensates for. See\n");
        std::printf("   docs/kinematics.md before using IK output with MoveJ_P.)\n");
    } else {
        std::printf("  no solution returned\n");
    }

    // ---------------------------------------------------------------- the sentinel trap
    std::printf("\n=== DEFAULT_INVALID_VALUE is not accepted by IK ===\n");
    Juxie::CartesianSpaceData one_armed{};
    for (auto &v : one_armed) v = Juxie::DEFAULT_INVALID_VALUE;
    one_armed[0] = 0.20; one_armed[1] = 0.0; one_armed[2] = 0.50; one_armed[3] = 1.0;
    Eigen::VectorXd ignored;
    const int sentinel_ret = controller.IK(one_armed, ignored);
    std::printf("  IK(left only, right = -100) = %d   (expected non-zero)\n", sentinel_ret);
    std::printf("  The header says -100 means \"keep the current position\", and that holds for\n");
    std::printf("  MoveJ / MoveJ_P. IK rejects it: the whole call fails. Nothing raises, so\n");
    std::printf("  always check the return code.\n");

    // ---------------------------------------------------------------- the overflow
    print_overflow_note();
    return 0;
}

namespace {

void print_overflow_note() {
    std::printf("\n=== getFKpose takes at most 14 values ===\n");
    std::printf("  getFKpose allocates a fixed 7-double buffer and copies (n - 7) doubles into\n");
    std::printf("  it, so it is in bounds only while n <= 14. LeftNum / RightNum do not affect\n");
    std::printf("  the buffer size. Measured under AddressSanitizer: n<=14 clean, n>=15 reports\n");
    std::printf("  a heap-buffer-overflow WRITE (64 bytes for n=15, 80 for n=17).\n");
    std::printf("  Without a sanitizer the call usually returns a plausible pose and the damage\n");
    std::printf("  surfaces at the next allocation, if at all.\n");
    std::printf("  Reproduce it with examples/cpp/fk_overflow_repro.cpp -- see its header.\n");
}

}  // namespace
