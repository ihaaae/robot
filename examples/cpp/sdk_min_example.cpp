// Minimal program that links the vendor's high-level SDK, and the best starting point for
// your own. Copy it out of this repository and adapt it.
//
// It includes ONLY <juxie_controller/juxie_controller.h>, which is the public API and needs
// nothing but Eigen. It does not pull in the headers the vendor forgot to ship, and it does
// not use anything that only exists in this repository.
//
// Build, with CMake -- see cmake/juxie-sdk.cmake, which carries the flags below for you:
//
//   cmake_minimum_required(VERSION 3.16)
//   project(my_app CXX)
//   set(CMAKE_CXX_STANDARD 17)
//   include(<repo>/cmake/juxie-sdk.cmake)
//   add_executable(my_app main.cpp)
//   target_link_libraries(my_app PRIVATE Juxie::SDK)
//
// Build, by hand (aarch64; run this on the robot's board or cross-compile for it):
//
//   aarch64-linux-gnu-g++ -std=c++17 -I<sdk>/usr/include -I/usr/include/eigen3 \
//       sdk_min_example.cpp \
//       -L<sdk>/usr/lib -Wl,-rpath-link,<sdk>/usr/lib \
//       -Wl,--disable-new-dtags -Wl,-rpath,<sdk>/usr/lib \
//       -ljuxie_controller -o sdk_min_example
//
// -Wl,-rpath-link is required at link time. libjuxie_controller.so pulls in libexecutor,
// libbot_servo, libbot_planner, libbot_traj_planner and libbot_kinematics, and without it the
// linker reports "undefined reference" -- which reads like a missing header but is not.
//
// -Wl,--disable-new-dtags is the run-time half. None of those five libraries carries an RPATH
// of its own, and DT_RUNPATH (what the toolchain emits by default) is not searched for
// transitive dependencies, so the program dies with "libexecutor.so.3: cannot open shared
// object file". --disable-new-dtags emits DT_RPATH instead, which is searched transitively.
// Setting LD_LIBRARY_PATH works too.
//
// At run time the SDK needs the configuration root in DUAL_ARM_SDK_CONFIG:
//   export DUAL_ARM_SDK_CONFIG=<sdk>/usr/etc
#include <juxie_controller/juxie_controller.h>

#include <cstdio>
#include <vector>

int main() {
    std::printf("link ok: header parsed and library resolved\n");

    Juxie::ControllerJuxie controller;

    // ---------------------------------------------------------------- without OnRobot()
    // These calls work before OnRobot() and touch no hardware. getFKpose takes a std::vector
    // plus how many of its values belong to each arm; use the tested shape, 14 values with
    // 7 + 7. Passing more than 14 overflows a fixed 7-double buffer inside the SDK -- see
    // docs/sdk-usage.md section 6.1.
    std::vector<double> zeros(14, 0.0);
    auto pose = controller.getFKpose(zeros, 7, 7);
    std::printf("FK(zeros)    =");
    for (double value : pose) std::printf(" %.6f", value);
    std::printf("\n");

    const auto limits = controller.getConfig();
    std::printf("limits       = [%.4f, %.4f] rad for joint 1\n",
                limits.lower[0], limits.upper[0]);

    std::printf("state        = %d (0 = power_off, since OnRobot() has not run)\n",
                controller.GetRobotState());

    // ---------------------------------------------------------------- the mandatory call
    // OnRobot() switches the robot to "ready" and powers the low-level board. Without it,
    // getDof(), IK(), getJointerrcode() and setJointZeroPosition() dereference an
    // uninitialised state pointer and segfault. It returns false here because no CAN bus is
    // attached; the call still initialises the controller's internal state.
    //
    // Comment these three lines out if you want this program to stay power-off.
    std::printf("OnRobot()    = %d\n", static_cast<int>(controller.OnRobot()));
    std::printf("dof          = %d (joints per arm)\n", controller.getDof());
    std::printf("state        = %d (4 = fault: no CAN bus attached)\n",
                controller.GetRobotState());
    return 0;
}
