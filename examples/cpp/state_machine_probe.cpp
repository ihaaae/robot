// Exercise the vendor's robot state machine (Juxie::State*) under emulation.
//
// docs/robot-state-machine.md reads the per-state behaviour out of the disassembly. This probe
// checks the parts of that reading that can be reached without a robot: the power_off and
// fault columns of the table, and two lifecycle defects.
//
//   state_machine_probe power_off   every gated method without OnRobot()
//   state_machine_probe fault       OnRobot() (lands in fault with no bus), then every gated method
//   state_machine_probe onrobot3    OnRobot() three times in a row
//   state_machine_probe offrobot    OnRobot() twice (starts the state thread), then OffRobot()
//   state_machine_probe offon       OnRobot(), OffRobot(), OnRobot(), then destroy the controller
//
// Emulator only. With no CAN bus OnRobot() ends in fault, where no command reaches the joints.
// On a real robot it can end in ready instead, and the calls below include EnableRobot() and
// motion commands, so every scenario after OnRobot() refuses to continue unless the state is
// fault.
//
// Build: ./examples/cpp/build.sh (builds every example here).
#include <juxie_controller/juxie_controller.h>

#include <chrono>
#include <cstdio>
#include <string>
#include <thread>
#include <vector>

namespace {

const char *kUsage = "usage: state_machine_probe power_off | fault | onrobot3 | offrobot | offon\n";

// Targets that are never reached: every call below is expected to be refused by the state.
const Juxie::JointSpaceData kJoints{};
const Juxie::CartesianSpaceData kPose{0.3, 0.2, 0.3, 1, 0, 0, 0, 0.3, -0.2, 0.3, 1, 0, 0, 0};
const std::vector<int> kAllJoints{1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14};

void report(Juxie::ControllerJuxie &c, const char *call, int result) {
    std::printf("%-14s -> %4d   state=%d\n", call, result, c.GetRobotState());
    std::fflush(stdout);
}

// The gated methods, in an order that never leaves the column being measured: the ones that
// can change state (ClearFault, OffRobot) come last.
void every_gated_method(Juxie::ControllerJuxie &c) {
    report(c, "EnableRobot", c.EnableRobot());
    report(c, "DisableRobot", c.DisableRobot());
    report(c, "Stop", c.Stop());
    report(c, "MoveJ", c.MoveJ(kJoints, 10));
    report(c, "MoveJ_P", c.MoveJ_P(kPose, 10));
    report(c, "MoveL", c.MoveL(kPose, 10));
    report(c, "MoveJ_Canfd", c.MoveJ_Canfd(kJoints, 50));
    report(c, "MoveP_Canfd", c.MoveP_Canfd(kPose, 50));
    report(c, "MoveEnd", c.MoveEnd(0.0, 0));
    report(c, "BreakEngage", c.BreakEngage(kAllJoints));
    report(c, "BreakRelease", c.BreakRelease(kAllJoints));
    report(c, "ClearFault", c.ClearFault());
    report(c, "OffRobot", c.OffRobot());
}

bool still_in_fault(Juxie::ControllerJuxie &c) {
    if (c.GetRobotState() == 4) return true;
    std::printf("state is %d, not fault: this is not the emulator. Stopping.\n",
                c.GetRobotState());
    return false;
}

int run(const std::string &scenario) {
    Juxie::ControllerJuxie c;
    report(c, "(constructed)", 0);

    if (scenario == "power_off") {
        every_gated_method(c);
        return 0;
    }

    report(c, "OnRobot", c.OnRobot());
    if (!still_in_fault(c)) return 1;

    if (scenario == "fault") {
        every_gated_method(c);
        return 0;
    }
    if (scenario == "onrobot3") {
        report(c, "OnRobot", c.OnRobot());
        report(c, "OnRobot", c.OnRobot());
        std::printf("survived three OnRobot() calls\n");
        return 0;
    }
    if (scenario == "offrobot") {
        report(c, "OnRobot", c.OnRobot());
        if (!still_in_fault(c)) return 1;
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        report(c, "OffRobot", c.OffRobot());
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        std::printf("survived 500 ms after OffRobot()\n");
        return 0;
    }
    if (scenario == "offon") {
        report(c, "OffRobot", c.OffRobot());
        report(c, "OnRobot", c.OnRobot());
        if (!still_in_fault(c)) return 1;
        std::printf("destroying the controller\n");
        std::fflush(stdout);
        return 0;   // the destructor runs here
    }
    std::fputs(kUsage, stderr);
    return 2;
}

}  // namespace

int main(int argc, char **argv) {
    if (argc != 2) {
        std::fputs(kUsage, stderr);
        return 2;
    }
    const int status = run(argv[1]);
    std::printf("controller destroyed\n");
    return status;
}
