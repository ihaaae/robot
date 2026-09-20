// Probe the vendor's high-level SDK one method at a time.
//
// Includes only <juxie_controller/juxie_controller.h> (the public API, needs only Eigen),
// so it does NOT depend on the headers the vendor forgot to ship.
//
// Each run constructs the controller and calls exactly one method, so a crash in one
// method cannot hide the behaviour of the others.
//
// Build:
//   aarch64-linux-gnu-g++ -std=c++17 -O1 -I<sdk>/usr/include -I/usr/include/eigen3 \
//       sdk_probe.cpp -L<sdk>/usr/lib -Wl,-rpath-link,<sdk>/usr/lib \
//       -ljuxie_controller -o sdk_probe
#include <juxie_controller/juxie_controller.h>

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

namespace {

std::vector<double> kVec;

const char *kUsage =
    "usage: sdk_probe <method> [onrobot]\n"
    "  construct | getDof | state | joints | tcp | torques | config | fk | ik\n"
    "  fkvec <v...>   forward kinematics for the given joint vector\n"
    "  errcode | faulttype | axisfault | onrobot | clearfault | stop\n"
    "  enable | disable | zero | movej | moveend\n";

template <typename T, size_t N>
void print_array(const char *label, const std::array<T, N> &values) {
    std::printf("%s =", label);
    for (const auto &v : values) std::printf(" %.6f", static_cast<double>(v));
    std::printf("\n");
}

int run(const std::string &method, bool onrobot_first) {
    std::printf("[probe] constructing controller...\n");
    std::fflush(stdout);

    Juxie::ControllerJuxie controller;
    std::printf("[probe] constructed ok\n");
    std::fflush(stdout);

    if (onrobot_first) {
        std::printf("[probe] calling OnRobot() first\n");
        std::fflush(stdout);
        std::printf("onrobot = %d\n", static_cast<int>(controller.OnRobot()));
        std::fflush(stdout);
    }

    if (method == "construct") return 0;

    if (method == "getDof") {
        std::printf("[probe] calling getDof()\n"); std::fflush(stdout);
        std::printf("dof = %d\n", controller.getDof());
        return 0;
    }
    if (method == "state") {
        std::printf("[probe] calling GetRobotState()\n"); std::fflush(stdout);
        std::printf("state = %d\n", controller.GetRobotState());
        return 0;
    }
    if (method == "joints") {
        std::printf("[probe] calling GetJointPositions()\n"); std::fflush(stdout);
        print_array("joints", controller.GetJointPositions());
        return 0;
    }
    if (method == "tcp") {
        std::printf("[probe] calling GetTCPPose()\n"); std::fflush(stdout);
        print_array("tcp", controller.GetTCPPose());
        return 0;
    }
    if (method == "torques") {
        std::printf("[probe] calling GetSingleTorques()\n"); std::fflush(stdout);
        print_array("torques", controller.GetSingleTorques());
        return 0;
    }
    if (method == "config") {
        std::printf("[probe] calling getConfig()\n"); std::fflush(stdout);
        auto cfg = controller.getConfig();
        std::printf("upper[0] = %.6f  lower[0] = %.6f\n", cfg.upper[0], cfg.lower[0]);
        return 0;
    }
    if (method == "fk") {
        std::printf("[probe] calling getFKpose() with 7 values, left=7 right=0\n"); std::fflush(stdout);
        std::vector<double> left(7, 0.0);
        auto pose = controller.getFKpose(left, 7, 0);
        print_array("fk", pose);
        return 0;
    }
    if (method == "fk17") {
        // This is exactly what the vendor's own get_FK_pose handler does: a 17-element joint
        // vector with LeftNum=7, RightNum=7.
        //
        // It prints a plausible pose and then exits 0 -- and that is the trap. getFKpose
        // copies the input into a FIXED 14-element buffer, so 17 values overflow the heap.
        // Nothing is detected here because this process allocates nothing afterwards; in a
        // long-running process the next malloc aborts (that is the get_FK_pose crash).
        // 15, 16, 17 and 40 values all abort; 12, 13 and 14 are safe, regardless of
        // LeftNum / RightNum. Use "fk14" or "fkvec" with at most 14 values.
        std::printf("[probe] calling getFKpose() with 17 values, left=7 right=7  (OVERFLOWS, see comment)\n");
        std::fflush(stdout);
        std::vector<double> all(17, 0.0);
        auto pose = controller.getFKpose(all, 7, 7);
        print_array("fk17", pose);
        return 0;
    }
    if (method == "fk14") {
        std::printf("[probe] calling getFKpose() with 14 values, left=7 right=7\n"); std::fflush(stdout);
        std::vector<double> all(14, 0.0);
        auto pose = controller.getFKpose(all, 7, 7);
        print_array("fk14", pose);
        return 0;
    }
    if (method == "fkvec") {
        // Arbitrary joint vector, so the offline model can be validated directly against
        // the vendor's own FK instead of going through IK.
        std::printf("[probe] calling getFKpose() with a supplied joint vector\n");
        std::fflush(stdout);
        std::vector<double> all(kVec.begin(), kVec.end());
        // Match the vendor's own handler: LeftNum = RightNum = 7, so the first seven
        // values drive the left arm and the next seven the right arm.
        int left = 7;
        int right = 7;
        std::printf("left_num = %d  right_num = %d  size = %zu\n", left, right, all.size());
        auto pose = controller.getFKpose(all, left, right);
        print_array("fkvec", pose);
        return 0;
    }
    if (method == "ik") {
        std::printf("[probe] calling IK()\n"); std::fflush(stdout);
        Juxie::CartesianSpaceData target{};
        for (auto &v : target) v = Juxie::DEFAULT_INVALID_VALUE;
        target[0] = 0.2; target[1] = 0.0; target[2] = 0.5;
        target[3] = 1.0; target[4] = 0.0; target[5] = 0.0; target[6] = 0.0;
        Eigen::VectorXd joint;
        std::printf("ik ret = %d\n", controller.IK(target, joint));
        std::printf("ik joint size = %ld\n", static_cast<long>(joint.size()));
        return 0;
    }
    if (method == "errcode") {
        std::printf("[probe] calling getJointerrcode()\n"); std::fflush(stdout);
        auto codes = controller.getJointerrcode();
        std::printf("codes =");
        for (int i = 0; i < codes.size(); ++i) std::printf(" %u", codes[i]);
        std::printf("\n");
        return 0;
    }
    if (method == "faulttype") {
        std::printf("[probe] calling GetFaultType(1)\n"); std::fflush(stdout);
        std::printf("fault type = %s\n", controller.GetFaultType(1).c_str());
        return 0;
    }
    if (method == "axisfault") {
        std::printf("[probe] calling GetAxisFault(0x0001)\n"); std::fflush(stdout);
        std::printf("axis fault = %s\n", controller.GetAxisFault(0x0001).c_str());
        return 0;
    }
    if (method == "onrobot") {
        std::printf("[probe] calling OnRobot()\n"); std::fflush(stdout);
        std::printf("onrobot = %d\n", static_cast<int>(controller.OnRobot()));
        return 0;
    }
    if (method == "clearfault") {
        std::printf("[probe] calling ClearFault()\n"); std::fflush(stdout);
        std::printf("clearfault = %d\n", static_cast<int>(controller.ClearFault()));
        return 0;
    }
    if (method == "stop") {
        std::printf("[probe] calling Stop()\n"); std::fflush(stdout);
        std::printf("stop = %d\n", static_cast<int>(controller.Stop()));
        return 0;
    }
    if (method == "enable") {
        std::printf("[probe] calling EnableRobot()\n"); std::fflush(stdout);
        std::printf("enable = %d\n", static_cast<int>(controller.EnableRobot()));
        return 0;
    }
    if (method == "disable") {
        std::printf("[probe] calling DisableRobot()\n"); std::fflush(stdout);
        std::printf("disable = %d\n", static_cast<int>(controller.DisableRobot()));
        return 0;
    }
    if (method == "zero") {
        std::printf("[probe] calling setJointZeroPosition()\n"); std::fflush(stdout);
        std::printf("zero = %d\n", static_cast<int>(controller.setJointZeroPosition()));
        return 0;
    }
    if (method == "movej") {
        std::printf("[probe] calling MoveJ()\n"); std::fflush(stdout);
        Juxie::JointSpaceData target{};
        target.fill(0.0);
        std::printf("movej = %d\n", controller.MoveJ(target, 10));
        return 0;
    }
    if (method == "moveend") {
        std::printf("[probe] calling MoveEnd()\n"); std::fflush(stdout);
        std::printf("moveend = %d\n", controller.MoveEnd(10.0, 0));
        return 0;
    }

    std::fprintf(stderr, "unknown method: %s\n%s", method.c_str(), kUsage);
    return 2;
}

}  // namespace

int main(int argc, char **argv) {
    if (argc < 2) {
        std::fprintf(stderr, "%s", kUsage);
        return 2;
    }
    bool onrobot_first = false;
    for (int i = 2; i < argc; ++i) {
        if (std::strcmp(argv[i], "onrobot") == 0) {
            onrobot_first = true;
        } else {
            kVec.push_back(std::atof(argv[i]));
        }
    }
    std::printf("[probe] method = %s  onrobot_first = %d  extra_values = %zu\n",
                argv[1], onrobot_first ? 1 : 0, kVec.size());
    std::fflush(stdout);
    return run(argv[1], onrobot_first);
}
