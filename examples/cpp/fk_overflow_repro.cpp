// Reproduction for the getFKpose buffer overflow. THIS PROGRAM IS MEANT TO FAIL.
//
// It is not a demo and not an example. It exists so the claim in docs/sdk.md and
// docs/sdk-usage.md §6.1 can be re-verified by anyone holding the SDK, instead of being
// taken on trust.
//
//   ./fk_overflow_repro 14    -> clean (no IK call, so nothing else can fail)
//   ./fk_overflow_repro 17    -> aborts with "free(): invalid next size (fast)"
//
// What actually goes wrong
// ------------------------
// getFKpose allocates a fixed 7-double (56-byte) buffer and memmoves (n - 7) doubles into
// it, where n is the element count of the vector you pass. So it is in bounds only when
// n - 7 <= 7, i.e. **n <= 14**. LeftNum and RightNum do not affect the buffer size.
//
// AddressSanitizer output for n=17, LeftNum=7, RightNum=7:
//
//   WRITE of size 80 at ... 0 bytes to the right of 56-byte region
//     #1 Juxie::State::getFKpose(std::vector<double> const&, int, int)
//   allocated by ... operator new inside Juxie::State::getFKpose
//
// 80 bytes = (17 - 7) doubles, written into a 56-byte region. Confirmed for n=15 (64 bytes)
// and n=20 (104 bytes), and for LeftNum values of 3, 5, 7, 9, 10 and 11.
//
// How to get a reliable answer
// ---------------------------
// The plain build below aborts only when the allocator happens to notice, which depends on
// the binary and on what ran before the call -- it can also corrupt silently. For a
// deterministic, layout-independent check, build with AddressSanitizer:
//
//   aarch64-linux-gnu-g++ -std=c++17 -O1 -fsanitize=address -static-libasan \
//       -I<sdk>/usr/include -I/usr/include/eigen3 fk_overflow_repro.cpp \
//       -L<sdk>/usr/lib -Wl,-rpath-link,<sdk>/usr/lib -ljuxie_controller -o fk_overflow_repro
//
// Under ASan, n<=14 is clean and n>=15 reports the heap-buffer-overflow WRITE every time.
//
// Note why the n<=14 case skips the IK call: IK has its own, unrelated ASan finding (a
// heap-buffer-overflow READ of 8 bytes in ~Eigen::DenseStorage, see docs/sdk.md), so calling
// it in the control case would report that instead and make the control look dirty.
//
// Build (plain): ./examples/cpp/build.sh
#include <juxie_controller/juxie_controller.h>

#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>

int main(int argc, char **argv) {
    const int count = argc > 1 ? std::atoi(argv[1]) : 17;
    const int left = argc > 2 ? std::atoi(argv[2]) : 7;
    const int right = argc > 3 ? std::atoi(argv[3]) : 7;

    Juxie::ControllerJuxie controller;
    controller.OnRobot();

    // The overflow itself: it returns a plausible pose and reports no error.
    std::vector<double> joints_in(count, 0.0);
    auto pose = controller.getFKpose(joints_in, left, right);

    // Where the damage surfaces: the next allocation. Only call IK when the count actually
    // overflows -- IK carries its own unrelated ASan finding, so calling it in the control
    // case would report that instead.
    const bool overflows = count > 14;
    int ret = 0;
    if (overflows) {
        Juxie::CartesianSpaceData target{};
        for (auto &v : target) v = 0.0;
        target[0] = 0.20; target[3] = 1.0;
        target[7] = 0.20; target[10] = 1.0;
        Eigen::VectorXd joints_out;
        ret = controller.IK(target, joints_out);
    }

    std::printf("count=%d left=%d right=%d  ->  survived\n", count, left, right);
    std::printf("  getFKpose z=%.6f, IK %s\n", pose[2],
                overflows ? std::to_string(ret).c_str() : "(not called, count <= 14)");
    std::printf("  getFKpose copies (n - 7) doubles into a 7-double buffer, so n=%d writes %d.\n",
                count, count - 7);
    std::printf("  A clean exit does NOT mean the call was safe -- it may just mean the\n");
    std::printf("  allocator did not notice. Rebuild with -fsanitize=address to see it.\n");
    return 0;
}
