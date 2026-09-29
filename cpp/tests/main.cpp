#include <cstdio>

#include "check.hpp"

int main() {
    for (const auto& test_case : shensi::test::cases()) {
        std::printf("  %s\n", test_case.name);
        std::fflush(stdout);
        test_case.fn();
    }
    const int failures = shensi::test::failures();
    std::printf("\n%d checks, %d failures\n", shensi::test::checks(), failures);
    return failures == 0 ? 0 : 1;
}
