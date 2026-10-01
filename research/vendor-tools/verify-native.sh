#!/usr/bin/env bash
# Frozen vendor tooling: build-checks for everything that links the vendor's binary SDK.
#
# Split out of tools/verify.sh (its old --with-native step) when the vendor SDK was demoted
# from a compatibility target to reference material. Nothing here is maintained beyond
# keeping it runnable; see README.md next to this script.
#
#   ./research/vendor-tools/verify-native.sh
#
# Runs the frozen module's offline tests, cross-compiles the C++ programs and the Python
# bridge, and builds a consumer project against Juxie::SDK. Needs aarch64-linux-gnu-g++;
# CMake and readelf are optional. Runs nothing against the vendor binaries.
set -euo pipefail

repo="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$repo"

pass() { printf '  \033[32mok\033[0m   %s\n' "$1"; }
fail() { printf '  \033[31mFAIL\033[0m %s\n' "$1"; exit 1; }

work="$(mktemp -d "${TMPDIR:-/tmp}/shensi-verify-native-XXXXXX")"
trap 'rm -rf "$work"' EXIT

echo "1. the frozen Python module's offline tests"
runner=python3
if ! python3 -c "import pytest, numpy, yaml" 2>/dev/null; then
    # Same fallback as tools/verify.sh: a throwaway venv outside the repository.
    python3 -m venv "$work/venv" >/dev/null 2>&1
    "$work/venv/bin/pip" install -q -e ".[dev]" 2>/dev/null
    runner="$work/venv/bin/python"
fi
PYTHONPATH="$repo/research/vendor-tools/python:$repo/src${PYTHONPATH:+:$PYTHONPATH}" \
    "$runner" -m pytest -q -p no:cacheprovider research/vendor-tools/python/test_juxie_sdk.py >/dev/null 2>&1 \
    || fail "research/vendor-tools/python/test_juxie_sdk.py"
pass "test_juxie_sdk.py"

echo "2. C++ demos and the Python bridge cross-compile against the SDK"
if command -v aarch64-linux-gnu-g++ >/dev/null; then
    ./research/vendor-tools/cpp/build.sh >/dev/null 2>&1 || fail "research/vendor-tools/cpp/build.sh"
    # Build only. Running them needs an arm64 sysroot and qemu; see research/vendor-analysis/sdk-usage.md.
    for prog in 01_offline_kinematics 02_read_telemetry 04_guarded_motion \
                06_vendor_cyclic_motion 07_replay_trajectory \
                sdk_min_example sdk_probe fk_overflow_repro state_machine_probe; do
        [[ -x ".sdk/bin/$prog" ]] || fail "research/vendor-tools/cpp/$prog did not build"
    done
    pass "9 programs built (5 demos + min example + two probes + overflow repro)"

    # The Python path needs its own aarch64 artifact. Build only, like the demos: running
    # it needs an arm64 interpreter and qemu, which is the manual bench in research/vendor-analysis/sdk.md §9.
    ./research/vendor-tools/python/build_bridge.sh >/dev/null 2>&1 || fail "research/vendor-tools/python/build_bridge.sh"
    [[ -f "research/vendor-tools/python/build/juxie_sdk_bridge.so" ]] \
        || fail "research/vendor-tools/python/build/juxie_sdk_bridge.so was not produced"
    pass "the Python bridge built (research/vendor-tools/python/build/juxie_sdk_bridge.so)"
else
    fail "aarch64-linux-gnu-g++ is not installed, and these checks need it"
fi

echo "3. a consumer project builds against Juxie::SDK"
# The consumer path, which is the one that matters to anyone writing their own program:
# a project outside this repository, including research/vendor-tools/cmake/juxie-sdk.cmake and linking
# Juxie::SDK, with no hand-written flags. Skipped only if CMake is absent.
if command -v cmake >/dev/null; then
    consumer="$work/consumer"
    mkdir -p "$consumer"
    cat > "$consumer/main.cpp" <<'CPP'
#include <juxie_controller/juxie_controller.h>
#include <cstdio>
#include <vector>
int main() {
Juxie::ControllerJuxie controller;
std::vector<double> zeros(14, 0.0);          // 14 values, 7 + 7: the tested shape
auto pose = controller.getFKpose(zeros, 7, 7);
std::printf("%.5f\n", pose[2]);
return 0;
}
CPP
    cat > "$consumer/CMakeLists.txt" <<CMAKE
cmake_minimum_required(VERSION 3.16)
project(consumer CXX)
set(CMAKE_CXX_STANDARD 17)
include("$repo/research/vendor-tools/cmake/juxie-sdk.cmake")
add_executable(my_app main.cpp)
target_link_libraries(my_app PRIVATE Juxie::SDK)
CMAKE
    cmake -S "$consumer" -B "$consumer/build" >/dev/null 2>&1 \
        -DCMAKE_CXX_COMPILER=aarch64-linux-gnu-g++ \
        -DCMAKE_SYSTEM_NAME=Linux -DCMAKE_SYSTEM_PROCESSOR=aarch64 \
        || fail "a consumer project could not configure against Juxie::SDK"
    cmake --build "$consumer/build" >/dev/null 2>&1 \
        || fail "a consumer project could not link against Juxie::SDK"
    pass "a project outside the repo builds against Juxie::SDK via CMake"

    # Linking and loading are two different things. The five libraries libjuxie_controller
    # pulls in carry no RPATH of their own, so the executable must carry one -- and it has
    # to be DT_RPATH, because DT_RUNPATH is not searched for transitive dependencies. A
    # RUNPATH here links fine and then dies at startup with
    # "libexecutor.so.3: cannot open shared object file".
    if command -v aarch64-linux-gnu-readelf >/dev/null; then
        aarch64-linux-gnu-readelf -d "$consumer/build/my_app" | grep -q '(RPATH)' \
            || fail "the consumer binary carries no RPATH; it would not find libexecutor at run time"
        pass "the consumer binary carries DT_RPATH, so transitive dependencies resolve"
    else
        printf '  \033[33mskip\033[0m aarch64-linux-gnu-readelf not installed, run-time tag not checked\n'
    fi
else
    printf '  \033[33mskip\033[0m cmake not installed, consumer integration not checked\n'
fi

echo
echo "all native checks passed"
