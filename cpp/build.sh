#!/usr/bin/env bash
# Build and run the L0 tests (wire codec + transport seam).
#
#   ./cpp/build.sh                 # cmake when available, otherwise a direct g++ build
#   OUT=/tmp/l0 ./cpp/build.sh
#   CXX=g++-13 ./cpp/build.sh
#
# No dependencies beyond a C++17 compiler. Everything here is offline: no bus, no robot, no
# vendor SDK, no network. That is the point -- L0 is the one layer that can be finished and
# verified before the robot arrives.
set -euo pipefail

here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
repo="$(cd "$here/.." && pwd)"
OUT="${OUT:-$repo/.l0}"

if command -v cmake >/dev/null 2>&1; then
    echo "configuring with cmake into $OUT"
    cmake -S "$here" -B "$OUT" -DCMAKE_BUILD_TYPE=RelWithDebInfo >/dev/null
    cmake --build "$OUT" --parallel
    echo
    "$OUT/shensi_can_tests"
    exit 0
fi

CXX="${CXX:-c++}"
command -v "$CXX" >/dev/null || {
    echo "error: neither cmake nor a C++ compiler ($CXX) is available" >&2
    exit 2
}

echo "no cmake; building directly with $CXX into $OUT/direct"
mkdir -p "$OUT/direct"
# shellcheck disable=SC2046  # the globs are meant to expand into the source list
"$CXX" -std=c++17 -O1 -Wall -Wextra -Wpedantic \
    -I"$here/include" -I"$here/tests" \
    "$here"/src/*.cpp "$here"/tests/*.cpp \
    -o "$OUT/direct/shensi_can_tests"
echo
"$OUT/direct/shensi_can_tests"
