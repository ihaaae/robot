#!/usr/bin/env bash
# Build the C++ examples against the SDK tree committed in this repository.
#
#   ./examples/cpp/build.sh                 # uses ./vendor/sdk/dual-arm-app/0.6.4
#   SDK=/path/to/another/sdk ./examples/cpp/build.sh
#
# Requires an aarch64 cross toolchain and Eigen headers:
#   sudo apt-get install -y g++-aarch64-linux-gnu libeigen3-dev
set -euo pipefail

here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
repo="$(cd "$here/../.." && pwd)"
SDK="${SDK:-$repo/vendor/sdk/dual-arm-app/0.6.4}"
OUT="${OUT:-$repo/.sdk/bin}"   # build output is scratch, not committed

if [[ ! -d "$SDK/usr/include/juxie_controller" ]]; then
    echo "error: no SDK at $SDK" >&2
    echo "The tree is committed, so this usually means the checkout is incomplete." >&2
    echo "Restore it with 'git checkout -- vendor/sdk'." >&2
    exit 2
fi

CXX="${CXX:-aarch64-linux-gnu-g++}"
command -v "$CXX" >/dev/null || { echo "error: $CXX not found" >&2; exit 2; }

eigen="${EIGEN_INCLUDE:-/usr/include/eigen3}"
[[ -d "$eigen/Eigen" ]] || { echo "error: Eigen headers not found at $eigen" >&2; exit 2; }

mkdir -p "$OUT"
for src in "$here"/*.cpp; do
    name="$(basename "$src" .cpp)"
    echo "building $name"
    # -Wl,-rpath-link is required: libjuxie_controller.so pulls in libexecutor,
    # libbot_servo, libbot_planner, libbot_traj_planner and libbot_kinematics, and the
    # linker cannot find those transitively without it. The resulting "undefined
    # reference" errors look like a missing-header problem but are not.
    "$CXX" -std=c++17 -O1 \
        -I"$SDK/usr/include" -I"$eigen" \
        "$src" \
        -L"$SDK/usr/lib" -Wl,-rpath-link,"$SDK/usr/lib" \
        -ljuxie_controller -o "$OUT/$name"
done

echo
echo "built into $OUT"
echo "run under emulation with:"
echo "  DUAL_ARM_SDK_CONFIG=$SDK/usr/etc LD_LIBRARY_PATH=$SDK/usr/lib \\"
echo "  qemu-aarch64-static -L <sysroot> $OUT/01_offline_kinematics"
echo
echo "01_offline_kinematics, sdk_probe, state_machine_probe and fk_overflow_repro run with no robot attached."
echo "02, 04, 06 and 07 need a robot on the CAN bus."
echo "04, 06 and 07 are dry runs unless you pass --yes."
