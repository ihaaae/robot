#!/usr/bin/env bash
# Build the C ABI bridge that lets Python call the SDK through ctypes.
#
#   ./python/build_bridge.sh              # figures out the rest; see below
#   ./python/build_bridge.sh --help
#
# It works on both machines, and picks the right one by itself:
#
#   on the robot's board (aarch64)  -> native build with the board's own g++
#   on an x86_64 development machine -> cross build with aarch64-linux-gnu-g++
#
# Everything can be overridden: CXX, SDK, EIGEN_INCLUDE, OUT. The SDK root is looked for in
# this order: $SDK, the tree committed in this repository, a deployed /opt/juxie, /usr, and /
# (a candidate is any directory that contains usr/lib/libjuxie_controller.so, and the vendor's
# .deb installs into /usr, so there the root is /).
#
# The result is aarch64 Linux, like the SDK. Building it on x86 produces a file the host
# interpreter cannot load -- that is expected, it is for the board.
set -euo pipefail

if [[ "${1:-}" == "--help" || "${1:-}" == "-h" ]]; then
    sed -n '2,17p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'
    exit 0
fi

here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
repo="$(cd "$here/.." && pwd)"

# ---------------------------------------------------------------- which compiler, and why
host="$(uname -m)"
if [[ -n "${CXX:-}" ]]; then
    cxx="$CXX"
    mode="as requested (CXX=$CXX)"
elif [[ "$host" == "aarch64" || "$host" == "arm64" ]]; then
    cxx="g++"
    mode="native ($host)"
else
    cxx="aarch64-linux-gnu-g++"
    mode="cross ($host -> aarch64)"
fi

# ---------------------------------------------------------------------------- the SDK tree
find_sdk() {
    local candidate
    for candidate in "${SDK:-}" \
                     "$repo/vendor/sdk/dual-arm-app/0.6.4" \
                     /opt/juxie \
                     /usr \
                     /; do
        [[ -n "$candidate" ]] || continue
        if [[ -f "$candidate/usr/lib/libjuxie_controller.so" ]]; then
            printf '%s\n' "$candidate"
            return 0
        fi
    done
    return 1
}
if ! sdk="$(find_sdk)"; then
    echo "error: no SDK tree found. Looked at \$SDK, $repo/vendor/sdk/dual-arm-app/0.6.4," >&2
    echo "       /opt/juxie and /usr. Set SDK=/path/to/sdk (the directory holding usr/)." >&2
    exit 2
fi
# Absolute, always: a relative $SDK would put a relative path in the RPATH and in the sidecar,
# and a relative RPATH is resolved against whatever the working directory happens to be.
sdk="$(cd "$sdk" && pwd)"

if [[ "$sdk" == "$repo/vendor/sdk/dual-arm-app/0.6.4" ]] && \
   [[ ! -d "$sdk/usr/include/juxie_controller" ]]; then
    echo "error: the committed SDK tree is incomplete. Restore it with" >&2
    echo "       'git checkout -- vendor/sdk'." >&2
    exit 2
fi

# ----------------------------------------------------------------------------------- Eigen
eigen="${EIGEN_INCLUDE:-}"
if [[ -z "$eigen" ]]; then
    for candidate in /usr/include/eigen3 /usr/local/include/eigen3 "$sdk/usr/include/eigen3"; do
        if [[ -d "$candidate/Eigen" ]]; then eigen="$candidate"; break; fi
    done
fi
if [[ -z "$eigen" || ! -d "$eigen/Eigen" ]]; then
    echo "error: Eigen headers not found. Install them (apt-get install libeigen3-dev)" >&2
    echo "       or set EIGEN_INCLUDE=/path/to/eigen3." >&2
    exit 2
fi

command -v "$cxx" >/dev/null || {
    echo "error: $cxx not found." >&2
    if [[ "$cxx" == "aarch64-linux-gnu-g++" ]]; then
        echo "       On a Debian/Ubuntu host: apt-get install -y g++-aarch64-linux-gnu" >&2
    else
        echo "       Install a C++ compiler, or run this on the robot's board." >&2
    fi
    exit 2
}

# ---------------------------------------------------------------------------------- output
if [[ -n "${OUT:-}" ]]; then
    out="$OUT"
elif [[ -w "$here" ]]; then
    out="$here/build"            # inside the checkout: scratch, and git-ignored
else
    out="${XDG_CACHE_HOME:-$HOME/.cache}/shensi_robot"
fi
mkdir -p "$out"

# -Wl,-rpath-link is needed at link time and -Wl,--disable-new-dtags at run time, for the same
# five libraries libjuxie_controller pulls in; see the README section "Writing your own
# program" for why the second one matters. The rpath records wherever this tree is right now,
# so a binary built on x86 only runs on the board if the tree is there too, or if
# LD_LIBRARY_PATH points at it.
echo "mode  : $mode"
echo "SDK   : $sdk"
echo "eigen : $eigen"
if ! "$cxx" -std=c++17 -O2 -fPIC -shared \
        -I"$sdk/usr/include" -I"$eigen" \
        "$here/juxie_sdk_bridge.cpp" \
        -L"$sdk/usr/lib" -Wl,-rpath-link,"$sdk/usr/lib" \
        -Wl,--disable-new-dtags -Wl,-rpath,"$sdk/usr/lib" \
        -ljuxie_controller -o "$out/juxie_sdk_bridge.so"; then
    echo "error: the build failed." >&2
    echo "       A link error about libjuxie_controller usually means the compiler and the" >&2
    echo "       SDK tree disagree about architecture: build on the robot's board, or" >&2
    echo "       cross-compile with aarch64-linux-gnu-g++." >&2
    exit 1
fi

# Record which tree this bridge was built against. The module reads it to set
# DUAL_ARM_SDK_CONFIG, so that the library and its configuration always come from the same
# tree instead of the module guessing at runtime.
printf '%s\n' "$sdk" > "$out/juxie_sdk_bridge.sdk"

echo "wrote : $out/juxie_sdk_bridge.so"
echo "        $out/juxie_sdk_bridge.sdk (the tree above, for the module to find its config)"
echo
echo "next  : python3 examples/python/sdk_min_example.py"
echo "        (the module finds both files by itself; set JUXIE_SDK_BRIDGE to use another)"
