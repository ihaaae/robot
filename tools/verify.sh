#!/usr/bin/env bash
# Everything that can be checked without a robot, in one command.
#
# This is the executable form of the repository's acceptance criterion: a fresh clone can
# verify that the vendor originals are unmodified, that the committed SDK tree is complete
# and linkable, that the Python package carries no vendor data and its tests pass, and that
# the L0 wire/transport/trace layer reproduces the protocol document's own frames.
#
#   ./tools/verify.sh                 # offline checks only
#   ./tools/verify.sh --with-native   # also cross-compile the C++ examples (needs a toolchain)
#
# Never opens a socket, never touches a robot. Step 6 needs a host C++ compiler and nothing
# else; it is skipped, not failed, when there is none.
set -euo pipefail

repo="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$repo"
with_native=false
[[ "${1:-}" == "--with-native" ]] && with_native=true

pass() { printf '  \033[32mok\033[0m   %s\n' "$1"; }
fail() { printf '  \033[31mFAIL\033[0m %s\n' "$1"; exit 1; }

work="$(mktemp -d "${TMPDIR:-/tmp}/shensi-verify-XXXXXX")"
trap 'rm -rf "$work"' EXIT

echo "1. vendor sources of truth"
deb="$(ls vendor/originals/dual-arm-app/*/*.deb 2>/dev/null | head -1 || true)"
[[ -n "$deb" ]] || fail "no .deb under vendor/originals/"
# Every recorded original, not just the .deb: the three vendor documents are sources of
# truth too, and a document that silently changed is just as misleading as a changed binary.
python3 - "$deb" <<'PYHASH' || fail "an original does not match its recorded sha256"
import hashlib, json, pathlib, sys
repo = pathlib.Path(".")
manifest = json.loads((repo / "vendor/manifests/dual-arm-app-0.6.4.json").read_text())
bad, checked = [], 0
for artifact in manifest["artifacts"]:
    expected = artifact.get("sha256")
    if not expected:
        bad.append(f"{artifact.get('stored_name')}: no sha256 recorded")
        continue
    matches = list((repo / "vendor/originals").rglob(artifact["stored_name"]))
    if len(matches) != 1:
        bad.append(f"{artifact['stored_name']}: found {len(matches)} copies, expected 1")
        continue
    path = matches[0]
    digest = hashlib.sha256(path.read_bytes()).hexdigest()
    checked += 1
    if digest != expected:
        bad.append(f"{artifact['stored_name']}: {digest} != {expected}")
if bad:
    print("  \033[31mFAIL\033[0m " + "; ".join(bad))
    sys.exit(1)
print(f"  \033[32mok\033[0m   {checked} originals match their recorded sha256")
PYHASH

echo "2. the vendored SDK tree is present and usable"
# The .deb is kept as provenance evidence, not as something the repository derives from, so
# there is no extraction step and nothing to diff against. What matters for an SDK is that
# the committed tree can actually be compiled and linked against.
SDK="vendor/sdk/dual-arm-app/0.6.4"
python3 - "$SDK" <<'PYTREE' || fail "the vendored SDK tree is incomplete"
import os, pathlib, sys

sdk = pathlib.Path(sys.argv[1])
problems = []

# Resolve each library through its symlink chain. `Path.exists()` follows links, so a
# dangling chain -- the .so and .so.3 present but the real .so.0.6.4 gone -- is caught. That
# matters because -ljuxie_controller only works when the chain resolves.
# The SDK's whole run-time closure, not a sample: a missing one of these links fine on a
# developer machine that happens to have it elsewhere and fails on the robot.
for name in ("libjuxie_controller", "libexecutor", "libbot_servo", "libbot_planner",
             "libbot_traj_planner", "libbot_path_planner", "libbot_kinematics",
             "libbot_validator", "libbot_utils", "librk3576_can_canfd"):
    entry = sdk / "usr/lib" / f"{name}.so"
    if not entry.exists():
        problems.append(f"usr/lib/{name}.so does not resolve to a file")
        continue
    if not entry.is_symlink():
        problems.append(f"usr/lib/{name}.so is not a symlink")
    real = entry.resolve()
    if real.stat().st_size < 4096:
        problems.append(f"{real.name} is only {real.stat().st_size} bytes")

if not (sdk / "usr/include/juxie_controller/juxie_controller.h").is_file():
    problems.append("usr/include/juxie_controller/juxie_controller.h is missing")
if not (sdk / "usr/etc/juxie_73/kinematics_leftArm.yml").is_file():
    problems.append("usr/etc/juxie_73/kinematics_leftArm.yml is missing")

# Every real library starts with the ELF magic. That catches a truncated file, a placeholder,
# and the wrong file, all at once -- a bare size check does not.
for lib in (sdk / "usr/lib").glob("*.so.0.6.4"):
    if lib.read_bytes()[:4] != b"\x7fELF":
        problems.append(f"{lib} is not an ELF shared object")

if problems:
    print("  \033[31mFAIL\033[0m " + "; ".join(problems))
    sys.exit(1)
print(f"  \033[32mok\033[0m   libraries, symlinks, headers and config are all in place")
PYTREE

echo "3. Python package: no vendor data bundled, tests pass offline"
# Use the checkout's own package, not whatever happens to be installed: otherwise the tests
# could pass against an unrelated copy.
if python3 -c "
import pathlib, sys
try:
    import shensi_robot
except ImportError:
    sys.exit(1)
here = pathlib.Path('src/shensi_robot').resolve()
sys.exit(0 if pathlib.Path(shensi_robot.__file__).resolve().parent == here else 1)
" 2>/dev/null; then
    runner=python3
else
    # Build the throwaway venv outside the repository so it can never be committed.
    python3 -m venv "$work/venv" >/dev/null 2>&1
    "$work/venv/bin/pip" install -q -e ".[dev]" 2>/dev/null
    runner="$work/venv/bin/python"
fi
"$runner" -m pytest -q >/dev/null 2>&1 || fail "unit tests"
pass "$("$runner" -m pytest -q 2>&1 | tail -1)"
"$runner" - <<'PYCHECK' || fail "vendor configuration is bundled in the package"
import pathlib, sys
import shensi_robot
pkg = pathlib.Path(shensi_robot.__file__).parent
# Any vendor payload would show up as data or binary files under the package, not just YAML.
allowed = {".py", ".pyi", ".typed"}
bundled = [
    p for p in pkg.rglob("*")
    if p.is_file()
    and p.suffix not in allowed
    and "__pycache__" not in p.parts  # build output, not package data
]
if bundled:
    print(f"  \033[31mFAIL\033[0m non-source files bundled in the package: {bundled}")
    sys.exit(1)
print("  \033[32mok\033[0m   package contains source only, no vendor data")
PYCHECK

echo "4. offline kinematics produces the recorded zero pose"
DUAL_ARM_SDK_CONFIG="$SDK/usr/etc" "$runner" - <<'PYCHECK' || fail "zero pose check"
import sys
import shensi_robot as S
arm = S.load_arm("left")
z = arm.fk_pose([0.0] * 7, "zyx", tcp_offset=0.0)[2]
# get_tcp_pose reports exactly the yml's M at the zero configuration.
if abs(z - 0.6752) > 1e-6:
    print(f"  \033[31mFAIL\033[0m zero pose z={z}, expected 0.6752")
    sys.exit(1)
print(f"  \033[32mok\033[0m   zero pose z={z:.6f} (matches get_tcp_pose)")
PYCHECK

echo "5. the offline demo runs against the vendored SDK"
DUAL_ARM_SDK_CONFIG="$SDK/usr/etc" "$runner" examples/python/01_offline_kinematics.py \
    >/dev/null || fail "examples/python/01_offline_kinematics.py"
pass "01_offline_kinematics.py"

echo "6. L0 wire, transport and trace tests (offline, host C++ compiler)"
if command -v c++ >/dev/null 2>&1 || command -v g++ >/dev/null 2>&1; then
    if ! l0_output="$(./cpp/build.sh 2>&1)"; then
        printf '%s\n' "$l0_output" >&2
        fail "cpp/build.sh"
    fi
    pass "$(printf '%s\n' "$l0_output" | tail -1)"
else
    printf '  \033[33mskip\033[0m no host C++ compiler, L0 tests not run\n'
fi

if $with_native; then
    echo "7. C++ demos and the Python bridge cross-compile against the SDK"
    if command -v aarch64-linux-gnu-g++ >/dev/null; then
        ./examples/cpp/build.sh >/dev/null 2>&1 || fail "examples/cpp/build.sh"
        # Build only. Running them needs an arm64 sysroot and qemu; see docs/sdk-usage.md.
        for prog in 01_offline_kinematics 02_read_telemetry 04_guarded_motion \
                    06_vendor_cyclic_motion 07_replay_trajectory \
                    sdk_min_example sdk_probe fk_overflow_repro; do
            [[ -x ".sdk/bin/$prog" ]] || fail "examples/cpp/$prog did not build"
        done
        pass "8 programs built (5 demos + min example + probe + overflow repro)"

        # The Python path needs its own aarch64 artifact. Build only, like the demos: running
        # it needs an arm64 interpreter and qemu, which is the manual bench in docs/sdk.md §9.
        ./python/build_bridge.sh >/dev/null 2>&1 || fail "python/build_bridge.sh"
        [[ -f "python/build/juxie_sdk_bridge.so" ]] \
            || fail "python/build/juxie_sdk_bridge.so was not produced"
        pass "the Python bridge built (python/build/juxie_sdk_bridge.so)"
    else
        fail "aarch64-linux-gnu-g++ is not installed, but --with-native was requested"
    fi

    # The consumer path, which is the one that matters to anyone writing their own program:
    # a project outside this repository, including cmake/juxie-sdk.cmake and linking
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
include("$repo/cmake/juxie-sdk.cmake")
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
fi

echo
echo "all checks passed"
