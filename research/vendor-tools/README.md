# Vendor tooling (frozen)

Everything in this directory links or loads the vendor's binary SDK
(`vendor/sdk/dual-arm-app/0.6.4`, `Juxie::ControllerJuxie`). It was written while this
repository aimed to be detail-compatible with that SDK. That goal is gone: we now match the
vendor SDK only at the level of abstraction, and the binary is reference material. These tools
stay because they are how the findings in [`../vendor-analysis/`](../vendor-analysis) were
obtained and can be reproduced. They are **frozen**: kept runnable, not extended. New code goes
in `cpp/` and does not depend on anything here.

| Path | What it is |
|---|---|
| `cpp/` | C++ demos and probes against `Juxie::ControllerJuxie`; `cpp/build.sh` cross-compiles them into `.sdk/bin/` |
| `cmake/juxie-sdk.cmake` | the `Juxie::SDK` imported target, for a CMake project linking the vendor SDK |
| `python/` | C ABI bridge (`juxie_sdk_bridge.cpp`, `build_bridge.sh`), its ctypes module `juxie_sdk.py`, an example, and offline tests |
| `python/vendor_model/` | pure-Python model of the vendor's kinematics, rebuilt from its YAML (FK, numerical IK, a CLI, an example, tests). Never validated; see [`../vendor-analysis/kinematics.md`](../vendor-analysis/kinematics.md) |
| `probes/` | disassembly excerpts and an FK cross-check used by the vendor analysis |
| `verify-native.sh` | offline Python tests, plus build-checks for all of the above |

```bash
./research/vendor-tools/verify-native.sh            # needs aarch64-linux-gnu-g++
./research/vendor-tools/python/build_bridge.sh
python3 research/vendor-tools/python/sdk_min_example.py   # on the board or under qemu
```

Nothing here is an installable package. Put `research/vendor-tools/python` on `PYTHONPATH`
to import `juxie_sdk` or `vendor_model` (`juxie_sdk` uses `vendor_model.config`); they need
numpy and PyYAML:

```bash
export PYTHONPATH="$PWD/research/vendor-tools/python"
export DUAL_ARM_SDK_CONFIG="$PWD/vendor/sdk/dual-arm-app/0.6.4/usr/etc"
python3 -m vendor_model.cli --arm left fk --joints "0 0 0 0 0 0 0"
python3 -m vendor_model.example_offline_kinematics
```

 How to run any of this under emulation:
[`../vendor-analysis/sdk.md`](../vendor-analysis/sdk.md) §9 and
[`../vendor-analysis/sdk-usage.md`](../vendor-analysis/sdk-usage.md).

The rest of this file is the usage guide for the vendor SDK that used to be the repository's
top-level README. It is kept as written, apart from paths.

## Demos and probes

`cpp/` — five runnable demos, two diagnostic programs and one bug reproduction,
all linking the vendor SDK through its public header only. The numbering is historical.

| Program | What it shows | Needs |
|---|---|---|
| [`01_offline_kinematics.cpp`](cpp/01_offline_kinematics.cpp) | FK, IK, the joint limits the controller reports, the `-100` sentinel, and the `getFKpose` limit | only the SDK |
| [`02_read_telemetry.cpp`](cpp/02_read_telemetry.cpp) | state, the 17-slot joints decoded, TCP pose, torques, the three error encodings | a robot on the CAN bus |
| [`04_guarded_motion.cpp`](cpp/04_guarded_motion.cpp) | enable → move → disable, with preflight checks. **Dry run by default** | a robot on the CAN bus |
| [`06_vendor_cyclic_motion.cpp`](cpp/06_vendor_cyclic_motion.cpp) | the vendor's built-in cyclic motion, run through the SDK instead of the node's command. **Dry run by default** | a robot on the CAN bus |
| [`07_replay_trajectory.cpp`](cpp/07_replay_trajectory.cpp) | replays the vendor's recorded CSV through the 50 Hz streaming interface, the way the vendor's own `MoveJCanfdTest` does. **Dry run by default** | a robot on the CAN bus |
| [`sdk_min_example.cpp`](cpp/sdk_min_example.cpp) | the smallest program that links the SDK at all | only the SDK |
| [`sdk_probe.cpp`](cpp/sdk_probe.cpp) | one SDK method per run, so a crash in one cannot hide the others | only the SDK |
| [`state_machine_probe.cpp`](cpp/state_machine_probe.cpp) | the vendor state machine's `power_off` and `fault` columns, and the three lifecycle crashes (`research/vendor-analysis/robot-state-machine.md`). **Emulator only** — refuses to continue unless `OnRobot()` lands in `fault` | only the SDK |
| [`fk_overflow_repro.cpp`](cpp/fk_overflow_repro.cpp) | **a bug reproduction, not a demo** — it is meant to fail. Proves the `getFKpose` overflow under AddressSanitizer | only the SDK |

```bash
./research/vendor-tools/cpp/build.sh
# 01 and the probe run under emulation with no robot:
DUAL_ARM_SDK_CONFIG=$SDK/usr/etc LD_LIBRARY_PATH=$SDK/usr/lib \
    qemu-aarch64-static -L run/sysroot .sdk/bin/01_offline_kinematics
```

Demos 04, 06 and 07 are the C++ programs that move the robot. All three refuse to move while
the robot reports a fault, print the whole plan, and **require `--yes` to actually execute**.
Demo 04 additionally validates its target against the limits the controller reports.

### What is safe to run on the robot

Nothing here has ever run on real hardware, so "safe" below means "what it can touch", not
"verified". One thing to know before pointing any of this at the robot: **`--yes` is the only
thing that commands motion**, but it is not the only thing that touches hardware. The C++
programs call `OnRobot()` at startup, which switches the robot to `ready` and powers the
low-level board.

| Program | On hardware |
|---|---|
| `python/vendor_model/` (`example_offline_kinematics.py`, `python -m vendor_model.cli`) | **Nothing at all** — no SDK, no socket, no device. Pure computation over the YAML. |
| `research/vendor-tools/cpp/01_offline_kinematics.cpp` | **No power-on.** FK, `getConfig`, `GetRobotState` all work without `OnRobot()`, so this runs without energising anything. `--power-on` adds `getDof` and `IK` and does power the board. |
| Dry runs of `04`, `06`, `07` | Read-only commands, but `OnRobot()` has already powered the board. |
| Any invocation with `--yes` | **Commands motion.** Read `docs/hardware-bringup.md` first. |

`research/vendor-tools/cpp/02_read_telemetry.cpp` needs `OnRobot()` for `getJointerrcode()`, so it powers
the board in order to read.


## Linking the vendor SDK from your own program

How to point a build outside this repository at the vendor SDK.

**With CMake** (recommended, and verified to work from a project outside this repository):

```cmake
cmake_minimum_required(VERSION 3.16)
project(my_app CXX)
set(CMAKE_CXX_STANDARD 17)

include(/path/to/this/repo/research/vendor-tools/cmake/juxie-sdk.cmake)

add_executable(my_app main.cpp)
target_link_libraries(my_app PRIVATE Juxie::SDK)
```

```bash
# Cross-compiling from x86 for the robot's board:
cmake -S . -B build \
      -DCMAKE_CXX_COMPILER=aarch64-linux-gnu-g++ \
      -DCMAKE_SYSTEM_NAME=Linux -DCMAKE_SYSTEM_PROCESSOR=aarch64
cmake --build build
```

`Juxie::SDK` carries the include directory, the link directory, the transitive dependency
search path and an rpath, so you do not have to reproduce the flags by hand. Override
`JUXIE_SDK_ROOT` to build against a different copy of the tree.

**Without CMake:**

```bash
aarch64-linux-gnu-g++ -std=c++17 -O1 \
    -I"$SDK/usr/include" -I/usr/include/eigen3 \
    main.cpp \
    -L"$SDK/usr/lib" -Wl,-rpath-link,"$SDK/usr/lib" \
    -Wl,--disable-new-dtags -Wl,-rpath,"$SDK/usr/lib" \
    -ljuxie_controller -o my_app
```

`-Wl,-rpath-link` is for **link time** (otherwise `undefined reference` from the five transitive
libraries); `-Wl,--disable-new-dtags` is for **run time** (otherwise `libexecutor.so.3: cannot
open shared object file`). Why each is needed: [`research/vendor-analysis/sdk-usage.md`](../vendor-analysis/sdk-usage.md) §2. On
the robot you set `LD_LIBRARY_PATH` regardless — see
[`research/vendor-analysis/running-on-the-robot.md`](../vendor-analysis/running-on-the-robot.md).

**From Python** — the counterpart of the CMake target. `research/vendor-tools/python/build_bridge.sh` builds a C
ABI shim around the SDK (its public signatures use `std::array`, `std::vector` and Eigen, none
of which ctypes can express), and `juxie_sdk.py` next to it drives it. One command, on either machine:
the script compiles natively on the robot's board and cross-compiles from x86, and it records
which SDK tree it used, so the module finds both the library and its configuration by itself.

Why a C ABI and ctypes rather than pybind11 (short version: no target-architecture `Python.h`
needed, and ctypes releases the GIL around blocking calls): [`research/vendor-analysis/sdk-usage.md`](../vendor-analysis/sdk-usage.md) §7.

```bash
./research/vendor-tools/python/build_bridge.sh
python3 research/vendor-tools/python/sdk_min_example.py
```

```python
from juxie_sdk import Controller     # PYTHONPATH=research/vendor-tools/python

with Controller() as robot:
    print(robot.state_name(), robot.joint_positions())
    print(robot.fk_pose([0.0] * 14))       # no power-on
    robot.on_robot()                       # this powers the low-level board
    print(robot.get_dof(), robot.ik([0.2, 0.0, 0.5, 1.0, 0.0, 0.0, 0.0] * 2))
```

`JUXIE_SDK_BRIDGE` and `DUAL_ARM_SDK_CONFIG` override the two things it works out for itself.

Same architecture rule as the C++ SDK: both halves are aarch64, so it runs on the robot's
board, not on a laptop interpreter. The bridge does one thing the C++ header cannot — it turns
three ways of crashing the interpreter into `SdkError`: a second controller, the four methods
that segfault before `on_robot()`, and `fk_pose` with more than 14 values. Details in
[`research/vendor-analysis/sdk-usage.md`](../vendor-analysis/sdk-usage.md) §7.

**Your program must call `OnRobot()` before `getDof()`, `IK()`, `getJointerrcode()` or
`setJointZeroPosition()`**, or those segfault. `getFKpose`, `getConfig`, `GetRobotState` and
the telemetry reads work without it. The full list, the return-code convention, the joint
layouts and the traps are in [`research/vendor-analysis/sdk-usage.md`](../vendor-analysis/sdk-usage.md) — read it before writing
anything that moves the robot. [`research/vendor-tools/cpp/sdk_min_example.cpp`](cpp/sdk_min_example.cpp)
is the smallest program that links; [`research/vendor-tools/cpp/01_offline_kinematics.cpp`](cpp/01_offline_kinematics.cpp)
is the smallest one that does something useful without powering the robot.
