# shensi-Robot

Reverse engineering, tooling and an SDK record for the **巨蟹智能 (Juxie) dual-arm robot**,
based on the vendor's `dual_arm_app` 0.6.4 package for Linux arm64.

We bought the robot; the vendor shipped a `.deb` and some loose documents, but no
controller SDK documentation. This repository is the result of working out what is actually
in that package, and everything needed to build against it without asking the vendor.

It holds three things, which answer different questions:

* **A buildable binary SDK** for writing applications against the vendor's own controller
  (`Juxie::ControllerJuxie`). This is the fastest path if your code runs on the vendor's board.
* **A protocol and evidence record** for writing an **independent CAN-FD master** that talks to
  the joint modules directly — the vendor's published protocol, compared frame by frame against
  their implementation.
* **Our own controller SDK, in progress** in `cpp/`, built on that protocol and meant to
  replace `Juxie::ControllerJuxie`. The vendor SDK is its reference, not its dependency. Plan:
  [`docs/development-plan.md`](docs/development-plan.md); L0 (wire, transport, trace) is done.

The binary SDK is **not** a portable controller foundation: it is aarch64-only, closed, and it
reaches the joints through the vendor board's `/dev/mem` registers and shared memory. If you
intend to *replace* the controller rather than write an application on top of it, the protocol
record and our own SDK are what you need, and the vendor SDK's role there is reference
material. Nothing here has been validated on the robot — see [`docs/hardware-acceptance.md`](docs/hardware-acceptance.md).

## Read this first

Three findings change how you should use anything here:

**1. The package already exposes a complete high-level controller API.** `libjuxie_controller.so`
exports `Juxie::ControllerJuxie` with `MoveJ` / `MoveL` / `MoveJ_P` / `IK` / `getFKpose` /
servo modes, and the public header ships in the same package. You do not need to
reverse-engineer it — see [`docs/sdk-usage.md`](docs/sdk-usage.md). Exposed is not the same as
verified: it has known defects, the buffer overflow below among them, and nothing in it has
run on the robot yet.

**2. The vendor ships three mutually inconsistent kinematic models.** The YAML
configuration, the `get_FK_pose` command and the `get_IK_joint_position` command do not
agree with each other. Our offline FK reproduces the target poses the vendor's **IK** was
asked to reach to 0.09 mm — but only with a *fitted* 84.721 mm tool offset, and our numerical
IK was never compared against theirs. It disagrees with their **FK** by hundreds of
millimetres in general configurations. Until this is resolved on real hardware, treat offline
FK as unverified. Details in
[`docs/kinematics.md`](docs/kinematics.md).

**3. `getFKpose` has a buffer overflow.** It allocates a fixed 7-double buffer and copies
`n - 7` doubles into it, so it is in bounds only while `n <= 14` — regardless of
`LeftNum` / `RightNum`. The WebSocket `get_FK_pose` command passes 17, which is why calling
it kills the whole control node. Measured under AddressSanitizer; reproduce it with
[`examples/cpp/fk_overflow_repro.cpp`](examples/cpp/fk_overflow_repro.cpp).
See [`docs/sdk-usage.md`](docs/sdk-usage.md) §6.1.

## Layout

```
docs/            our SDK's plan and interfaces, and our analysis of the vendor package
cpp/             our own controller SDK (C++); L0 is implemented and tested
src/             Python package (offline kinematics)
examples/cpp/    programs that link the vendor SDK using only its public header
cmake/           Juxie::SDK, for pointing your own CMake project at the SDK tree
tools/           probes and the verification script
tests/           offline unit tests (no network, no robot)
vendor/          the SDK tree you build against, plus the originals it came from
research/        evidence and vendor-derived material (symbol tables, recovered sources)
```

`vendor/` and `research/` are records. `cpp/`, `src/`, `tools/`, `examples/`, `cmake/` and
`tests/` are ours.

## Which of these do you want?

The robot can be driven three ways, and one part of this repository does not touch it at all.
Pick before reading further.

| I want to… | Use | Needs |
|---|---|---|
| **Compute offline** — forward/inverse kinematics, joint limits, plotting | `shensi_robot.kinematics` (Python) | the committed YAML. No robot, no vendor binaries, no socket. |
| **Write native controller code** — real-time loops, servo streaming, anything on the board | the vendor C++ SDK: `#include <juxie_controller/juxie_controller.h>` | a **Linux aarch64** target on the robot's board. Cross-compile from x86 or build natively there. |
| **Replace the controller** — your own CAN-FD master talking to the joint modules | our own SDK in `cpp/`, planned in [`docs/development-plan.md`](docs/development-plan.md), on the vendor's published protocol and our frame-by-frame comparison: [`docs/can-protocol-comparison.md`](docs/can-protocol-comparison.md) | a **CAN-FD interface you control** (SocketCAN, a USB-CAN adapter). The vendor SDK is **not** part of this path; see the acceptance gates at the end of [`docs/hardware-acceptance.md`](docs/hardware-acceptance.md). |

> The C++ SDK is **not** a workstation library. Those `.so` files are aarch64 Linux and are
> meant to run on the robot's board, not to be loaded from your laptop over the network.
>
> The vendor also ships a WebSocket application for the workstation case. It is **built on
> top of the SDK, not part of it** — it links `Juxie::ControllerJuxie` the same way any
> consumer does — so this repository neither documents it nor vendors its libraries. See
> [`docs/running-on-the-robot.md`](docs/running-on-the-robot.md).
>
> None of these needs the `.deb` — the SDK tree is committed and ready.

## Prerequisites

Once per machine:

```bash
# For the Python parts.
python3 -m venv .venv && . .venv/bin/activate && pip install -e ".[dev]"

# To compile anything against the SDK -- the C++ examples, and the Python bridge that
# shensi_robot.sdk loads. On the robot's board use its own g++; on an x86 host cross-compile
# with aarch64-linux-gnu-g++. Eigen and CMake are the same either way.
sudo apt-get install -y libeigen3-dev cmake
sudo apt-get install -y g++-aarch64-linux-gnu     # x86 host only; the board uses its own g++
```

Installing the package tries to build the Python bridge, and skips it quietly if the compiler
or Eigen is missing -- so on the board, install the toolchain *before* `pip install`, or run
`./python/build_bridge.sh` afterwards.

## Quick start

Everything below works without the robot, and without root.

```bash
# 0. Check the repository (offline). Drop --with-native if you have not installed the
#    cross toolchain above; without it that flag fails rather than skipping.
./tools/verify.sh --with-native

# 1. The SDK is the committed tree; there is nothing to install or unpack
export SDK="$PWD/vendor/sdk/dual-arm-app/0.6.4"

# 2. Build the C++ examples against it
./examples/cpp/build.sh

# 3. Offline kinematics (pure Python, no SDK binaries involved)
export DUAL_ARM_SDK_CONFIG="$SDK/usr/etc"
shensi-kin --arm left fk --joints "0 0 0 0 0 0 0"
```

`tools/verify.sh` is the executable form of this repository's acceptance criterion: a fresh
clone can check that the vendor originals are unmodified, that the committed SDK tree is
complete and linkable, that the Python package carries no vendor data and its tests pass, and
that our L0 layer (`cpp/`) reproduces the protocol document's own frames — that last step runs
offline with only a host C++ compiler, and is skipped if there is none. With `--with-native` it
adds cross-compiling the C++ examples and building a consumer project against `Juxie::SDK`.
It does not run anything against the vendor binaries, does not check the robot, and proves
nothing about motion or safety.
See [The SDK and its provenance](#the-sdk-and-its-provenance).

**To run something you built**, pick one:

* **On the robot** — see [`docs/running-on-the-robot.md`](docs/running-on-the-robot.md).
* **Under emulation on this machine** — the aarch64 binaries run under `qemu-user-static` with
  a faked `/dev/mem`. That setup is a manual recipe needing root and arm64 packages:
  [`docs/sdk.md`](docs/sdk.md) §9. It is how every "emulation" claim in `docs/` was produced.
* **Your own program, from your own project** — see
  [Writing your own program](#writing-your-own-program).

## Demos

`examples/python/` — two programs:

| Program | What it shows | Needs |
|---|---|---|
| [`01_offline_kinematics.py`](examples/python/01_offline_kinematics.py) | FK, IK, joint limits, and the 84.7 mm tool-frame trap | only the YAML config |
| [`sdk_min_example.py`](examples/python/sdk_min_example.py) | the SDK itself, from Python, through the bridge | the SDK, and `./python/build_bridge.sh` |

```bash
export DUAL_ARM_SDK_CONFIG="$PWD/vendor/sdk/dual-arm-app/0.6.4/usr/etc"
python3 examples/python/01_offline_kinematics.py            # no robot needed
```

`examples/cpp/` — five runnable demos, two diagnostic programs and one bug reproduction,
all linking the vendor SDK through its public header only. The numbering is historical: it
was once shared with `examples/python/`, which now holds only 01.

| Program | What it shows | Needs |
|---|---|---|
| [`01_offline_kinematics.cpp`](examples/cpp/01_offline_kinematics.cpp) | FK, IK, the joint limits the controller reports, the `-100` sentinel, and the `getFKpose` limit | only the SDK |
| [`02_read_telemetry.cpp`](examples/cpp/02_read_telemetry.cpp) | state, the 17-slot joints decoded, TCP pose, torques, the three error encodings | a robot on the CAN bus |
| [`04_guarded_motion.cpp`](examples/cpp/04_guarded_motion.cpp) | enable → move → disable, with preflight checks. **Dry run by default** | a robot on the CAN bus |
| [`06_vendor_cyclic_motion.cpp`](examples/cpp/06_vendor_cyclic_motion.cpp) | the vendor's built-in cyclic motion, run through the SDK instead of the node's command. **Dry run by default** | a robot on the CAN bus |
| [`07_replay_trajectory.cpp`](examples/cpp/07_replay_trajectory.cpp) | replays the vendor's recorded CSV through the 50 Hz streaming interface, the way the vendor's own `MoveJCanfdTest` does. **Dry run by default** | a robot on the CAN bus |
| [`sdk_min_example.cpp`](examples/cpp/sdk_min_example.cpp) | the smallest program that links the SDK at all | only the SDK |
| [`sdk_probe.cpp`](examples/cpp/sdk_probe.cpp) | one SDK method per run, so a crash in one cannot hide the others | only the SDK |
| [`fk_overflow_repro.cpp`](examples/cpp/fk_overflow_repro.cpp) | **a bug reproduction, not a demo** — it is meant to fail. Proves the `getFKpose` overflow under AddressSanitizer | only the SDK |

```bash
./examples/cpp/build.sh
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
| `examples/python/01_offline_kinematics.py`, `shensi-kin` | **Nothing at all** — no SDK, no socket, no device. Pure computation over the YAML. |
| `examples/cpp/01_offline_kinematics.cpp` | **No power-on.** FK, `getConfig`, `GetRobotState` all work without `OnRobot()`, so this runs without energising anything. `--power-on` adds `getDof` and `IK` and does power the board. |
| Dry runs of `04`, `06`, `07` | Read-only commands, but `OnRobot()` has already powered the board. |
| Any invocation with `--yes` | **Commands motion.** Read `docs/hardware-acceptance.md` first. |

`examples/cpp/02_read_telemetry.cpp` needs `OnRobot()` for `getJointerrcode()`, so it powers
the board in order to read.

## Writing your own program

The examples are all inside this repository. Your program will not be, so here is the part
that matters: how to point your own build at the SDK.

**With CMake** (recommended, and verified to work from a project outside this repository):

```cmake
cmake_minimum_required(VERSION 3.16)
project(my_app CXX)
set(CMAKE_CXX_STANDARD 17)

include(/path/to/this/repo/cmake/juxie-sdk.cmake)   # or put cmake/ on CMAKE_MODULE_PATH

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
open shared object file`). Why each is needed: [`docs/sdk-usage.md`](docs/sdk-usage.md) §2. On
the robot you set `LD_LIBRARY_PATH` regardless — see
[`docs/running-on-the-robot.md`](docs/running-on-the-robot.md).

**From Python** — the counterpart of the CMake target. `python/build_bridge.sh` builds a C
ABI shim around the SDK (its public signatures use `std::array`, `std::vector` and Eigen, none
of which ctypes can express), and `shensi_robot.sdk` drives it. One command, on either machine:
the script compiles natively on the robot's board and cross-compiles from x86, and it records
which SDK tree it used, so the module finds both the library and its configuration by itself.

Why a C ABI and ctypes rather than pybind11 (short version: no target-architecture `Python.h`
needed, and ctypes releases the GIL around blocking calls): [`docs/sdk-usage.md`](docs/sdk-usage.md) §7.

```bash
./python/build_bridge.sh
python3 examples/python/sdk_min_example.py
```

```python
from shensi_robot.sdk import Controller

with Controller() as robot:
    print(robot.state_name(), robot.joint_positions())
    print(robot.fk_pose([0.0] * 14))       # no power-on
    robot.on_robot()                       # this powers the low-level board
    print(robot.get_dof(), robot.ik([0.2, 0.0, 0.5, 1.0, 0.0, 0.0, 0.0] * 2))
```

`pip install -e .` runs that same build too, so on the board the install is enough.
`JUXIE_SDK_BRIDGE` and `DUAL_ARM_SDK_CONFIG` override the two things it works out for itself.

Same architecture rule as the C++ SDK: both halves are aarch64, so it runs on the robot's
board, not on a laptop interpreter. The bridge does one thing the C++ header cannot — it turns
three ways of crashing the interpreter into `SdkError`: a second controller, the four methods
that segfault before `on_robot()`, and `fk_pose` with more than 14 values. Details in
[`docs/sdk-usage.md`](docs/sdk-usage.md) §7.

**Your program must call `OnRobot()` before `getDof()`, `IK()`, `getJointerrcode()` or
`setJointZeroPosition()`**, or those segfault. `getFKpose`, `getConfig`, `GetRobotState` and
the telemetry reads work without it. The full list, the return-code convention, the joint
layouts and the traps are in [`docs/sdk-usage.md`](docs/sdk-usage.md) — read it before writing
anything that moves the robot. [`examples/cpp/sdk_min_example.cpp`](examples/cpp/sdk_min_example.cpp)
is the smallest program that links; [`examples/cpp/01_offline_kinematics.cpp`](examples/cpp/01_offline_kinematics.cpp)
is the smallest one that does something useful without powering the robot.

## The SDK and its provenance

**The SDK is `vendor/sdk/dual-arm-app/0.6.4/`.** Headers, shared libraries, configuration and
the recorded trajectories, committed so that a fresh clone can compile and link against it
immediately. There is no installer, no unpack step and nothing to build first.

```bash
export SDK="$PWD/vendor/sdk/dual-arm-app/0.6.4"
ls "$SDK/usr/lib" "$SDK/usr/include" "$SDK/usr/etc"
```

| | Where | What it is for |
|---|---|---|
| **The SDK** | `vendor/sdk/dual-arm-app/0.6.4/usr/{lib,include,etc}` | What you build against. The symlink chains are real symlinks, because `-ljuxie_controller` depends on them. |
| **The evidence** | `vendor/originals/dual-arm-app/0.6.4/*.deb` plus the three vendor documents | Proof of where the SDK came from and what the vendor actually shipped. Never modified, never derived from at build time. Its sha256 is recorded in `vendor/manifests/`, and `tools/verify.sh` checks it. |

Nothing in this repository extracts, unpacks or repacks the `.deb`. It is kept because a
re-issued 0.6.4 with different bytes has to be distinguishable from this one, and because it
records what the vendor shipped rather than what we copied out of it.

Deliberately **not** vendored, because nothing needs them to build: the compiled web UI and
its 8 MB source map, and the three executables (`dual_arm_app_interface_node` and the two
gtest binaries). They remain inside the `.deb`, which is where to take them from if you ever
want them.

**Removed** from the tree after the fact, because they are not the SDK:

* `usr/include/third_party/` (6.2 MB — ZLG's USB-CAN SDK plus header copies of fmt,
  nlohmann/json, taskflow, websocketpp, piqp, csv.hpp, sdqp.hpp). Nothing in the package
  references ZLG's library, the public header needs only Eigen, and the one file that includes
  anything from that directory cannot compile anyway.
* `libweb_interface`, `libbot_interface` and `libbot_communication` (2.8 MB), plus
  `libbot_math` (8.4 KB, referenced by nothing). The first three exist only for the vendor's
  WebSocket application, which is built *on top of* the SDK and whose executable was never
  vendored either.

What is left in `usr/lib` is the SDK's own run-time closure: ten libraries. The `.deb` still
has all of it, so nothing is lost; [`docs/sdk.md`](docs/sdk.md) §1 has the evidence.

`usr/etc/data/array0_all/` **is** vendored (100 KB): those two CSVs are the vendor's recorded
joint trajectories, and `array04_2.csv` is the input to the vendor's own streaming test. See
[`docs/sdk.md`](docs/sdk.md) §3.

## Documents

The index is [`docs/index.md`](docs/index.md). It splits the documents into two groups:
**our own controller SDK** (living: [`development-plan.md`](docs/development-plan.md),
[`l0-interface.md`](docs/l0-interface.md), [`l3-executor-interface.md`](docs/l3-executor-interface.md),
[`hardware-acceptance.md`](docs/hardware-acceptance.md)) and **the vendor SDK analysis**
(reference: [`sdk.md`](docs/sdk.md), [`sdk-usage.md`](docs/sdk-usage.md),
[`kinematics.md`](docs/kinematics.md), [`can-protocol-comparison.md`](docs/can-protocol-comparison.md),
[`error-codes.md`](docs/error-codes.md), [`running-on-the-robot.md`](docs/running-on-the-robot.md)).

## Test bench

Most conclusions here were verified by running the vendor's arm64 binaries under
`qemu-user-static` with a faked `/dev/mem`, not on the robot. That is enough to verify
protocol, ABI and kinematics, and **not** enough to verify motion, state transitions or
fault handling. The setup is described in [`docs/sdk.md`](docs/sdk.md) §9; the
`run/` directory it produces is gitignored.

## Contributing

* Default test run must stay offline: `pytest` never opens a socket and never touches a
  robot. Anything that needs hardware is marked `hardware`, anything that needs the vendor
  binary and qemu is marked `vendor_binary`, and both are deselected by default. A few
  tests do read the committed vendor trajectories under `vendor/sdk/.../usr/etc/data/`;
  that is data, not a robot or a socket, but it does mean the default run is not
  vendor-data-free.
* When adding a vendor artifact, record its origin and hash in `vendor/manifests/` — a hash
  alone proves the bytes did not change, not where they came from.
* When adding a conclusion, say how it was established: static inspection, emulation, or
  real hardware.
