# Running your own program on the robot

This is the handoff that was missing: how to get from a program you built to something running
against the real machine.

**Read the honesty note first.** Everything in this repository was established without the
robot — by reading the vendor package and running their arm64 binaries under emulation. The
robot has not arrived. So this page is split into **what the package tells us** (verifiable
from the `.deb`, and true regardless of the robot) and **what nobody knows yet** (needs the
hardware, marked ⚠️). Do not treat a ⚠️ item as settled.

## First: how the pieces stack up

It is easy to read `/dev/mem` below and conclude that the SDK talks over `/dev/mem` rather
than over CAN. It does not. There are two separate things, at two different layers:

```
your code
  └─ libjuxie_controller.so   Juxie::ControllerJuxie    <- what this repository gives you
       └─ libexecutor.so        builds the CAN frames
            └─ librk3576_can_canfd.so                    <- this layer uses /dev/mem
                 └─ RK3576 CAN-FD controller registers (0x2AC00000 / 0x2AC10000)
                      └─ CAN-FD bus (1 Mbps arbitration / 5 Mbps data, BRS)
                           └─ the joint modules, speaking the vendor's CANopen/CiA402 protocol
```

* **What goes over the wire** is the vendor's CAN protocol — the one in
  [`can-protocol-comparison.md`](can-protocol-comparison.md), which matches their published
  documentation frame for frame.
* **How the host reaches the CAN controller** is direct register mapping instead of Linux
  SocketCAN. That is a host-side access mechanism one layer below the protocol.
* **The vendor SDK is the top layer.** With `Juxie::ControllerJuxie` you never see a CAN frame
  or `/dev/mem`; both are implementation detail of the vendor's stack. This page is about
  deploying a program that links it. Our own replacement lives in `cpp/` and takes the second
  path below ([`development-plan.md`](development-plan.md)).

Which means there are two independent ways to drive the joints, and `/dev/mem` only matters
for one of them:

| | Use the vendor SDK | Write your own CAN layer (our `cpp/` SDK) |
|---|---|---|
| When | control software on the vendor's board | replacing the board, or driving the joints from a PC |
| You write | `MoveJ`, `IK`, `MoveJ_Canfd`, … | the CAN frames themselves |
| Your CAN interface | the vendor stack's, via `/dev/mem` | **yours**: SocketCAN, a ZLG USB-CAN adapter, … |
| `/dev/mem` | required, because the vendor's driver uses it | **irrelevant** |
| The CAN protocol document | an implementation detail you never touch | **your specification** |

**Why the second path is legitimate, and why the bundled ZLG library was not the reason.** The
package contained ZLG's USB-CAN library (`libusbcanfd.so`) under
`usr/include/third_party/zlgcanfd/libs/`, for **both aarch64 and x86**. It looks like evidence
that the vendor expects someone to write a PC-side CAN master — but it is not: **nothing in
the package references it.** Checked three ways, all negative: none of the 17 binaries lists
it in `DT_NEEDED`; no binary contains any `ZCAN_*` or `VCI_*` symbol; and no binary calls
`dlopen`. It sat in the include tree, not in `usr/lib`, and the x86 build would be useless on
an aarch64 board. Treat it as a third-party drop that got swept into their packaging. We
removed it — together with the rest of `usr/include/third_party/` — from the vendored tree.

What actually makes the second path viable is that **the joint modules' protocol is documented
publicly** — the vendor sells the modules as components and publishes the CANopen/CiA402
specification, which is what [`can-protocol-comparison.md`](can-protocol-comparison.md)
compares against the driver. That document is the specification for writing your own master;
the bundled library is a red herring.

## What the SDK needs at run time

Everything below applies to **running the vendor's stack on the vendor's board**, which is the
first column above. From the vendor package itself, not from guesswork:

| Requirement | Detail | Where it comes from |
|---|---|---|
| Architecture | **Linux aarch64** | `Architecture: arm64` in the package control file |
| Boost runtime | **not needed by the SDK** | the *removed* vendor application linked `libboost_thread` / `system` / `regex` 1.74.0, and the package declares no `Depends` — but none of the ten libraries the SDK is made of names Boost in `DT_NEEDED`. A program that links the SDK needs no Boost at all. |
| Configuration root | the `DUAL_ARM_SDK_CONFIG` environment variable must point at the config root (the vendor build defaults to `/usr/etc`) | the SDK reads it during `OnRobot()`; unset, the process dies with `basic_string::_M_construct null not valid` |
| Device access | `/dev/mem` (mapped at `0x2AC00000` / `0x2AC10000`) and `/dev/misc_shm_can0`, `/dev/misc_shm_can1` | needed because the vendor's CAN-FD driver maps the RK3576 controller registers directly instead of using SocketCAN. This is the **vendor stack's** requirement, not an interface of the SDK — if you write your own CAN layer it does not apply. |
| Eigen headers | needed to **compile**, not to run | `juxie_controller.h` includes `<Eigen/Dense>`; Eigen is not shipped in the package |

⚠️ Whether `/dev/mem` needs root, a specific user, or a capability on your robot's image is not
known from the package. Check `ls -l /dev/mem` and `id` on the robot.

## What it costs to compile there

The RK3576 is not the constraint. Measured on a 2.6 GHz x86 core, compiling the Python bridge
(`python/juxie_sdk_bridge.cpp`, 407 lines, `-O2`) takes **1.5 s** and peaks at **204 MB** RSS;
the eight `examples/cpp/` programs together take **9.4 s** at `-O1`. Nearly all of it is parsing
Eigen's templates, not our code.

Method, because the numbers are measured but the scaling is not: PassMark rates the RK3576 at
1036 single-thread against roughly 1800–2200 for that class of Xeon core, so expect about
**2x** — **~3–4 s** for the bridge, **~20 s** for everything. Neither build script parallelises
today, so that is wall-clock, not CPU-seconds.

Two things matter more than the speed:

* **The vendor's libraries are never compiled.** All ten `.so` files in `usr/lib` are prebuilt
  aarch64 binaries (22 MB, unstripped); we only link against them.
* **The real prerequisite is Eigen, and the package does not ship it.** `juxie_controller.h`
  includes `<Eigen/Dense>`, so the board needs its own `g++` plus `libeigen3-dev` — or simply a
  copy of `/usr/include/eigen3`, which is header-only. A build that fails on the board is far
  more likely to be a missing Eigen than a slow CPU.

You can also skip it entirely: cross-compiling on the workstation is the same 1.5 s, and
`build_bridge.sh` picks the compiler by itself.

## Deploying your program

The vendor libraries are aarch64 Linux shared objects with symlink chains. **Copy the whole
`usr/lib` directory preserving symlinks** — `-ljuxie_controller` only resolves because
`libjuxie_controller.so -> libjuxie_controller.so.3 -> libjuxie_controller.so.0.6.4` is intact.
`rsync -a` preserves them; `cp -r` and `scp -r` follow the links and flatten the chains into
plain copies, which breaks `-ljuxie_controller`.

```bash
# From your checkout, to the robot (adjust host and paths):
rsync -a vendor/sdk/dual-arm-app/0.6.4/usr/lib/  robot:/opt/juxie/usr/lib/
rsync -a vendor/sdk/dual-arm-app/0.6.4/usr/etc/  robot:/opt/juxie/usr/etc/
scp my_app                                       robot:/opt/juxie/

# On the robot:
export DUAL_ARM_SDK_CONFIG=/opt/juxie/usr/etc
export LD_LIBRARY_PATH=/opt/juxie/usr/lib
/opt/juxie/my_app
```

If you built with the CMake target in `cmake/juxie-sdk.cmake`, the binary carries an rpath to
wherever the tree was during the build — as `DT_RPATH`, because the target passes
`-Wl,--disable-new-dtags`, so the loader searches it for transitive dependencies too. That
makes `LD_LIBRARY_PATH` unnecessary **only if you deploy the tree to exactly that same path**.

Linking by hand without that flag, or deploying to a different path, needs `LD_LIBRARY_PATH`.
Without either one the loader finds `libjuxie_controller` and then fails on its own dependency:
`libexecutor.so.3: cannot open shared object file`. Why: [`sdk-usage.md`](sdk-usage.md) §2.

⚠️ The paths above are a suggestion, not a vendor convention. Nothing in the package dictates
where a user program or the SDK tree should live.

## One way in, and the vendor's own application above it

**Link the SDK into your own process and drive the hardware yourself.** This is what the C++
examples do. You get the servo-streaming interface (`MoveJ_Canfd`) and everything else without
a network hop.

The package also contains a vendor application, `dual_arm_app_interface_node`, which serves a
JSON-over-WebSocket API and a web UI on port 5566, plus a second TCP service on 30485. It is
**built on top of the SDK, not part of it** — it links `Juxie::ControllerJuxie` like any other
consumer (evidence: [`sdk.md`](sdk.md) §4). This repository neither documents that application
nor vendors its libraries; its executable is inside the `.deb`.

⚠️ **It matters anyway, because it owns the CAN bus while it runs.** The node opens the bus
through `/dev/mem` and owns the joint bus; a second process doing the same would at best
duplicate commands and at worst fight it. Assume **one owner of the hardware at a time**: stop
the node before running your own native program, and stop your program before restarting the
node.

⚠️ Also unknown: whether the node is already running on the shipped image, how it is started
(systemd unit, init script, manual), and whether the vendor's own `test_controller` binary is
installed there.

## What to establish the moment the robot arrives

In order, before writing anything that moves:

1. **Which image is on the board** — `uname -a`, `cat /etc/os-release`. Does it match
   `Architecture: arm64`? (Boost 1.74.0 matters only if you want to run the vendor's own node;
   your program does not need it — see the table above.)
2. **Is the node running** — `ss -ltnp | grep -E '5566|30485'`, and if so what started it.
3. **Device access** — `ls -l /dev/mem /dev/misc_shm_can*`, and whether your user can open them.
4. **A no-power program first** — `examples/cpp/01_offline_kinematics.cpp` in its default mode
   reads FK, limits and state without calling `OnRobot()`. If that runs and prints a sensible
   FK, your toolchain, deployment and configuration are all correct, and nothing has moved.
5. **Only then** something that powers the robot, and only after reading
   [`hardware-acceptance.md`](hardware-acceptance.md) — that page lists what is unverified and
   in what order to check it.

The emergency stop comes before all of this.

## Related

- [`sdk-usage.md`](sdk-usage.md) — the API contracts, the `OnRobot()` ordering, the traps.
- [`hardware-acceptance.md`](hardware-acceptance.md) — what to verify on arrival, in priority order.
- [`sdk.md`](sdk.md) §9 — the emulation bench, if you want to run the vendor's arm64 binaries
  without the robot.
