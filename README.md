# shensi-Robot

Our own controller SDK for the **巨蟹智能 (Juxie) dual-arm robot**, built on the joint modules'
public CAN / CAN-FD protocol (PR0002).

The vendor ships a binary controller SDK (`dual_arm_app` 0.6.4, `Juxie::ControllerJuxie`):
aarch64-only, closed, reaching the joints through its board's `/dev/mem` registers. We match it
**at the level of abstraction** — capabilities at the same level, derived from our own use cases
and checked against its list — and not in its details: not its method names, return codes, state-machine cells or bus traffic. Its details are
often opaque and sometimes self-contradictory, and following them distorted our design. We still
study it, because it is the only stack known to drive this hardware; that study lives in
[`research/`](research/README.md), frozen, as reference material.

Where sources disagree, the order of authority is:

1. PR0002 and measurements on the robot;
2. our own requirements;
3. vendor behaviour — a hint that something at least works, never a requirement.

Every hardware fact the design relies on is one row in
[`docs/hardware-facts.md`](docs/hardware-facts.md), with its source and confidence. Design
documents cite that file, not the vendor analysis.

**Status.** L0 (wire codec, transport, trace) is implemented and tested in `cpp/`. L1–L5 are
designed in [`docs/development-plan.md`](docs/development-plan.md); L3 has an interface draft.
Nothing has run on the robot yet — the gates before the first motion are in
[`docs/hardware-acceptance.md`](docs/hardware-acceptance.md).

## Layout

```
cpp/             our controller SDK (C++); L0 is implemented and tested
docs/            plan, layer interfaces, hardware facts, acceptance gates
src/             Python package shensi_robot (offline kinematics; discarded, to be replaced — see below)
examples/        Python examples for the package
tests/           offline unit tests (no network, no robot)
tools/           verify.sh
research/        reference only, frozen: vendor SDK analysis, tools that link it, evidence
vendor/          the vendor's originals and the SDK tree extracted from them, with manifests
```

`cpp/`, `docs/`, `src/`, `examples/`, `tests/` and `tools/` are ours and are where new work goes.
`research/` and `vendor/` are records; nothing outside them depends on them, except that
`tools/verify.sh` checks the vendor originals' hashes and the offline kinematics reads the
vendor's YAML configuration.

## Layers

| Layer | What it is | State |
|---|---|---|
| L0 | CAN frame codec for PR0002, transport interface, trace recording and diff | done |
| — | virtual joint module: PR0002 as an executable spec, the offline test bench for L1 and L3; built before L1 | planned |
| L1 | one joint module: enable / brake / clear / mode via the control byte, SDO diagnostics, units; no threads | planned |
| L2 | merged into L3 as its construction-time config (joint ↔ bus, Dev_ID); the number is kept unused | — |
| L3 | executor: the one clocked layer — tick, `0x200` heartbeat, watchdog, freshness, a safety gate that cannot be disabled, the 14-joint map | interface draft |
| L4 | streaming and offline trajectory planning | planned |
| L5 | facade: our own API, error codes and state machine, and a C ABI | planned |

Details and the "done" criterion of each task: [`docs/development-plan.md`](docs/development-plan.md).

## Quick start

Everything below runs offline, without the robot and without root.

```bash
# Check the repository: vendor originals unmodified, SDK tree complete, Python tests,
# offline kinematics, and the L0 tests (needs a host C++ compiler; skipped if none)
./tools/verify.sh

# Build and run only the L0 tests
./cpp/build.sh

# Offline kinematics (pure Python). DISCARDED: it reads the vendor's YAML and was never validated;
# it is not a basis or a reference and will be replaced by a from-scratch C++ implementation with
# our own model file (docs/development-plan.md task 4). Kept only until then.
python3 -m venv .venv && . .venv/bin/activate && pip install -e ".[dev]"
export DUAL_ARM_SDK_CONFIG="$PWD/vendor/sdk/dual-arm-app/0.6.4/usr/etc"
shensi-kin --arm left fk --joints "0 0 0 0 0 0 0"
python3 examples/python/01_offline_kinematics.py
```

`tools/verify.sh` proves nothing about motion or safety, and never opens a socket or touches a
robot. The frozen vendor tooling has its own check, `research/vendor-tools/verify-native.sh`.

## Documents

The index is [`docs/index.md`](docs/index.md). Start with
[`development-plan.md`](docs/development-plan.md), then
[`hardware-facts.md`](docs/hardware-facts.md); the layer interfaces are
[`l0-interface.md`](docs/l0-interface.md) and [`l3-executor-interface.md`](docs/l3-executor-interface.md).

## The vendor SDK (reference)

If you need to run the vendor's own stack — to observe it, or to write an application on its
board — see [`research/vendor-tools/README.md`](research/vendor-tools/README.md): its demos,
which of them touch hardware, and how to link `Juxie::SDK` from your own CMake project or
from Python. The analysis behind it is under
[`research/vendor-analysis/`](research/vendor-analysis/), starting with
[`sdk-usage.md`](research/vendor-analysis/sdk-usage.md).

Three findings about it are worth knowing even if you never use it:

* **It has real defects.** `getFKpose` copies `n - 7` doubles into a 7-double buffer, so any
  call with more than 14 values overflows; the vendor's own WebSocket `get_FK_pose` passes 17
  and kills the control node. Reproduced under AddressSanitizer:
  [`fk_overflow_repro.cpp`](research/vendor-tools/cpp/fk_overflow_repro.cpp),
  [`sdk-usage.md`](research/vendor-analysis/sdk-usage.md) §6.1.
* **Its three kinematic models disagree.** The YAML configuration, the `get_FK_pose` command
  and the `get_IK_joint_position` command do not agree with each other. Our offline FK matches
  the vendor's IK targets to 0.09 mm, but only with a fitted 84.721 mm tool offset, and disagrees
  with its FK by hundreds of millimetres. Treat offline FK as unverified until measured:
  [`kinematics.md`](research/vendor-analysis/kinematics.md).
* **Powering on is a side effect of startup.** Its programs call `OnRobot()` early, which powers
  the low-level board; several methods segfault without it. Only one master may own the CAN bus
  at a time, so the vendor stack and ours are never run together — switching is whole.

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
has all of it, so nothing is lost; [`research/vendor-analysis/sdk.md`](research/vendor-analysis/sdk.md) §1 has the evidence.

`usr/etc/data/array0_all/` **is** vendored (100 KB): those two CSVs are the vendor's recorded
joint trajectories, and `array04_2.csv` is the input to the vendor's own streaming test. See
[`research/vendor-analysis/sdk.md`](research/vendor-analysis/sdk.md) §3.

The SDK tree is reference material for us: our SDK does not link it. It is kept buildable
because the frozen tools in `research/vendor-tools/` link it.

## Test bench

Most conclusions about the vendor SDK were established by running the vendor's arm64 binaries under
`qemu-user-static` with a faked `/dev/mem`, not on the robot. That is enough to verify
protocol, ABI and kinematics, and **not** enough to verify motion, state transitions or
fault handling. The setup is described in [`research/vendor-analysis/sdk.md`](research/vendor-analysis/sdk.md) §9; the
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
* Hardware facts go into [`docs/hardware-facts.md`](docs/hardware-facts.md) first, with their
  source and confidence; design documents cite the row.
