# Research: the vendor SDK as reference material

This directory holds everything we learned from the vendor's `dual_arm_app` 0.6.4 package
(`Juxie::ControllerJuxie`) and the tools we used to learn it. It is **frozen**: findings are
corrected when wrong, and confirmed or refuted when the robot arrives, but nothing here is
extended and no code outside this directory depends on it.

## Why it is here and not in `docs/`

The goal of this repository is our own controller SDK on the public CAN / CAN-FD protocol,
matching the vendor SDK **at the level of abstraction only**: the same capabilities and layer
boundaries, not the same method names, return codes, state-machine cells or bus traffic
([`ARCHITECTURE.md`](../ARCHITECTURE.md)). The vendor SDK has real defects, its internals are opaque, and where
its details were vague or self-contradictory, chasing them distorted our design.

We still study it, because we lack experience with this hardware and the vendor stack is the
only thing that is known to drive it. The rule is:

* **Order of authority:** PR0002 and measurements on the robot, then our own requirements, then
  vendor behaviour. Vendor behaviour tells us "this at least works", never "this is required".
* **One narrow interface:** a finding that matters for the design is distilled into one row of
  [`docs/hardware-facts.md`](../docs/hardware-facts.md), with its source and confidence. Design
  documents cite that file, not this directory.

## Layout

| Path | What it is |
|---|---|
| [`vendor-analysis/`](vendor-analysis) | Write-ups: package contents, API contracts and defects, error codes, kinematics, the state machine, CAN protocol vs driver, deployment. Indexed in [`docs/index.md`](../docs/index.md) |
| [`vendor-tools/`](vendor-tools/README.md) | Programs that link the vendor binary: C++ demos and probes, the CMake target, the Python bridge, `verify-native.sh` |
| `evidence/` | Symbol tables, DWARF source listings and disassembly excerpts behind every "found in the binary" claim (`evidence/disasm/README.md` maps files to claims) |
| `vendor-derived/` | Text conversions of the vendor documents. Derived from vendor material, so still vendor material |

The vendor originals themselves (the `.deb`, the documents) and the extracted SDK tree live under
[`vendor/`](../vendor/README.md), with their provenance in `vendor/manifests/`.

## How the claims were established

Almost entirely **without the robot**: static reading of headers, symbols and disassembly, plus
the vendor's arm64 binaries run under `qemu-user-static` with a faked `/dev/mem`. That is enough
to establish protocol encodings, ABI and kinematics, and not enough to establish motion, state
transitions or fault handling on real hardware. Each write-up says which of its claims were
executed and which were only read.

## Three findings worth knowing even if you never use the vendor SDK

* **It has real defects.** `getFKpose` copies `n - 7` doubles into a 7-double buffer, so any
  call with more than 14 values overflows; the vendor's own WebSocket `get_FK_pose` passes 17
  and kills the control node. Reproduced under AddressSanitizer:
  [`fk_overflow_repro.cpp`](vendor-tools/cpp/fk_overflow_repro.cpp),
  [`sdk-usage.md`](vendor-analysis/sdk-usage.md) §6.1.
* **Its three kinematic models disagree.** The YAML configuration, the `get_FK_pose` command
  and the `get_IK_joint_position` command do not agree with each other. An offline FK matches
  the vendor's IK targets to 0.09 mm, but only with a fitted 84.721 mm tool offset, and disagrees
  with its FK by hundreds of millimetres: [`kinematics.md`](vendor-analysis/kinematics.md).
* **Its API is order-dependent.** `OnRobot()` must be called first — despite the name it does
  not switch power, it creates the executor and starts the polling thread — and several methods
  segfault without it: [`sdk-usage.md`](vendor-analysis/sdk-usage.md) §3.

To run the vendor stack itself — its demos, which of them touch hardware, linking `Juxie::SDK`
from CMake or Python — see [`vendor-tools/README.md`](vendor-tools/README.md); the API is
described in [`sdk-usage.md`](vendor-analysis/sdk-usage.md).
