# Research: the vendor SDK as reference material

This directory holds everything we learned from the vendor's `dual_arm_app` 0.6.4 package
(`Juxie::ControllerJuxie`) and the tools we used to learn it. It is **frozen**: findings are
corrected when wrong, and confirmed or refuted when the robot arrives, but nothing here is
extended and no code outside this directory depends on it.

## Why it is here and not in `docs/`

The goal of this repository is our own controller SDK on the public CAN / CAN-FD protocol,
matching the vendor SDK **at the level of abstraction only**: the same capabilities and layer
boundaries, not the same method names, return codes, state-machine cells or bus traffic
([`docs/development-plan.md`](../docs/development-plan.md) v2). An earlier aim of detail-level
compatibility was dropped. The vendor SDK has real defects, its internals are opaque, and where
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
| [`vendor-analysis/`](vendor-analysis/) | Write-ups: package contents, API contracts and defects, error codes, kinematics, the state machine, CAN protocol vs driver, deployment. Indexed in [`docs/index.md`](../docs/index.md) |
| [`vendor-tools/`](vendor-tools/README.md) | Programs that link the vendor binary: C++ demos and probes, the CMake target, the Python bridge, `verify-native.sh` |
| `evidence/` | Symbol tables, DWARF source listings and disassembly excerpts behind every "found in the binary" claim (`evidence/disasm/README.md` maps files to claims) |
| `vendor-derived/` | Text conversions of the vendor documents. Derived from vendor material, so still vendor material |

The vendor originals themselves (the `.deb`, the documents) and the extracted SDK tree live under
`vendor/`, with their provenance in `vendor/manifests/`.

## How the claims were established

Almost entirely **without the robot**: static reading of headers, symbols and disassembly, plus
the vendor's arm64 binaries run under `qemu-user-static` with a faked `/dev/mem`. That is enough
to establish protocol encodings, ABI and kinematics, and not enough to establish motion, state
transitions or fault handling on real hardware. Each write-up says which of its claims were
executed and which were only read.
