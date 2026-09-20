# Documentation index

Start with the [repository README](../README.md) if you have not read it.

## If you are here to write code

| Document | What it covers |
|---|---|
| [`running-on-the-robot.md`](running-on-the-robot.md) | **How to get a program onto the robot**: runtime requirements, deployment, the two ways to control it, and what is still unknown |
| [`sdk-usage.md`](sdk-usage.md) | The API contracts: the mandatory `OnRobot()` ordering, what needs it, per-method results, the joint layouts, the traps — and how to call the SDK from Python (`shensi_robot.sdk`) |
| [`error-codes.md`](error-codes.md) | The four failure encodings, and which codes we have actually observed |

## The analysis behind it

| Document | What it covers | How the claims were established |
|---|---|---|
| [`sdk.md`](sdk.md) | The main report: what is inside the vendor package, the original project layout, the high-level SDK, packaging defects | Static inspection + emulation |
| [`sdk-usage.md`](sdk-usage.md) | Building against the SDK; the mandatory `OnRobot()` call; per-method results; the qemu test bench | Compiled and executed |
| [`kinematics.md`](kinematics.md) | The kinematic model, the ZYX convention, the TCP offset, and the three-way model inconsistency | Executed against the vendor binary |
| [`can-protocol-comparison.md`](can-protocol-comparison.md) | The vendor's CAN documents vs the reverse-engineered driver, field by field | Disassembly vs vendor documents |
| [`hardware-acceptance.md`](hardware-acceptance.md) | **What to verify when the robot arrives**, in priority order | Open work |

Everything except `hardware-acceptance.md` describes behaviour observed **without the
robot**: the vendor's arm64 binaries run under `qemu-user-static` with a faked `/dev/mem`.
That is sufficient to verify protocol, ABI and kinematics, and insufficient to verify
motion, state transitions or fault handling.

Related material outside `docs/`:

* `research/evidence/` — symbol tables and DWARF source listings extracted from the binaries.
* `research/vendor-derived/` — text conversions of the vendor documents. Derived from vendor
  material, so still vendor material.
* `vendor/manifests/` — where every artifact came from, with hashes.
