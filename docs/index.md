# Documentation index

Start with the [repository README](../README.md) if you have not read it. This is the only
document index; other files link here instead of keeping their own list.

The documents fall into two groups with different lifecycles:

* **Our own controller SDK** — living documents. The plan and the layer interfaces change as
  work lands; each keeps a history section at the end.
* **Vendor SDK analysis** — reference material. It records what the vendor's `dual_arm_app`
  0.6.4 package contains and how it behaves. It changes only when a finding is corrected or the
  robot confirms or refutes something.

Each finding has **one home**. Where another document mentions it, it summarises in a line and
links to the home rather than repeating the argument.

## Our own controller SDK

| Document | What it covers | Status |
|---|---|---|
| [`development-plan.md`](development-plan.md) | The layering (L0–L5), the tasks per layer, and what "done" means for each. **Start here** | Draft v1 |
| [`l0-interface.md`](l0-interface.md) | L0 — wire codec, transport, trace and diff | Implemented in `cpp/` |
| [`l3-executor-interface.md`](l3-executor-interface.md) | L3 — the executor that owns the control clock, the watchdog and the last safety gate | Draft |
| [`hardware-acceptance.md`](hardware-acceptance.md) | What to verify when the robot arrives. The last section is the gate list for our own CAN master | Open work |

## Vendor SDK analysis

| Document | Home for | How the claims were established |
|---|---|---|
| [`sdk.md`](sdk.md) | What is inside the vendor package, the original project layout, packaging defects, the vendor's WebSocket node (§4), and the qemu test bench (§9) | Static inspection + emulation |
| [`sdk-usage.md`](sdk-usage.md) | The API contracts: the mandatory `OnRobot()` ordering, per-method results, joint layouts, link flags (§2), **SDK defects** (§6), and the Python bridge (§7) | Compiled and executed |
| [`error-codes.md`](error-codes.md) | The four failure encodings, and which codes we have actually observed | Compiled and executed |
| [`kinematics.md`](kinematics.md) | The kinematic model, the ZYX convention, the 84.721 mm offset and what it does *not* prove, the three-way model inconsistency | Executed against the vendor binary |
| [`can-protocol-comparison.md`](can-protocol-comparison.md) | The vendor's CAN documents vs the reverse-engineered driver, field by field; what is still unverified (§8) | Disassembly vs vendor documents |
| [`running-on-the-robot.md`](running-on-the-robot.md) | Deploying a program that links the **vendor** SDK onto the robot, and what is still unknown | From the package; ⚠️ items need the robot |

Everything except `hardware-acceptance.md` describes behaviour observed **without the
robot**: the vendor's arm64 binaries run under `qemu-user-static` with a faked `/dev/mem`.
That is sufficient to verify protocol, ABI and kinematics, and insufficient to verify
motion, state transitions or fault handling.

## Related material outside `docs/`

* `cpp/` — our own SDK. L0 is implemented; `./cpp/build.sh` builds and runs its tests.
* `research/evidence/` — symbol tables and DWARF source listings extracted from the binaries.
* `research/vendor-derived/` — text conversions of the vendor documents. Derived from vendor
  material, so still vendor material.
* `vendor/originals/documents/` — the vendor documents themselves (CANopen xlsx, fault-logic
  docx, controller user manual).
* `vendor/manifests/` — where every artifact came from, with hashes.
