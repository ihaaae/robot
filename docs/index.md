# Documentation index

Start with the [repository README](../README.md) if you have not read it. This is the only
document index; other files link here instead of keeping their own list.

The documents fall into two groups with different lifecycles:

* **Our own controller SDK** — living documents. The plan and the layer interfaces change as
  work lands; each keeps a history section at the end.
* **Vendor SDK analysis** — reference material under [`research/`](../research/README.md),
  frozen. It records what the vendor's `dual_arm_app` 0.6.4 package contains and how it behaves.
  It changes only when a finding is corrected or the robot confirms or refutes something.

We match the vendor SDK only at the level of abstraction: capabilities at the same level (derived
from our own use cases) and similar layer boundaries, not the same method names, return codes or
bus traffic. The two groups meet at
one narrow interface, [`hardware-facts.md`](hardware-facts.md): design documents take hardware
facts from there and never cite vendor behaviour directly. The order of authority is PR0002 and
measurements on the robot, then our own requirements, then vendor behaviour (a hint only).

Each finding has **one home**. Where another document mentions it, it summarises in a line and
links to the home rather than repeating the argument.

## Our own controller SDK

| Document | What it covers | Status |
|---|---|---|
| [`development-plan.md`](development-plan.md) | The goal, the layering (L0–L4), the three phases and their exit criteria, the tasks, and what "done" means for each. **Start here** | Draft |
| [`deployment.md`](deployment.md) | Which layer runs on which machine: L0–L2 and the L4 state machine on the control board, L3 on an external computer, an RPC between them (time conversion, link loss) | Draft |
| [`hardware-facts.md`](hardware-facts.md) | Every hardware fact the design relies on, one row each: source, confidence, evidence. Conflicts and unknowns are marked | Living |
| [`l0-interface.md`](l0-interface.md) | L0 — wire codec, transport, trace and diff | Implemented in `cpp/` |
| [`l2-executor-interface.md`](l2-executor-interface.md) | L2 — the executor that owns the control clock, the watchdog and the last safety gate | Draft |
| [`hardware-acceptance.md`](hardware-acceptance.md) | What to verify when the robot arrives. The first section is the gate list for our own CAN master | Open work |

## Vendor SDK analysis (reference, frozen)

Read these to understand the hardware and to see what a working stack does; do not treat them as
a specification. Where they are vague or contradict each other, `hardware-facts.md` records the
conflict and the design does not depend on either side.

| Document | Home for | How the claims were established |
|---|---|---|
| [`sdk.md`](../research/vendor-analysis/sdk.md) | What is inside the vendor package, the original project layout, packaging defects, the vendor's WebSocket node (§4), and the qemu test bench (§9) | Static inspection + emulation |
| [`robot-state-machine.md`](../research/vendor-analysis/robot-state-machine.md) | The vendor's `power_off / ready / idle / running / fault` state machine: every method in every state, and the 5 ms polling thread that actually decides the state. Our L4 uses its cells as a scenario checklist, not as a specification | Disassembly; the `power_off` and `fault` columns and two lifecycle crashes executed under qemu |
| [`sdk-usage.md`](../research/vendor-analysis/sdk-usage.md) | The API contracts: the mandatory `OnRobot()` ordering, per-method results, joint layouts, link flags (§2), **SDK defects** (§6), and the Python bridge (§7) | Compiled and executed |
| [`error-codes.md`](../research/vendor-analysis/error-codes.md) | The four failure encodings, and which codes we have actually observed | Compiled and executed |
| [`kinematics.md`](../research/vendor-analysis/kinematics.md) | The kinematic model, the ZYX convention, the 84.721 mm offset and what it does *not* prove, the three-way model inconsistency | Executed against the vendor binary |
| [`can-protocol-comparison.md`](../research/vendor-analysis/can-protocol-comparison.md) | The vendor's CAN documents vs the reverse-engineered driver, field by field; how the vendor stack actually sends frames (§1.1); what is still unverified (§8) | Disassembly vs vendor documents; excerpts in `research/evidence/disasm/` |
| [`running-on-the-robot.md`](../research/vendor-analysis/running-on-the-robot.md) | Deploying a program that links the **vendor** SDK onto the robot, and what is still unknown | From the package; ⚠️ items need the robot |

All of the vendor analysis describes behaviour observed **without the robot**: the vendor's arm64 binaries run under `qemu-user-static` with a faked `/dev/mem`.
That is sufficient to verify protocol, ABI and kinematics, and insufficient to verify
motion, state transitions or fault handling.

## Related material outside `docs/`

* `cpp/` — our own SDK. L0 is implemented; `./cpp/build.sh` builds and runs its tests.
* `research/vendor-tools/` — frozen programs that link the vendor SDK (demos, probes, the
  Python bridge). See its README.
* `research/evidence/` — symbol tables and DWARF source listings extracted from the binaries.
* `research/evidence/disasm/` — disassembly excerpts behind every "found in the binary" claim,
  regenerated by `research/vendor-tools/probes/disasm_excerpts.sh`. Its README lists which file supports which claim.
* `research/vendor-derived/` — text conversions of the vendor documents. Derived from vendor
  material, so still vendor material.
* `vendor/originals/documents/` — the vendor documents themselves (CANopen xlsx, fault-logic
  docx, controller user manual).
* `vendor/manifests/` — where every artifact came from, with hashes.
