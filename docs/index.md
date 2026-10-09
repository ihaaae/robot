# Documentation index

Start with [`ARCHITECTURE.md`](../ARCHITECTURE.md): the layering, the constraints and the order
of authority. This is the only document index; other files link here instead of keeping their
own list.

The documents fall into two groups with different lifecycles:

* **Our own controller SDK** — living documents. The plan and the layer interfaces change as
  work lands.
* **Vendor SDK analysis** — reference material under [`research/`](../research/README.md),
  frozen. It records what the vendor's `dual_arm_app` 0.6.4 package contains and how it behaves.
  It changes only when a finding is corrected or the robot confirms or refutes something.

The two groups meet at one narrow interface, [`hardware-facts.md`](hardware-facts.md): design
documents take hardware facts from there and never cite vendor behaviour directly.

Each finding has **one home**. Where another document mentions it, it summarises in a line and
links to the home rather than repeating the argument.

## Our own controller SDK

Grouped by document type. Technical terms are used in English throughout; each one is defined
once, in Chinese, in [`glossary.md`](glossary.md).

| Type | Document | What it covers | Status |
|---|---|---|---|
| Requirements | [`requirements.md`](requirements.md) | Goal and scope, use cases (UC), functional (FR) and non-functional (NFR) requirements, gap check against the vendor SDK | Living |
| Architecture | [`ARCHITECTURE.md`](../ARCHITECTURE.md) | Context, constraints (C1–C5), deployment view, logical view (L0–L4), runtime view, key decisions, open questions, code map. **Start here** | Living |
| Cross-cutting | [`safety-concept.md`](safety-concept.md) | Every software safety mechanism across layers, how they relate, timing constraints, parameters still to calibrate, accepted risks | Draft |
| Cross-cutting | [`verification.md`](verification.md) | What decides that the code is right: the Joint Emulator, test doubles, offline tests, acceptance by feedback curves | Draft |
| Interface | [`interfaces/rpc.md`](interfaces/rpc.md) | The RPC contract between the external computer and the control board: operations, time conversion, link loss, the single control session | Draft |
| Component | [`components/l0-can-io.md`](components/l0-can-io.md) | L0 CAN I/O — wire codec, `Transport`, trace and diff. Interface and internal design are marked | Implemented in `cpp/`, frozen |
| Component | [`components/l2-executor.md`](components/l2-executor.md) | L2 Executor — the only clock owner: tick loop, motion sources, freshness, safety limiter. Interface and internal design are marked | Draft |
| Plan | [`development-plan.md`](development-plan.md) | Current status, phases and exit criteria, work packages with estimates and owners, interface freeze status | Draft |
| Hardware | [`hardware-facts.md`](hardware-facts.md) | Every hardware fact the design relies on, one row each: source, confidence, evidence. Conflicts and unknowns are marked | Living |
| Hardware | [`hardware-bringup.md`](hardware-bringup.md) | Hardware bring-up: what to verify when the robot arrives, starting with the gates before our own stack first moves the arm | Open work |
| Reference | [`glossary.md`](glossary.md) | Term → Chinese definition, with the document that owns each term | Living |

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
