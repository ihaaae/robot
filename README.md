# shensi-Robot

Our own controller SDK for the **巨蟹智能 (Juxie) dual-arm robot**, built on the joint modules'
public CAN / CAN-FD protocol (PR0002).

The vendor ships a closed, aarch64-only controller SDK (`dual_arm_app` 0.6.4). We match it at
the level of abstraction — the same capabilities and layer boundaries — not in its details; its
analysis is kept in [`research/`](research/README.md) as frozen reference material.

**Read [`ARCHITECTURE.md`](ARCHITECTURE.md) first** (layers, constraints, order of authority),
then [`docs/development-plan.md`](docs/development-plan.md) (phases and tasks). Every other
document is listed in [`docs/index.md`](docs/index.md).

**Languages.** The program on the control board is Go; everything on the external computer
(L3 motion planning, the RPC client, applications) is Python. See `ARCHITECTURE.md` §6.

**Status.** L0 (wire codec, transport, trace) is implemented and tested in `cpp/`; nothing above
it exists yet. Nothing has run on the robot — the gates before the first motion are in
[`docs/hardware-bringup.md`](docs/hardware-bringup.md).

## Layout

```
cpp/             L0 in C++, implemented and tested; to be ported to Go, then removed
docs/            requirements, safety concept, verification, RPC interface, component docs,
                 plan, hardware facts, hardware bring-up, glossary
tools/           verify.sh
research/        reference only, frozen: vendor SDK analysis, tools that link it, evidence
vendor/          the vendor's originals and the SDK tree extracted from them, with manifests
```

`cpp/`, `docs/` and `tools/` are ours and are where new work goes. `research/` and
[`vendor/`](vendor/README.md) are records; nothing outside them depends on them, except that
`tools/verify.sh` checks the vendor originals' hashes.

## Quick start

Everything below runs offline, without the robot and without root.

```bash
# Check the repository: vendor originals unmodified, SDK tree complete, and the L0 tests
# (needs a host C++ compiler; skipped if none)
./tools/verify.sh

# Build and run only the L0 tests
./cpp/build.sh
```

`tools/verify.sh` proves nothing about motion or safety.

## Contributing

* The default checks must stay offline: `./tools/verify.sh` never opens a socket and never
  touches a robot. The frozen vendor tooling has its own check,
  `research/vendor-tools/verify-native.sh`; some of its Python tests read the committed
  vendor trajectories under `vendor/sdk/.../usr/etc/data/` — that is data, not a robot or a
  socket, but it does mean that run is not vendor-data-free.
* When adding a vendor artifact, record its origin and hash in `vendor/manifests/`.
* When adding a conclusion, say how it was established: static inspection, emulation, or
  real hardware.
* Hardware facts go into [`docs/hardware-facts.md`](docs/hardware-facts.md) first, with their
  source and confidence; design documents cite the row.
