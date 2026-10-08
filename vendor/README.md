# Vendor originals and the vendored SDK

Reference material only: our SDK (`cpp/`) does not link anything here. The tree is kept buildable
because the frozen tools in [`research/vendor-tools/`](../research/vendor-tools/README.md) link it.

| | Where | What it is for |
|---|---|---|
| **The SDK** | `sdk/dual-arm-app/0.6.4/usr/{lib,include,etc}` | What the vendor tools build against. The symlink chains are real symlinks, because `-ljuxie_controller` depends on them. |
| **The evidence** | `originals/dual-arm-app/0.6.4/*.deb` plus the vendor documents in `originals/documents/` | Proof of where the SDK came from and what the vendor actually shipped. Never modified, never derived from at build time. |
| **Provenance** | `manifests/` | Origin and sha256 of every original; `tools/verify.sh` checks the hashes. |

```bash
export SDK="$PWD/vendor/sdk/dual-arm-app/0.6.4"
ls "$SDK/usr/lib" "$SDK/usr/include" "$SDK/usr/etc"
```

Nothing in this repository extracts, unpacks or repacks the `.deb`. It is kept because a
re-issued 0.6.4 with different bytes has to be distinguishable from this one, and because it
records what the vendor shipped rather than what we copied out of it.

## What the vendored tree leaves out

Deliberately **not** vendored, because nothing needs them to build: the compiled web UI and
its 8 MB source map, and the three executables (`dual_arm_app_interface_node` and the two
gtest binaries). They remain inside the `.deb`.

**Removed** after the fact, because they are not the SDK:

* `usr/include/third_party/` (6.2 MB — ZLG's USB-CAN SDK plus header copies of fmt,
  nlohmann/json, taskflow, websocketpp, piqp, csv.hpp, sdqp.hpp). Nothing in the package
  references ZLG's library, the public header needs only Eigen, and the one file that includes
  anything from that directory cannot compile anyway.
* `libweb_interface`, `libbot_interface` and `libbot_communication` (2.8 MB), plus
  `libbot_math` (8.4 KB, referenced by nothing). The first three exist only for the vendor's
  WebSocket application, whose executable was never vendored either.

What is left in `usr/lib` is the SDK's own run-time closure: ten libraries. Evidence:
[`research/vendor-analysis/sdk.md`](../research/vendor-analysis/sdk.md) §1.

`usr/etc/data/array0_all/` **is** vendored (100 KB): those two CSVs are the vendor's recorded
joint trajectories, and `array04_2.csv` is the input to the vendor's own streaming test
([`sdk.md`](../research/vendor-analysis/sdk.md) §3).

## Adding an artifact

Record its origin and hash in `manifests/` — a hash alone proves the bytes did not change, not
where they came from.
