# Vendor tooling (frozen)

Everything in this directory links or loads the vendor's binary SDK
(`vendor/sdk/dual-arm-app/0.6.4`, `Juxie::ControllerJuxie`). It was written while this
repository aimed to be detail-compatible with that SDK. That goal is gone: we now match the
vendor SDK only at the level of abstraction, and the binary is reference material. These tools
stay because they are how the findings in [`../vendor-analysis/`](../vendor-analysis/) were
obtained and can be reproduced. They are **frozen**: kept runnable, not extended. New code goes
in `cpp/` and `src/` and does not depend on anything here.

| Path | What it is |
|---|---|
| `cpp/` | C++ demos and probes against `Juxie::ControllerJuxie`; `cpp/build.sh` cross-compiles them into `.sdk/bin/` |
| `cmake/juxie-sdk.cmake` | the `Juxie::SDK` imported target, for a CMake project linking the vendor SDK |
| `python/` | C ABI bridge (`juxie_sdk_bridge.cpp`, `build_bridge.sh`), its ctypes module `juxie_sdk.py`, an example, and offline tests |
| `probes/` | disassembly excerpts and an FK cross-check used by the vendor analysis |
| `verify-native.sh` | build-checks for all of the above (formerly `tools/verify.sh --with-native`) |

```bash
./research/vendor-tools/verify-native.sh            # needs aarch64-linux-gnu-g++
./research/vendor-tools/python/build_bridge.sh
python3 research/vendor-tools/python/sdk_min_example.py   # on the board or under qemu
```

`juxie_sdk.py` is not part of the `shensi_robot` package. Put `research/vendor-tools/python`
on `PYTHONPATH` to import it from elsewhere; it still uses `shensi_robot.config`, so install
the package first (`pip install -e .`). How to run any of this under emulation:
[`../vendor-analysis/sdk.md`](../vendor-analysis/sdk.md) §9 and
[`../vendor-analysis/sdk-usage.md`](../vendor-analysis/sdk-usage.md).
