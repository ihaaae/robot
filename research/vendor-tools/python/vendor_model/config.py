"""Where the vendor configuration lives.

The robot's kinematics / planner / executor parameters ship inside the vendor's `.deb` as
YAML under `<sdk-root>/usr/etc/<model>/`. This library deliberately does **not** bundle them:
the configuration belongs to the SDK, and a stale bundled copy silently taking precedence
over the SDK actually installed on the robot would be worse than an explicit error.

So a configuration root is required, and is resolved in this order:

1. an explicit ``config_root`` argument,
2. the ``DUAL_ARM_SDK_CONFIG`` environment variable (which is what the vendor's own
   binaries read),
3. otherwise :class:`ConfigNotFoundError`, whose message says how to get one.
"""
from __future__ import annotations

import os
from pathlib import Path

ENV_VAR = "DUAL_ARM_SDK_CONFIG"

#: Model directories shipped in dual_arm_app 0.6.4, i.e. the valid values for ``model``.
MODELS = ("juxie_53", "juxie_62", "juxie_62KML", "juxie_73")

#: Which arm geometry to assume when the caller does not say.
DEFAULT_MODEL = "juxie_73"


class ConfigNotFoundError(FileNotFoundError):
    """Raised when no usable vendor configuration root can be located."""


def resolve_config_root(config_root: str | os.PathLike | None = None) -> Path:
    """Return the vendor configuration root, or raise with instructions on how to get one."""
    candidates = []
    if config_root is not None:
        candidates.append(Path(config_root))
    elif ENV_VAR in os.environ:
        candidates.append(Path(os.environ[ENV_VAR]))
    else:
        raise ConfigNotFoundError(
            "no vendor configuration root given.\n"
            f"Pass config_root=... or set {ENV_VAR}=<sdk-root>/usr/etc.\n"
            "The SDK tree is committed in this repository, so for a checkout that is:\n"
            "    export DUAL_ARM_SDK_CONFIG=$PWD/vendor/sdk/dual-arm-app/0.6.4/usr/etc"
        )

    for candidate in candidates:
        if candidate.is_dir():
            return candidate
    raise ConfigNotFoundError(
        f"configuration root {candidates[0]} is not a directory.\n"
        f"Set {ENV_VAR} to the SDK's usr/etc directory, for example "
        f"vendor/sdk/dual-arm-app/0.6.4/usr/etc in a checkout of this repository."
    )


def model_dir(config_root: str | os.PathLike | None = None, model: str = DEFAULT_MODEL) -> Path:
    """Return the directory holding one robot model's YAML files."""
    root = resolve_config_root(config_root)
    if model not in MODELS:
        raise ValueError(f"unknown model {model!r}; known models: {', '.join(MODELS)}")
    path = root / model
    if not path.is_dir():
        raise ConfigNotFoundError(
            f"model directory {path} does not exist. "
            f"Available: {', '.join(sorted(p.name for p in root.iterdir() if p.is_dir()))}"
        )
    return path
