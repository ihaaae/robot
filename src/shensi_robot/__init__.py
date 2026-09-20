"""Python toolkit for the Juxie (巨蟹智能) dual-arm robot, from the `dual_arm_app` SDK.

:mod:`shensi_robot.kinematics` is offline forward/inverse kinematics reconstructed from the
vendor's YAML configuration. It needs a configuration root (see :mod:`shensi_robot.config`)
and no vendor binaries at all.

:mod:`shensi_robot.sdk` drives the vendor's C++ controller from Python, through the C ABI
bridge in ``python/``. It needs the SDK's aarch64 libraries and the compiled bridge, so it
only runs on the robot's board (or under emulation); importing it is free, and nothing is
loaded until you construct a controller.

Read ``docs/kinematics.md`` before trusting any number this library produces: the vendor
ships three mutually inconsistent kinematic models, and the offline FK is currently aligned
with only one of them.
"""
from __future__ import annotations

from . import sdk
from .config import ConfigNotFoundError, DEFAULT_MODEL, MODELS, resolve_config_root
from .kinematics import (
    ARM_DOF,
    IK_TCP_OFFSET,
    INVALID,
    JOINT_COUNT,
    LEFT_ARM_SLICE,
    RIGHT_ARM_SLICE,
    VENDOR_CYCLIC_POSE,
    WAIST_INDEX,
    ArmKinematics,
    load_arm,
)

__all__ = [
    "ARM_DOF",
    "ArmKinematics",
    "ConfigNotFoundError",
    "DEFAULT_MODEL",
    "IK_TCP_OFFSET",
    "INVALID",
    "JOINT_COUNT",
    "LEFT_ARM_SLICE",
    "MODELS",
    "RIGHT_ARM_SLICE",
    "VENDOR_CYCLIC_POSE",
    "WAIST_INDEX",
    "load_arm",
    "resolve_config_root",
    "sdk",
    "__version__",
]

__version__ = "0.1.0"
