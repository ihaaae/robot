"""Python toolkit for the Juxie (巨蟹智能) dual-arm robot.

:mod:`shensi_robot.kinematics` is offline forward/inverse kinematics reconstructed from the
vendor's YAML configuration. It needs a configuration root (see :mod:`shensi_robot.config`)
and no vendor binaries at all.

Read ``research/vendor-analysis/kinematics.md`` before trusting any number this library produces: the vendor
ships three mutually inconsistent kinematic models, and the offline FK is currently aligned
with only one of them.
"""
from __future__ import annotations

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
    "__version__",
]

__version__ = "0.1.0"
