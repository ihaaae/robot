"""Command line entry point: ``python -m vendor_model.cli``.

Kept out of the library modules so that importing ``vendor_model.kinematics`` never pulls in
an argument parser.
"""
from __future__ import annotations

import argparse
import sys

import numpy as np

from .config import DEFAULT_MODEL, ENV_VAR, MODELS
from .kinematics import load_arm

__all__ = ["kin_main"]


def _floats(text: str) -> list[float]:
    return [float(part) for part in text.replace(",", " ").split()]


def kin_main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        prog="python -m vendor_model.cli",
        description="Offline forward/inverse kinematics for the Juxie dual-arm robot.",
        epilog=f"The configuration root comes from --config-root or ${ENV_VAR}. "
               "The SDK tree is committed under vendor/sdk/dual-arm-app/0.6.4/.",
    )
    parser.add_argument("--arm", choices=("left", "right"), default="left")
    parser.add_argument("--config-root", default=None,
                        help=f"directory holding the model subdirectories (default: ${ENV_VAR})")
    parser.add_argument("--model", default=DEFAULT_MODEL, choices=MODELS)
    parser.add_argument("--rpy-convention", choices=("xyz", "zyx"), default="zyx",
                        help="ZYX is what the vendor uses (default)")
    sub = parser.add_subparsers(dest="action", required=True)

    fk = sub.add_parser("fk", help="forward kinematics for a joint vector")
    fk.add_argument("--joints", required=True, type=_floats)

    ik = sub.add_parser("ik", help="inverse kinematics for a [x,y,z,rx,ry,rz] pose")
    ik.add_argument("--pose", required=True, type=_floats)
    ik.add_argument("--seed", type=_floats, default=None)
    ik.add_argument("--tcp-offset", type=float, default=None,
                    help="use 0 to match get_tcp_pose, 0.084721 to match get_IK_joint_position")

    args = parser.parse_args(argv)
    arm = load_arm(args.arm, args.config_root, args.model)

    if args.action == "fk":
        T = arm.fk(args.joints)
        print("source :", arm.source)
        print("T      :\n", np.round(T, 6))
        print("pose   :", np.round(arm.fk_pose(args.joints, args.rpy_convention), 6))
        return 0

    offset = args.tcp_offset
    if offset is None:
        from .kinematics import IK_TCP_OFFSET
        offset = IK_TCP_OFFSET
    q, converged = arm.ik(args.pose, seed=args.seed, convention=args.rpy_convention,
                          tcp_offset=offset)
    print("converged:", converged)
    print("joints   :", np.round(q, 6).tolist())
    print("residual :", np.round(arm.fk_pose(q, args.rpy_convention, offset), 6).tolist())
    return 0 if converged else 1


def _guard(func, argv):
    try:
        return func(argv)
    except (FileNotFoundError, ValueError) as error:
        print(f"error: {error}", file=sys.stderr)
        return 1


def kin_entry() -> None:
    sys.exit(_guard(kin_main, None))


if __name__ == "__main__":
    kin_entry()
