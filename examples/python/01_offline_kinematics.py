#!/usr/bin/env python3
"""Demo 1 — offline kinematics. No robot, no SDK binaries, no network.

This is the only demo that works with nothing but the vendor YAML configuration, so it is
the right starting point while the robot is still in transit.

It also demonstrates the single most important gotcha in this codebase: the vendor has two
different tool-frame conventions, and mixing them costs you 84.7 mm.

    DUAL_ARM_SDK_CONFIG=<sdk>/usr/etc python3 examples/python/01_offline_kinematics.py
"""
from __future__ import annotations

import argparse

import numpy as np

import shensi_robot as S


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--config-root", default=None,
                        help=f"defaults to ${S.__dict__.get('ENV_VAR', 'DUAL_ARM_SDK_CONFIG')}")
    parser.add_argument("--model", default=S.DEFAULT_MODEL, choices=S.MODELS)
    args = parser.parse_args()

    left = S.load_arm("left", args.config_root, args.model)
    right = S.load_arm("right", args.config_root, args.model)

    print("=" * 78)
    print("configuration")
    print("=" * 78)
    for name, arm in (("left", left), ("right", right)):
        print(f"  {name:5s} dof={arm.dof}  source={arm.source}")

    # ------------------------------------------------------------------ forward kinematics
    print()
    print("=" * 78)
    print("forward kinematics")
    print("=" * 78)

    zero = [0.0] * left.dof
    pose = left.fk_pose(zero, "zyx", tcp_offset=0.0)
    print(f"  left arm at the zero configuration: {np.round(pose, 6).tolist()}")
    print("  ^ this z must equal what get_tcp_pose reports on the robot (0.6752)")

    # A real configuration: the vendor's own IK solution for a target 20 cm out, 50 cm up.
    q = [1.5589096407625993, -0.14092117020676254, 0.158, 1.8265720752129397,
         0.14298261185531347, 1.8019568291722654, -1.7514364750421243]
    print()
    print(f"  joints            {np.round(q, 4).tolist()}")
    print(f"  T (4x4)           \n{np.round(left.fk(q), 5)}")
    print(f"  pose (tcp_offset=0)      {np.round(left.fk_pose(q, 'zyx', 0.0), 5).tolist()}")
    print(f"  pose (tcp_offset=IK)     "
          f"{np.round(left.fk_pose(q, 'zyx', S.IK_TCP_OFFSET), 5).tolist()}")

    # ------------------------------------------------------------------ the 84.7 mm trap
    print()
    print("=" * 78)
    print("why the tool-frame offset matters")
    print("=" * 78)
    target = [0.20, 0.0, 0.50, 0.0, 0.0, 0.0]
    print(f"  target pose (as passed to get_IK_joint_position): {target}")
    plain = left.fk_pose(q, "zyx", tcp_offset=0.0)
    ik_frame = left.fk_pose(q, "zyx", tcp_offset=S.IK_TCP_OFFSET)
    print(f"  same joints, measured against get_tcp_pose  -> {np.round(plain[:3], 5).tolist()}"
          f"   ({np.linalg.norm(np.array(plain[:3]) - np.array(target[:3])) * 1000:.1f} mm off)")
    print(f"  same joints, measured against the IK frame   -> {np.round(ik_frame[:3], 5).tolist()}"
          f"   ({np.linalg.norm(np.array(ik_frame[:3]) - np.array(target[:3])) * 1000:.1f} mm off)")
    print(f"  difference: {S.IK_TCP_OFFSET * 1000:.3f} mm along the tool's local z")
    print("  -> pass tcp_offset=0 when comparing against get_tcp_pose / MoveJ_P,")
    print(f"     and tcp_offset={S.IK_TCP_OFFSET} when comparing against get_IK_joint_position.")

    # ------------------------------------------------------------------ inverse kinematics
    print()
    print("=" * 78)
    print("inverse kinematics (damped least squares, this library)")
    print("=" * 78)
    for target in ([0.20, 0.0, 0.50, 0.0, 0.0, 0.0],
                   [0.30, 0.10, 0.45, 0.0, 0.10, 0.0],
                   [5.00, 0.0, 0.50, 0.0, 0.0, 0.0]):  # deliberately unreachable
        solved, converged = left.ik(target, convention="zyx", tcp_offset=S.IK_TCP_OFFSET)
        reached = left.fk_pose(solved, "zyx", S.IK_TCP_OFFSET)
        error = np.linalg.norm(np.array(reached[:3]) - np.array(target[:3])) * 1000
        label = "reachable  " if converged else "UNREACHABLE"
        print(f"  {label} target={str(target):38s} residual={error:9.3f} mm")
        if converged:
            print(f"             joints={np.round(solved, 4).tolist()}")

    # ------------------------------------------------------------------ joint limits
    print()
    print("=" * 78)
    print("joint limits from the configuration")
    print("=" * 78)
    lo = np.array([row[1] for row in left.limits])
    hi = np.array([row[0] for row in left.limits])
    for i, (a, b) in enumerate(zip(np.minimum(lo, hi), np.maximum(lo, hi))):
        print(f"  J{i + 1}  [{a:+.4f}, {b:+.4f}] rad  = [{np.degrees(a):+7.2f}, "
              f"{np.degrees(b):+7.2f}] deg")
    print()
    print("  NOTE: params.yml sets UseLimit: false, so the controller does not enforce these")
    print("        by default. Your own planner has to.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
