#!/usr/bin/env python3
"""Validate the offline FK against the vendor SDK's own getFKpose, directly.

Both implementations get the same joint vector and the forward poses are compared, so an FK
convention error shows up directly instead of through an inverse solve.

IMPORTANT -- this check is expected to FAIL, and that failure is the finding. The yml's
`screws`, `get_FK_pose` and `get_IK_joint_position` are three mutually inconsistent models:
our model matches the vendor's IK to 0.09 mm, but disagrees with their FK by hundreds of
millimetres for general configurations (every single-joint excitation still matches to
0.05 mm, and the rotation always matches exactly). See KINEMATICS.md for the analysis.

It drives `sdk_probe fkvec` (a tiny aarch64 program that links libjuxie_controller and
calls getFKpose) through qemu.

    python3 validate_fk_direct.py --probe run/sysroot/usr/bin/sdk_probe \
        --sysroot run/sysroot --config-root run/sysroot/usr/etc
"""
from __future__ import annotations

import argparse
import os
import re
import subprocess

import numpy as np

import vendor_model as S
from vendor_model.kinematics import rpy_to_rotation

QEMU = "qemu-aarch64-static"


def vendor_fk(probe: str, sysroot: str, joints: list[float]) -> np.ndarray | None:
    """Run the vendor SDK's getFKpose for a 14-value joint vector; return 12 poses values."""
    env = dict(os.environ, DUAL_ARM_SDK_CONFIG="/usr/etc", LD_LIBRARY_PATH="/usr/lib")
    cmd = [QEMU, "-L", sysroot, probe, "fkvec", "onrobot"] + [repr(v) for v in joints]
    try:
        out = subprocess.run(cmd, capture_output=True, text=True, timeout=90, env=env).stdout
    except subprocess.TimeoutExpired:
        return None
    match = re.search(r"fkvec = ([^\n]*)", out)
    if not match:
        return None
    values = [float(v) for v in match.group(1).split()]
    return np.array(values) if len(values) == 12 else None


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--probe", default="run/sysroot/usr/bin/sdk_probe")
    parser.add_argument("--sysroot", default="run/sysroot")
    parser.add_argument("--config-root", default="run/sysroot/usr/etc")
    parser.add_argument("--samples", type=int, default=8)
    parser.add_argument("--seed", type=int, default=11)
    args = parser.parse_args()

    arms = {name: S.load_arm(name, args.config_root) for name in ("left", "right")}
    rng = np.random.default_rng(args.seed)

    print(f"# {'left arm (mm / deg)':>26} {'right arm (mm / deg)':>26}   (tcp_offset=0)")
    left_errs, right_errs = [], []

    for i in range(args.samples):
        q_left = rng.uniform(-1.2, 1.2, 7)
        q_right = rng.uniform(-1.2, 1.2, 7)
        joints = list(q_left) + list(q_right)

        got = vendor_fk(args.probe, args.sysroot, joints)
        if got is None:
            print(f"  sample {i}: vendor returned nothing (crash or timeout)")
            continue

        cells = []
        for name, q, got_pose in (("left", q_left, got[:6]), ("right", q_right, got[6:])):
            mine = arms[name].fk_pose(q, "zyx", tcp_offset=0.0)
            dpos = float(np.linalg.norm(mine[:3] - got_pose[:3])) * 1000.0
            R_mine = rpy_to_rotation(mine[3:], "zyx")
            R_got = rpy_to_rotation(got_pose[3:], "zyx")
            cos = (np.trace(R_mine.T @ R_got) - 1.0) / 2.0
            drot = float(np.degrees(np.arccos(np.clip(cos, -1.0, 1.0))))
            cells.append(f"{dpos:9.4f} mm {drot:8.5f} deg")
            (left_errs if name == "left" else right_errs).append((dpos, drot))

        print(f"  sample {i}: {cells[0]:>26} {cells[1]:>26}")

    print()
    for name, errs in (("left", left_errs), ("right", right_errs)):
        if not errs:
            print(f"{name}: no samples")
            continue
        print(f"{name:5s} arm: max position {max(e[0] for e in errs):.4f} mm, "
              f"max rotation {max(e[1] for e in errs):.5f} deg, n={len(errs)}")


if __name__ == "__main__":
    main()
