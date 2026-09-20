#!/usr/bin/env python3
"""Minimal program that drives the SDK from Python, and the best starting point for your own.

The Python counterpart of examples/cpp/sdk_min_example.cpp: only the SDK and the bridge, no
network, no vendor node. Build the bridge first -- it is aarch64, so this runs on the robot's
board or under emulation, not on a workstation interpreter:

    ./python/build_bridge.sh
    export JUXIE_SDK_BRIDGE=$PWD/python/build/juxie_sdk_bridge.so
    export DUAL_ARM_SDK_CONFIG=$PWD/vendor/sdk/dual-arm-app/0.6.4/usr/etc
    python3 examples/python/sdk_min_example.py

Under emulation, with an aarch64 interpreter (see docs/sdk.md section 9):

    qemu-aarch64-static -L run/sysroot run/sysroot/usr/bin/python3.11 \
        examples/python/sdk_min_example.py
"""
from __future__ import annotations

from shensi_robot.sdk import Controller


def main() -> int:
    with Controller() as robot:
        print("bridge loaded and library resolved")

        # --------------------------------------------------------------- without on_robot()
        # These work before on_robot() and touch no hardware. fk_pose takes 7 to 14 values and
        # refuses anything else; 14 with 7 + 7 is the shape that was measured. See
        # docs/sdk-usage.md section 6.1 for why more is dangerous.
        pose = robot.fk_pose([0.0] * 14)
        print("FK(zeros)    =", " ".join(f"{value:.6f}" for value in pose))

        upper, lower = robot.config()
        print(f"limits       = [{lower[0]:.4f}, {upper[0]:.4f}] rad for joint 1")
        print(f"state        = {robot.state()} (0 = power_off, since on_robot() has not run)")

        # ------------------------------------------------------------ the mandatory call
        # on_robot() switches the robot to "ready" and powers the low-level board. Without it
        # get_dof(), ik(), joint_err_codes() and set_joint_zero_position() segfault inside the
        # SDK; the bridge refuses them with SdkError instead. It returns False when the board
        # does not come up -- with no CAN bus attached, for instance -- and the call still
        # initialises the controller either way.
        #
        # Comment out these three lines to keep the program power-off.
        print("on_robot()   =", robot.on_robot())
        print("dof          =", robot.get_dof(), "(joints per arm)")
        print("state        =", robot.state(), f"({robot.state_name()})")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
