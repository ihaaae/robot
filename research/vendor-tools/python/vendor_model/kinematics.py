#!/usr/bin/env python3
"""Offline kinematics for the Juxie dual-arm robot, reconstructed from the vendor's
`kinematics_leftArm.yml` / `kinematics_rightArm.yml` configuration files.

The vendor uses screw theory (product of exponentials), not Denavit-Hartenberg:

    T(joints) = Exp([S1] t1) ... Exp([S7] t7) @ M

where each `screws[i]` is a unit screw axis [omega(3), v(3)] expressed in the arm base
frame (v = -omega x q, with q the point the axis passes through), and `M` is the tool
frame at the zero configuration, given as SE(3) exponential coordinates.

Units: radians in, metres out.

Verified against the vendor binary (see `research/vendor-tools/probes/validate_fk_direct.py` and
`research/vendor-analysis/kinematics.md`):

* RPY convention is **ZYX**, i.e. R = Rz(rz) @ Ry(ry) @ Rx(rx). Using XYZ instead shows up
  as a 0.69 deg orientation error.
* With `tcp_offset=IK_TCP_OFFSET` this model reproduces the **target poses** the vendor's
  `get_IK_joint_position` solves for, to 0.09 mm / 0.0000 deg across 7 poses on both arms.
  Note what that does and does not show: the comparison feeds the vendor's joint angles into
  this module's `fk_pose()`. It says nothing about this module's `ik()`, and nothing about a
  constant offset between the vendor's IK and its FK.
* With `tcp_offset=0` it matches `get_tcp_pose` (which reports exactly the yml `M`).

CAVEAT: this model is aligned with the vendor's **IK**, not with their **FK**. Feeding the
same joint vector to `getFKpose` and to `fk()` disagrees by hundreds of millimetres for
general configurations. Single-joint excitations match in rotation exactly but not in
translation: the three axes with non-zero `v` (`screws[1/3/5]`) are 9-24 mm out. (The
0.05 mm figure sometimes quoted belongs to an experimental model with hand-corrected axis
heights, not to this one.) So the yml, `get_FK_pose` and `get_IK_joint_position` are three
mutually inconsistent models. Use `fk()` to predict where a joint vector lands only after
that is resolved on real hardware -- see KINEMATICS.md.

Command line:
    python -m vendor_model.cli --arm left fk --joints "1.5589 -0.1409 0.158 1.8266 0.1430 1.8020 -1.7514"
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import yaml

from .config import DEFAULT_MODEL, resolve_config_root

#: Joint vector layout: waist(1) | left arm(7) | right arm(7) | head(2)
JOINT_COUNT = 17
ARM_DOF = 7
WAIST_INDEX = 0
LEFT_ARM_SLICE = slice(1, 8)
RIGHT_ARM_SLICE = slice(8, 15)

#: Sentinel meaning "keep the current value" in the vendor's C++ API.
INVALID = -100.0

#: The pose the vendor's built-in cyclic motion (`fixed_action`, the 「循环运动」 button in
#: the web UI) moves to, alternating with the all-zero home pose. Recovered from the
#: `fixed_action` handler in `dual_arm_app_interface_node`: it calls `getDof()`, copies one of
#: two 136-byte (17-double) constants out of `.rodata` depending on the result, then loops
#: `MoveJ(zeros, v); MoveJ(pose, v)` `count` times. This is the dof == 7 constant, which is
#: the branch this machine takes (`getDof()` returns 7). Both arms get the same pose.
#:
#:   0.0 | 1.57079 2.443459 1.80526 1.58166 1.530725 1.258 1.5 | (same for the right arm) | 0.0 0.0
#:
#: The dof == 6 constant in the same binary is
#: `[0, -0.5, 0, 0, 1.1, 0, 0, 0, -0.5, 0, 0, 1.1, 0, 0, 0, 0, 0]`, for the 6-axis variant.
VENDOR_CYCLIC_POSE: tuple[float, ...] = (
    0.0,
    1.57079, 2.443459, 1.80526, 1.58166, 1.530725, 1.258, 1.5,
    1.57079, 2.443459, 1.80526, 1.58166, 1.530725, 1.258, 1.5,
    0.0, 0.0,
)

#: Tool offset, in metres along the tool frame's local +z, that separates the two TCP
#: definitions the vendor uses. `get_tcp_pose` and `get_FK_pose` report the frame described
#: by the yml `M` parameter (offset 0); `get_IK_joint_position` solves for a frame that is
#: this much further out. Measured against the vendor binary over 8 target poses:
#: translation (0, 0, 84.721) mm with 0.02-0.05 mm spread, rotation identity to 1.3e-3.
#: The yml's `linksXYZ[5] = [0, 0, -0.084668]` matches this magnitude (84.668 mm).
IK_TCP_OFFSET = 0.084721


def skew(omega: np.ndarray) -> np.ndarray:
    """3-vector -> 3x3 skew-symmetric matrix."""
    x, y, z = omega
    return np.array([[0.0, -z, y], [z, 0.0, -x], [-y, x, 0.0]])


def exp_screw(omega: np.ndarray, v: np.ndarray, theta: float) -> np.ndarray:
    """SE(3) exponential of a *unit* screw axis [omega, v] advanced by `theta`."""
    R = np.eye(3)
    p = np.zeros(3)
    if np.linalg.norm(omega) > 1e-12:
        w = skew(omega)
        R = np.eye(3) + math.sin(theta) * w + (1.0 - math.cos(theta)) * (w @ w)
        p = (np.eye(3) - R) @ np.cross(omega, v) + theta * omega * (omega @ v)
    else:
        p = v * theta
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = p
    return T


def exp_coords_to_se3(xi: list[float]) -> np.ndarray:
    """SE(3) exponential coordinates [omega*theta(3), v(3)] -> 4x4 transform."""
    omega_theta = np.asarray(xi[:3], dtype=float)
    v = np.asarray(xi[3:6], dtype=float)
    theta = float(np.linalg.norm(omega_theta))
    if theta < 1e-12:
        T = np.eye(4)
        T[:3, 3] = v
        return T
    return exp_screw(omega_theta / theta, v, theta)


def rotation_error(R: np.ndarray, R_target: np.ndarray) -> np.ndarray:
    """Rotation vector (axis * angle) that takes `R` to `R_target`.

    This is the log map of the relative rotation, and it is what a solver should use as the
    orientation residual. The common shortcut -- summing the cross products of corresponding
    columns -- has the same magnitude at 0 and 180 degrees, so a solution facing backwards
    looks converged. At 180 degrees the axis is ambiguous, so it is recovered from the
    symmetric part: R_rel + I = 2 * axis @ axis.T there.
    """
    R_rel = R_target @ R.T
    cos = (np.trace(R_rel) - 1.0) / 2.0
    angle = float(np.arccos(np.clip(cos, -1.0, 1.0)))
    if angle < 1e-12:
        return np.zeros(3)
    if angle > np.pi - 1e-6:
        symmetric = (R_rel + np.eye(3)) / 2.0
        index = int(np.argmax(np.diag(symmetric)))
        column = symmetric[:, index]
        norm = float(np.linalg.norm(column))
        axis = column / norm if norm > 1e-12 else np.array([1.0, 0.0, 0.0])
        return axis * angle
    axis = np.array([
        R_rel[2, 1] - R_rel[1, 2],
        R_rel[0, 2] - R_rel[2, 0],
        R_rel[1, 0] - R_rel[0, 1],
    ]) / (2.0 * np.sin(angle))
    return axis * angle


def rotation_to_rpy(R: np.ndarray, convention: str = "zyx") -> np.ndarray:
    """Extract roll-pitch-yaw from a rotation matrix.

    `convention` selects the order the angles are *applied*, and is the exact inverse of
    `rpy_to_rotation` for the same value:
      "xyz" -> R = Rx(rx) @ Ry(ry) @ Rz(rz)
      "zyx" -> R = Rz(rz) @ Ry(ry) @ Rx(rx)
    """
    if convention == "xyz":
        # R = Rx(rx) @ Ry(ry) @ Rz(rz)
        sb = float(np.clip(R[0, 2], -1.0, 1.0))
        if abs(sb) < 1.0 - 1e-9:
            return np.array([math.atan2(-R[1, 2], R[2, 2]),
                             math.asin(sb),
                             math.atan2(-R[0, 1], R[0, 0])])
        # Gimbal lock: ry = +/-90 deg, fold the remaining freedom into rx.
        return np.array([math.atan2(R[2, 1], R[1, 1]),
                         math.asin(sb),
                         0.0])
    if convention == "zyx":
        # R = Rz(rz) @ Ry(ry) @ Rx(rx)
        sb = float(np.clip(-R[2, 0], -1.0, 1.0))
        if abs(sb) < 1.0 - 1e-9:
            return np.array([math.atan2(R[2, 1], R[2, 2]),
                             math.asin(sb),
                             math.atan2(R[1, 0], R[0, 0])])
        return np.array([math.atan2(-R[1, 2], R[1, 1]),
                         math.asin(sb),
                         0.0])
    raise ValueError(f"unknown convention {convention!r}")


def rpy_to_rotation(rpy, convention: str = "zyx") -> np.ndarray:
    rx, ry, rz = rpy
    Rx = np.array([[1, 0, 0], [0, math.cos(rx), -math.sin(rx)], [0, math.sin(rx), math.cos(rx)]])
    Ry = np.array([[math.cos(ry), 0, math.sin(ry)], [0, 1, 0], [-math.sin(ry), 0, math.cos(ry)]])
    Rz = np.array([[math.cos(rz), -math.sin(rz), 0], [math.sin(rz), math.cos(rz), 0], [0, 0, 1]])
    return Rx @ Ry @ Rz if convention == "xyz" else Rz @ Ry @ Rx


@dataclass
class ArmKinematics:
    """Product-of-exponentials kinematics for one 7-DoF arm."""

    screws: np.ndarray  # (7, 6) unit screw axes [omega, v] in the arm base frame
    home: np.ndarray    # (4, 4) tool frame at the zero configuration
    limits: np.ndarray  # (7, 5) raw limit rows from the yml
    source: str

    @classmethod
    def from_yaml(cls, path: str) -> "ArmKinematics":
        with open(path) as handle:
            data = yaml.safe_load(handle)
        screws = np.array([data["screws"][i] for i in sorted(data["screws"])], dtype=float)
        home = exp_coords_to_se3(data["M"])
        limits = np.array([data["limits"][i] for i in sorted(data["limits"])], dtype=float)
        return cls(screws=screws, home=home, limits=limits, source=path)

    @property
    def dof(self) -> int:
        return len(self.screws)

    def fk(self, joints, tcp_offset: float = 0.0) -> np.ndarray:
        """Forward kinematics: 7 joint angles (rad) -> 4x4 tool transform in the arm base frame.

        `tcp_offset` shifts the result along the tool's local +z by that many metres. Use 0
        to match `get_tcp_pose` / `get_FK_pose`, or `IK_TCP_OFFSET` to match the frame that
        `get_IK_joint_position` solves for.
        """
        joints = np.asarray(joints, dtype=float)
        if joints.shape != (self.dof,):
            raise ValueError(f"expected {self.dof} joint values, got {joints.shape}")
        T = np.eye(4)
        for axis, theta in zip(self.screws, joints):
            T = T @ exp_screw(axis[:3], axis[3:], float(theta))
        T = T @ self.home
        if tcp_offset:
            T = T @ np.array([[1.0, 0, 0, 0], [0, 1.0, 0, 0], [0, 0, 1.0, tcp_offset], [0, 0, 0, 1.0]])
        return T

    def fk_pose(self, joints, convention: str = "zyx", tcp_offset: float = 0.0) -> np.ndarray:
        """FK as the vendor reports it: [x, y, z, rx, ry, rz]."""
        T = self.fk(joints, tcp_offset)
        return np.concatenate([T[:3, 3], rotation_to_rpy(T[:3, :3], convention)])

    def ik(self, pose, seed=None, convention: str = "zyx", tcp_offset: float = 0.0,
           tol: float = 1e-8, max_iter: int = 400, damping: float = 1e-3):
        """Damped-least-squares inverse kinematics.

        `pose` is [x, y, z, rx, ry, rz]. Returns (joints, converged). This is a plain
        numerical solver: it returns *a* solution near `seed`, not necessarily the same
        branch the vendor picks, and it does not do collision or limit avoidance beyond
        clamping to the yml limits.
        """
        target = np.asarray(pose, dtype=float)
        T_target = np.eye(4)
        T_target[:3, :3] = rpy_to_rotation(target[3:], convention)
        T_target[:3, 3] = target[:3]

        q = np.zeros(self.dof) if seed is None else np.array(seed, dtype=float).copy()
        lo = np.array([row[1] for row in self.limits])
        hi = np.array([row[0] for row in self.limits])
        # The yml stores limits in the order [upper, lower, ...]; normalise just in case.
        lo, hi = np.minimum(lo, hi), np.maximum(lo, hi)
        # Clip the seed before the first convergence test, so a seed that is out of limits
        # cannot be reported as a solution.
        q = np.clip(q, lo, hi)

        for _ in range(max_iter):
            T = self.fk(q, tcp_offset)
            err = np.empty(6)
            err[:3] = T_target[:3, 3] - T[:3, 3]
            err[3:] = rotation_error(T[:3, :3], T_target[:3, :3])
            if np.linalg.norm(err[:3]) < tol and np.linalg.norm(err[3:]) < tol:
                return q, True

            J = np.zeros((6, self.dof))
            eps = 1e-7
            for i in range(self.dof):
                dq = q.copy()
                dq[i] += eps
                Td = self.fk(dq, tcp_offset)
                J[:3, i] = (Td[:3, 3] - T[:3, 3]) / eps
                J[3:, i] = rotation_error(T[:3, :3], Td[:3, :3]) / eps

            # Levenberg-Marquardt step.
            A = J.T @ J + damping * np.eye(self.dof)
            dq = np.linalg.solve(A, J.T @ err)
            step = np.clip(dq, -0.2, 0.2)
            q = np.clip(q + step, lo, hi)
            if np.linalg.norm(step) < 1e-12:
                break

        # Judge convergence by the residual we actually reached, not by how the loop ended:
        # a clamped joint or a tiny step can exit early with the target already met. The
        # orientation test is a real angle: the previous cross-product sum also vanished at
        # 180 degrees, so a solution facing backwards was reported as converged.
        T = self.fk(q, tcp_offset)
        pos_err = float(np.linalg.norm(T_target[:3, 3] - T[:3, 3]))
        rot_err = float(np.linalg.norm(rotation_error(T[:3, :3], T_target[:3, :3])))
        return q, (pos_err < 1e-6 and rot_err < 1e-6)


def load_arm(arm: str, config_root=None, model: str = DEFAULT_MODEL) -> ArmKinematics:
    """Load one arm's kinematics from the vendor configuration.

    ``config_root`` defaults to the ``DUAL_ARM_SDK_CONFIG`` environment variable. There is
    no built-in fallback path, so the caller gets either a real configuration or a clear
    error explaining how to point at one -- see :mod:`vendor_model.config`.
    """
    if arm not in ("left", "right"):
        raise ValueError(f"arm must be 'left' or 'right', got {arm!r}")
    name = {"left": "leftArm", "right": "rightArm"}[arm]
    path = Path(resolve_config_root(config_root)) / model / f"kinematics_{name}.yml"
    if not path.is_file():
        raise FileNotFoundError(f"{path} not found; is model={model!r} correct?")
    return ArmKinematics.from_yaml(path)

