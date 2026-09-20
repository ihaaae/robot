"""Kinematics tests that use a synthetic arm and hardcoded constants, so they need no vendor
files at all.

The vendor YAML never enters the default test run: those comparisons live in
``tests/integration`` and are marked ``vendor_binary``.
"""
from __future__ import annotations

import math

import numpy as np
import pytest
import yaml

from shensi_robot.kinematics import (
    JOINT_COUNT,
    LEFT_ARM_SLICE,
    RIGHT_ARM_SLICE,
    VENDOR_CYCLIC_POSE,
    WAIST_INDEX,
    ArmKinematics,
    exp_coords_to_se3,
    exp_screw,
    rotation_to_rpy,
    rpy_to_rotation,
)


def make_planar_arm(tmp_path, link: float = 1.0) -> ArmKinematics:
    """Two joints both rotating about +z at the origin, with the tool at (link, 0, 0).

    Closed form: FK(q) puts the tool at (link*cos(q0+q1), link*sin(q0+q1), 0) with a
    rotation of q0+q1 about z.
    """
    config = {
        "screws": {
            0: [0, 0, 1, 0, 0, 0],
            1: [0, 0, 1, 0, 0, 0],
        },
        "M": [0, 0, 0, link, 0, 0],
        "limits": {
            0: [math.pi, -math.pi, math.pi, 1.0, 100.0],
            1: [math.pi, -math.pi, math.pi, 1.0, 100.0],
        },
    }
    path = tmp_path / "kinematics_leftArm.yml"
    path.write_text(yaml.safe_dump(config))
    return ArmKinematics.from_yaml(path)


def test_exp_coords_pure_translation():
    T = exp_coords_to_se3([0, 0, 0, 0.1, 0.2, 0.3])
    assert np.allclose(T[:3, :3], np.eye(3))
    assert np.allclose(T[:3, 3], [0.1, 0.2, 0.3])


def test_exp_screw_matches_analytic_rotation():
    # Rotating a point 1 unit out about the origin by 90 degrees should land on +y.
    T = exp_screw(np.array([0, 0, 1.0]), np.zeros(3), math.pi / 2)
    assert np.allclose(T @ np.array([1.0, 0, 0, 1]), [0, 1.0, 0, 1])


def test_fk_matches_closed_form(tmp_path):
    arm = make_planar_arm(tmp_path, link=0.5)
    for q0, q1 in [(0.0, 0.0), (0.3, 0.2), (-1.1, 0.7), (math.pi / 2, 0.0)]:
        T = arm.fk([q0, q1])
        total = q0 + q1
        assert np.allclose(T[:3, 3], [0.5 * math.cos(total), 0.5 * math.sin(total), 0.0],
                           atol=1e-12)


def test_dof_and_bad_input(tmp_path):
    arm = make_planar_arm(tmp_path)
    assert arm.dof == 2
    with pytest.raises(ValueError):
        arm.fk([0.0, 0.0, 0.0])


@pytest.mark.parametrize("convention", ["xyz", "zyx"])
def test_rpy_round_trip(convention):
    rng = np.random.default_rng(0)
    for _ in range(200):
        raw = rng.normal(size=(3, 3))
        Q, _ = np.linalg.qr(raw)
        if np.linalg.det(Q) < 0:
            Q[:, 0] *= -1
        back = rpy_to_rotation(rotation_to_rpy(Q, convention), convention)
        assert np.allclose(Q, back, atol=1e-10)


def test_ik_round_trip_uses_the_requested_offset(tmp_path):
    arm = make_planar_arm(tmp_path, link=0.5)
    for q0, q1 in [(0.4, -0.9), (-1.3, 0.6)]:
        pose = arm.fk_pose([q0, q1], "zyx", tcp_offset=0.0)
        solved, converged = arm.ik(pose, seed=[q0, q1], convention="zyx", tcp_offset=0.0)
        assert converged
        assert np.allclose(arm.fk(solved)[:3, 3], arm.fk([q0, q1])[:3, 3], atol=1e-6)


def test_ik_reports_failure_for_unreachable_target(tmp_path):
    arm = make_planar_arm(tmp_path, link=0.5)
    _, converged = arm.ik([5.0, 0.0, 0.0, 0.0, 0.0, 0.0], convention="zyx", tcp_offset=0.0)
    assert not converged


def test_tcp_offset_shifts_along_tool_z(tmp_path):
    arm = make_planar_arm(tmp_path, link=0.5)
    q = [0.0, 0.0]
    plain = arm.fk(q, tcp_offset=0.0)
    shifted = arm.fk(q, tcp_offset=0.1)
    assert np.allclose(shifted[:3, 3] - plain[:3, 3], [0.0, 0.0, 0.1])


def test_ik_rejects_a_solution_that_is_180_degrees_out(tmp_path):
    """The orientation residual must not vanish at 180 degrees.

    The previous residual -- the sum of cross products of corresponding rotation-matrix
    columns -- is zero both at 0 and at 180 degrees, so a target facing the opposite way was
    reported as converged whenever the position already matched.
    """
    arm = make_planar_arm(tmp_path, link=0.5)
    pose = [0.5, 0.0, 0.0, 0.0, 0.0, math.pi]
    solved, converged = arm.ik(pose, seed=[0.0, 0.0], convention="zyx")

    # Whatever it returns, the orientation must actually match before it claims success.
    R_target = rpy_to_rotation(pose[3:], "zyx")
    angle = np.degrees(np.arccos(np.clip(
        (np.trace(R_target.T @ arm.fk(solved)[:3, :3]) - 1.0) / 2.0, -1.0, 1.0)))
    assert not converged or angle < 1.0


def test_rotation_error_is_pi_at_180_degrees():
    """Directly: the residual at 180 degrees must be pi, not zero."""
    from shensi_robot.kinematics import rotation_error

    R = rpy_to_rotation([0.3, -0.2, 0.7], "zyx")
    R_flipped = R @ rpy_to_rotation([math.pi, 0.0, 0.0], "zyx")

    assert np.linalg.norm(rotation_error(R, R)) < 1e-12
    assert np.isclose(np.linalg.norm(rotation_error(R, R_flipped)), math.pi, atol=1e-6)


def test_ik_clips_an_out_of_limits_seed(tmp_path):
    """A seed outside the joint limits must not be returned as a solution."""
    arm = make_planar_arm(tmp_path, link=0.5)
    pose = arm.fk_pose([0.4, -0.9], "zyx", tcp_offset=0.0)
    solved, converged = arm.ik(pose, seed=[50.0, -50.0], convention="zyx")

    limits = np.array([[row[1], row[0]] for row in arm.limits])
    lo, hi = np.minimum(limits[:, 0], limits[:, 1]), np.maximum(limits[:, 0], limits[:, 1])
    assert np.all(solved >= lo - 1e-12) and np.all(solved <= hi + 1e-12)
    # A planar 2-DOF arm has mirrored solutions, so compare the pose, not the joints.
    if converged:
        reached = arm.fk_pose(solved, "zyx", tcp_offset=0.0)
        assert np.allclose(reached, pose, atol=1e-6)


# ------------------------------------------------------------------ vendor data constants
# These are constants in kinematics.py, recovered from the vendor's binary. They are asserted
# here because the C++ demo (examples/cpp/06_vendor_cyclic_motion.cpp) carries the same numbers.


def test_vendor_pose_matches_the_joint_layout():
    pose = VENDOR_CYCLIC_POSE
    assert len(pose) == JOINT_COUNT

    # waist and head are not driven on this machine
    assert pose[WAIST_INDEX] == 0.0
    assert pose[15] == 0.0 and pose[16] == 0.0

    # the binary's constant gives both arms the same pose
    assert pose[LEFT_ARM_SLICE] == pose[RIGHT_ARM_SLICE]
