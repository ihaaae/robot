"""Tests for the Python side of the SDK bridge that need no bridge and no robot.

The bridge itself is aarch64, so everything that actually calls into the SDK is verified under
emulation (research/vendor-analysis/sdk.md section 9) or on the robot. What is checkable here is the marshalling
rules, the error messages, and that a missing bridge fails with instructions rather than with
an ImportError or a crash.
"""
from __future__ import annotations

import os
from pathlib import Path

import pytest

import juxie_sdk as sdk


def test_state_names_cover_the_five_states():
    assert sdk.state_name(0) == "power_off"
    assert sdk.state_name(2) == "idle"
    assert sdk.state_name(4) == "fault"
    # An unknown value is reported, not guessed at.
    assert sdk.state_name(9) == "state 9"


def test_bridge_codes_are_explained_and_sdk_codes_are_not():
    error = sdk.SdkError("juxie_get_dof", sdk.ERR_NO_ON_ROBOT)
    assert error.code == sdk.ERR_NO_ON_ROBOT
    assert "on_robot()" in str(error)

    # -1 is the SDK's own "refused"; the bridge must not dress it up as something else.
    sdk_code = sdk.SdkError("juxie_move_j", -1)
    assert sdk_code.code == -1
    assert "on_robot()" not in str(sdk_code)


def test_the_lifecycle_refusal_points_at_close():
    error = sdk.SdkError("juxie_on_robot", sdk.ERR_NEEDS_NEW_CONTROLLER)
    assert error.code == -1006
    assert "close()" in str(error)
    # Bridge codes stay below the SDK's own range (-104..0) and do not collide.
    codes = [value for name, value in vars(sdk).items() if name.startswith("ERR_")]
    assert len(codes) == len(set(codes))
    assert all(code <= -1000 for code in codes)


def test_doubles_validates_size_and_finiteness():
    assert sdk._doubles([0] * 17, 17, "joints").shape == (17,)

    with pytest.raises(ValueError, match="needs 17 values"):
        sdk._doubles([0] * 14, 17, "joints")
    with pytest.raises(ValueError, match="non-finite"):
        sdk._doubles([float("nan")] + [0] * 16, 17, "joints")


def test_bridge_path_prefers_the_environment(monkeypatch, tmp_path):
    fake = tmp_path / "juxie_sdk_bridge.so"
    fake.write_bytes(b"")
    monkeypatch.setenv("JUXIE_SDK_BRIDGE", str(fake))
    assert sdk.bridge_path() == fake


def test_a_missing_bridge_says_how_to_build_it(monkeypatch, tmp_path):
    monkeypatch.delenv("JUXIE_SDK_BRIDGE", raising=False)
    monkeypatch.chdir(tmp_path)
    monkeypatch.setattr(Path, "is_file", lambda self: False)
    with pytest.raises(FileNotFoundError, match="build_bridge.sh"):
        sdk.bridge_path()


def test_constructing_without_a_bridge_fails_before_loading(monkeypatch, tmp_path):
    monkeypatch.delenv("JUXIE_SDK_BRIDGE", raising=False)
    monkeypatch.chdir(tmp_path)
    monkeypatch.setattr(Path, "is_file", lambda self: False)
    with pytest.raises(FileNotFoundError):
        sdk.Controller()


# ------------------------------------------------------------------ the configuration root
# Without DUAL_ARM_SDK_CONFIG the SDK aborts the whole process, so the module points it at the
# tree the bridge was built against (recorded next to the bridge) rather than guessing.


def test_config_root_prefers_the_environment(monkeypatch, tmp_path):
    monkeypatch.setenv(sdk.CONFIG_ENV, str(tmp_path))
    assert sdk.config_root() == tmp_path


def test_config_root_comes_from_the_sidecar(monkeypatch, tmp_path):
    # config_root() exports the variable for the SDK to read, so give it a private environment
    # to write into instead of the test process's own.
    monkeypatch.setattr(os, "environ", dict(os.environ))
    bridge = tmp_path / "juxie_sdk_bridge.so"
    bridge.write_bytes(b"")
    etc = tmp_path / "sdk" / "usr" / "etc"
    etc.mkdir(parents=True)
    (etc / "params.yml").write_text("")
    bridge.with_suffix(".sdk").write_text(f"{tmp_path / 'sdk'}\n")

    monkeypatch.delenv(sdk.CONFIG_ENV, raising=False)
    monkeypatch.setattr(sdk, "bridge_path", lambda: bridge)

    assert sdk.config_root() == etc
    # and it is exported, because that is how the SDK reads it
    assert os.environ[sdk.CONFIG_ENV] == str(etc)


def test_config_root_without_a_sidecar_says_what_to_do(monkeypatch, tmp_path):
    bridge = tmp_path / "juxie_sdk_bridge.so"
    bridge.write_bytes(b"")
    monkeypatch.delenv(sdk.CONFIG_ENV, raising=False)
    monkeypatch.setattr(sdk, "bridge_path", lambda: bridge)

    with pytest.raises(sdk.ConfigNotFoundError, match="build_bridge.sh"):
        sdk.config_root()


def test_config_root_ignores_a_sidecar_pointing_at_nothing(monkeypatch, tmp_path):
    bridge = tmp_path / "juxie_sdk_bridge.so"
    bridge.write_bytes(b"")
    bridge.with_suffix(".sdk").write_text(f"{tmp_path / 'gone'}\n")
    monkeypatch.delenv(sdk.CONFIG_ENV, raising=False)
    monkeypatch.setattr(sdk, "bridge_path", lambda: bridge)

    with pytest.raises(sdk.ConfigNotFoundError):
        sdk.config_root()
